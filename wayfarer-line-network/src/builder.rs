//! [`SharedLineNetworkBuilder`]: pushed inputs to a [`SharedLineNetwork`], layer
//! by layer.

use super::*;

#[derive(Debug, Default)]
pub struct SharedLineNetworkBuilder {
    inputs: Vec<SharedLineInput>,
    /// Interns each input's tag lists on the way in, so the buffered inputs,
    /// the raw lines and the topology edges built from them all share one
    /// allocation per distinct list rather than one per feature.
    attrs: AttrListInterner,
}

impl SharedLineNetworkBuilder {
    pub fn push_input(&mut self, mut input: SharedLineInput) {
        input.attrs = if attrs_are_normalized(&input.attrs) {
            self.attrs.intern_slice(&input.attrs)
        } else {
            let mut attrs = input.attrs.to_vec();
            normalize_attrs(&mut attrs);
            self.attrs.intern(attrs)
        };
        input.compile_inputs = self.attrs.intern_slice(&input.compile_inputs);
        if input.source_lineage_ids.is_empty() {
            input.source_lineage_ids.push(input.feature_id);
        }
        input.source_lineage_ids.sort_unstable();
        input.source_lineage_ids.dedup();
        self.inputs.push(input);
    }

    pub fn extend_inputs<I>(&mut self, inputs: I)
    where
        I: IntoIterator<Item = SharedLineInput>,
    {
        for input in inputs {
            self.push_input(input);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.inputs.is_empty()
    }

    /// The inputs pushed so far, in push order: for a caller that pushes its
    /// lines before their geometry is known and fills it in afterwards.
    pub fn inputs_mut(&mut self) -> &mut [SharedLineInput] {
        &mut self.inputs
    }

    /// Keeps only the inputs `keep` accepts, in push order.
    pub fn retain_inputs(&mut self, keep: impl FnMut(&SharedLineInput) -> bool) {
        self.inputs.retain(keep);
    }

    pub fn finish(self) -> SharedLineNetwork {
        self.finish_with_observer(&mut |_| {})
    }

    /// Build topology only: identical nodes, edges and topology-repair stats
    /// to [`finish`](Self::finish), without deriving render products. For
    /// consumers such as the routing compiler that would discard them.
    pub fn finish_topology_only(self) -> SharedLineNetwork {
        self.finish_inner(&mut |_| {}, false)
    }

    /// Build the network, reporting a named checkpoint at each internal phase.
    ///
    /// This is one call that dominates a build's memory, and its phases hold
    /// very different things: assembled raw lines, the topology being appended,
    /// and the render products derived from it. A caller with allocation
    /// counters can attribute the peak to a phase instead of to the call.
    pub fn finish_with_observer(self, observer: &mut dyn FnMut(&str)) -> SharedLineNetwork {
        self.finish_inner(observer, true)
    }

    pub fn finish_topology_only_with_observer(
        self,
        observer: &mut dyn FnMut(&str),
    ) -> SharedLineNetwork {
        self.finish_inner(observer, false)
    }

    fn finish_inner(
        mut self,
        observer: &mut dyn FnMut(&str),
        render_products: bool,
    ) -> SharedLineNetwork {
        // Layer processing order decides topology id assignment; do not reorder.
        // Stable, as `sort_by` was, so equal inputs keep their push order.
        self.inputs.par_sort_by(|left, right| {
            (
                left.layer,
                left.min_zoom,
                left.sort_order,
                left.feature_id,
                &left.attrs,
            )
                .cmp(&(
                    right.layer,
                    right.min_zoom,
                    right.sort_order,
                    right.feature_id,
                    &right.attrs,
                ))
        });

        let mut network = SharedLineNetwork::default();
        network.stats.input_lines = self.inputs.len();

        // Move each input into its layer's bucket rather than cloning the
        // matching ones per layer. The clone was cheap in time (~2% of this
        // function) but duplicated every input's geometry, node chain and tag
        // strings, which is a large share of the build's peak memory. Draining
        // preserves the sorted order within each layer, and the buckets are
        // consumed in `SHARED_LINE_LAYERS` order so topology node and edge ids
        // are assigned exactly as before.
        let mut buckets: [Vec<SharedLineInput>; SHARED_LINE_LAYERS.len()] =
            std::array::from_fn(|_| Vec::new());
        for input in self.inputs.drain(..) {
            if let Some(slot) = SHARED_LINE_LAYERS
                .iter()
                .position(|candidate| *candidate == input.layer)
            {
                buckets[slot].push(input);
            }
        }
        drop(self.inputs);

        observer("inputs_bucketed");
        for layer_inputs in buckets {
            if layer_inputs.is_empty() {
                continue;
            }
            append_layer_network(&mut network, layer_inputs, observer, render_products);
        }

        observer("layers_appended");
        network.stats.topology_nodes = network.topology.nodes.len();
        network.stats.topology_edges = network.topology.edges.len();
        network.stats.render_lines = network.render.render_lines.len();
        network.render.render_lines.sort_by(|left, right| {
            (
                left.layer,
                left.min_zoom,
                left.sort_order,
                left.feature_id,
                left.start_node,
                left.end_node,
            )
                .cmp(&(
                    right.layer,
                    right.min_zoom,
                    right.sort_order,
                    right.feature_id,
                    right.start_node,
                    right.end_node,
                ))
        });
        network
    }
}

/// Layer processing order. `append_layer_network` assigns topology node and
/// edge ids by appending, so this order is part of the output contract.
const SHARED_LINE_LAYERS: [SharedLineLayer; 3] = [
    SharedLineLayer::Roads,
    SharedLineLayer::WaterLines,
    SharedLineLayer::Railways,
];
