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
        // Equal inputs keep their push order (the index breaks ties), as the
        // stable `sort_by` this replaces kept them. Sorting indices and
        // permuting in place needs 4 bytes an input, where sorting the
        // ~250-byte inputs themselves allocated a buffer as large as them.
        let inputs = &self.inputs;
        let mut order =
            (0..u32::try_from(inputs.len()).expect("fewer than 2^32 inputs")).collect::<Vec<u32>>();
        order.par_sort_unstable_by(|&left, &right| {
            let (l, r) = (&inputs[left as usize], &inputs[right as usize]);
            (
                l.layer,
                l.min_zoom,
                l.sort_order,
                l.feature_id,
                &l.attrs,
                left,
            )
                .cmp(&(
                    r.layer,
                    r.min_zoom,
                    r.sort_order,
                    r.feature_id,
                    &r.attrs,
                    right,
                ))
        });
        permute_in_place(&mut self.inputs, &order);
        drop(order);

        let mut network = SharedLineNetwork::default();
        network.stats.input_lines = self.inputs.len();

        // Sorted by layer, each layer's inputs are one run: the later layers
        // are split off the end, and the first keeps the inputs' buffer, so no
        // input is copied. The buckets are consumed in `SHARED_LINE_LAYERS`
        // order (the layers' sort order) so topology node and edge ids are
        // assigned exactly as before.
        let mut buckets: [Vec<SharedLineInput>; SHARED_LINE_LAYERS.len()] =
            std::array::from_fn(|_| Vec::new());
        for (slot, layer) in SHARED_LINE_LAYERS.iter().enumerate().rev() {
            let start = self.inputs.partition_point(|input| input.layer < *layer);
            buckets[slot] = if start == 0 {
                std::mem::take(&mut self.inputs)
            } else {
                self.inputs.split_off(start)
            };
        }
        debug_assert!(self.inputs.is_empty());

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

/// Puts `items[order[i]]` at `i`, following each cycle of the permutation
/// with swaps.
pub(crate) fn permute_in_place<T>(items: &mut [T], order: &[u32]) {
    debug_assert_eq!(items.len(), order.len());
    let mut placed = vec![false; items.len()];
    for start in 0..items.len() {
        if placed[start] {
            continue;
        }
        let mut at = start;
        loop {
            placed[at] = true;
            let from = order[at] as usize;
            if from == start {
                break;
            }
            items.swap(at, from);
            at = from;
        }
    }
}

/// Layer processing order. `append_layer_network` assigns topology node and
/// edge ids by appending, so this order is part of the output contract (and
/// is the layers' sort order).
const SHARED_LINE_LAYERS: [SharedLineLayer; 3] = [
    SharedLineLayer::Roads,
    SharedLineLayer::WaterLines,
    SharedLineLayer::Railways,
];
