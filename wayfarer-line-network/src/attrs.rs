//! Attribute lists: interning, normalization, merge signatures and lookups.

use super::*;

/// Deduplicates whole tag lists handed to render seeds.
///
/// Seed attrs and merge signatures repeat heavily across a layer — most roads
/// share a class/subclass/brunnel combination, and many share their whole tag
/// list — so interning by content collapses a per-seed allocation into one per
/// distinct list.
#[derive(Debug, Default)]
pub(crate) struct AttrListInterner {
    lists: HashMap<Vec<AttrPair>, Arc<[AttrPair]>>,
}

type AttrPair = (String, String);

impl AttrListInterner {
    pub(crate) fn intern(&mut self, list: Vec<(String, String)>) -> Arc<[(String, String)]> {
        if let Some(existing) = self.lists.get(&list) {
            return Arc::clone(existing);
        }
        let shared: Arc<[(String, String)]> = Arc::from(list.clone());
        self.lists.insert(list, Arc::clone(&shared));
        shared
    }

    /// [`Self::intern`] for a borrowed list, copied only when it is new.
    pub(crate) fn intern_slice(&mut self, list: &[(String, String)]) -> Arc<[(String, String)]> {
        match self.lists.get(list) {
            Some(existing) => Arc::clone(existing),
            None => self.intern(list.to_vec()),
        }
    }
}

/// Whether [`normalize_attrs`] would leave `attrs` unchanged: strictly
/// increasing means sorted with no duplicates.
pub(crate) fn attrs_are_normalized(attrs: &[(String, String)]) -> bool {
    attrs.windows(2).all(|pair| pair[0] < pair[1])
}

pub(crate) fn merge_signature(
    layer: SharedLineLayer,
    attrs: &[(String, String)],
) -> Vec<(String, String)> {
    let keys: &[&str] = match layer {
        SharedLineLayer::Roads => &[
            "class", "subclass", "rank", "name", "ref", "bridge", "tunnel", "layer",
        ],
        SharedLineLayer::WaterLines => &["class", "name", "bridge", "tunnel", "layer"],
        SharedLineLayer::Railways => &["class", "service", "bridge", "tunnel", "layer"],
    };
    attrs
        .iter()
        .filter(|(key, _)| keys.contains(&key.as_str()))
        .cloned()
        .collect()
}

pub(crate) fn common_attrs(
    left: &[(String, String)],
    right: &[(String, String)],
) -> Vec<(String, String)> {
    let mut merged = Vec::new();
    let mut left_index = 0usize;
    let mut right_index = 0usize;
    while left_index < left.len() && right_index < right.len() {
        match left[left_index].0.cmp(&right[right_index].0) {
            std::cmp::Ordering::Less => left_index += 1,
            std::cmp::Ordering::Greater => right_index += 1,
            std::cmp::Ordering::Equal => {
                if left[left_index].1 == right[right_index].1 {
                    merged.push(left[left_index].clone());
                }
                left_index += 1;
                right_index += 1;
            }
        }
    }
    merged
}

pub(crate) fn merge_unique_into<T>(target: &mut Vec<T>, incoming: Vec<T>)
where
    T: Ord,
{
    if incoming.is_empty() {
        return;
    }
    let mut merged = BTreeSet::new();
    merged.extend(target.drain(..));
    merged.extend(incoming);
    *target = merged.into_iter().collect();
}

pub(crate) fn normalize_attrs(attrs: &mut Vec<(String, String)>) {
    attrs.sort();
    attrs.dedup();
}

pub(crate) fn attr_value<'a>(attrs: &'a [(String, String)], key: &str) -> Option<&'a str> {
    attrs
        .iter()
        .find(|(attr_key, _)| attr_key == key)
        .map(|(_, value)| value.as_str())
}
