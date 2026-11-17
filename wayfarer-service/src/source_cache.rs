//! The service's cache of opened sources: a handful of entries (4 by
//! default), so a vector in recency order beats a linked hash map.

/// A least-recently-used cache holding at most `capacity` entries, the most
/// recently used last.
#[derive(Debug)]
pub(crate) struct SourceCache<K, V> {
    capacity: usize,
    entries: Vec<(K, V)>,
}

impl<K: PartialEq, V> SourceCache<K, V> {
    /// A cache of at most `capacity` entries (at least one).
    pub(crate) fn new(capacity: usize) -> Self {
        let capacity = capacity.max(1);
        Self {
            capacity,
            entries: Vec::with_capacity(capacity.min(64)),
        }
    }

    /// The entry for `key`, which becomes the most recently used.
    pub(crate) fn get(&mut self, key: &K) -> Option<&V> {
        let at = self.entries.iter().position(|(k, _)| k == key)?;
        let entry = self.entries.remove(at);
        self.entries.push(entry);
        self.entries.last().map(|(_, v)| v)
    }

    /// Removes and returns the entry for `key`.
    pub(crate) fn pop(&mut self, key: &K) -> Option<V> {
        let at = self.entries.iter().position(|(k, _)| k == key)?;
        Some(self.entries.remove(at).1)
    }

    /// Inserts `value` as the most recently used entry. Returns the entry it
    /// displaced: the old value under the same key, or else the least
    /// recently used entry when the cache was full.
    pub(crate) fn push(&mut self, key: K, value: V) -> Option<(K, V)> {
        let displaced = match self.entries.iter().position(|(k, _)| *k == key) {
            Some(at) => Some(self.entries.remove(at)),
            None if self.entries.len() >= self.capacity => Some(self.entries.remove(0)),
            None => None,
        };
        self.entries.push((key, value));
        displaced
    }
}

#[cfg(test)]
mod tests {
    use super::SourceCache;

    #[test]
    fn evicts_the_least_recently_used_entry() {
        let mut cache = SourceCache::new(2);
        assert_eq!(cache.push("a", 1), None);
        assert_eq!(cache.push("b", 2), None);
        // Reading `a` makes `b` the oldest.
        assert_eq!(cache.get(&"a"), Some(&1));
        assert_eq!(cache.push("c", 3), Some(("b", 2)));
        assert_eq!(cache.get(&"b"), None);
        assert_eq!(cache.get(&"a"), Some(&1));
        assert_eq!(cache.get(&"c"), Some(&3));
    }

    #[test]
    fn push_on_a_present_key_returns_the_old_value() {
        let mut cache = SourceCache::new(2);
        cache.push("a", 1);
        cache.push("b", 2);
        assert_eq!(cache.push("a", 10), Some(("a", 1)));
        // `b` is untouched and `a` is now the most recent.
        assert_eq!(cache.push("c", 3), Some(("b", 2)));
        assert_eq!(cache.pop(&"a"), Some(10));
        assert_eq!(cache.pop(&"a"), None);
    }

    #[test]
    fn capacity_is_at_least_one() {
        let mut cache = SourceCache::new(0);
        assert_eq!(cache.push("a", 1), None);
        assert_eq!(cache.push("b", 2), Some(("a", 1)));
    }
}
