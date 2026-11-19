//! Per-id search state that grows with the search, not with the graph.
//!
//! The searches keep never-cleared sparse sets indexed by edge, node or CCH
//! rank (label slots, node bounds, potential slots). As dense vectors these
//! cost a few bytes per id of the whole package on the first search, however
//! small it is. [`PagedSlots`] splits the id space into pages of
//! [`PAGE_LEN`] ids, allocated (filled with the array's empty value) the first
//! time one of their ids is written, in one arena per array. Node and edge ids are numbered along a
//! Hilbert curve and CCH ranks by nested dissection, so a search's ids fall
//! into few pages.

/// Ids per page: 2^10, so 4 KiB of `u32` slots (one memory page on most
/// systems) or 16 KiB of node bounds.
pub(crate) const PAGE_BITS: u32 = 10;
pub(crate) const PAGE_LEN: usize = 1 << PAGE_BITS;
const MASK: usize = PAGE_LEN - 1;

/// A fixed-length array of `T` whose ids read as `empty` until written.
///
/// The pages live in one arena (`data`) in the order they were first
/// written; `dir` holds each id range's page number there, 0 being a page
/// that is never written (every id of an unwritten range reads it). A read
/// is two dependent loads and no branch, which keeps the searches' hot loops
/// as fast as with a dense vector.
pub(crate) struct PagedSlots<T: Copy> {
    dir: Vec<u32>,
    data: Vec<T>,
    empty: T,
}

impl<T: Copy> PagedSlots<T> {
    /// An array of `len` ids, all `empty`; allocates the page directory (4
    /// bytes per page) and the shared empty page.
    pub(crate) fn new(len: usize, empty: T) -> Self {
        Self {
            dir: vec![0; len.div_ceil(PAGE_LEN)],
            data: vec![empty; PAGE_LEN],
            empty,
        }
    }

    /// Ids this array holds room for (a multiple of the page length).
    pub(crate) fn capacity(&self) -> usize {
        self.dir.len() * PAGE_LEN
    }

    /// Makes room for `len` ids (keeps the pages already written).
    pub(crate) fn ensure_len(&mut self, len: usize) {
        let pages = len.div_ceil(PAGE_LEN);
        if self.dir.len() < pages {
            self.dir.resize(pages, 0);
        }
    }

    #[inline]
    fn index(&self, id: u32) -> usize {
        let id = id as usize;
        ((self.dir[id >> PAGE_BITS] as usize) << PAGE_BITS) | (id & MASK)
    }

    /// The value of `id` (`empty` if never written since the page was
    /// released).
    #[inline]
    pub(crate) fn get(&self, id: u32) -> T {
        self.data[self.index(id)]
    }

    /// The value of `id`, writable; allocates its page.
    #[inline]
    pub(crate) fn get_mut(&mut self, id: u32) -> &mut T {
        let page = (id as usize) >> PAGE_BITS;
        if self.dir[page] == 0 {
            self.allocate(page);
        }
        let index = self.index(id);
        &mut self.data[index]
    }

    #[cold]
    fn allocate(&mut self, page: usize) {
        let len = self.data.len();
        if len == self.data.capacity() {
            // Grow geometrically, but never past every page plus the empty
            // one, so a search reaching the whole graph holds no more than a
            // dense vector would.
            let most = (self.dir.len() + 1) * PAGE_LEN;
            let target = (len * 2).max(len + PAGE_LEN).min(most);
            self.data.reserve_exact(target - len);
        }
        self.dir[page] = (len >> PAGE_BITS) as u32;
        self.data.resize(len + PAGE_LEN, self.empty);
    }

    #[inline]
    pub(crate) fn set(&mut self, id: u32, value: T) {
        *self.get_mut(id) = value;
    }

    /// Sets `id` back to `empty` without allocating its page (an unwritten
    /// id lands on the empty page, which then stays empty).
    #[inline]
    pub(crate) fn clear(&mut self, id: u32) {
        let index = self.index(id);
        self.data[index] = self.empty;
    }

    /// Bytes held for written pages (the arena past the empty page).
    pub(crate) fn page_bytes(&self) -> usize {
        self.data.capacity().saturating_sub(PAGE_LEN) * size_of::<T>()
    }

    #[cfg(test)]
    fn pages_written(&self) -> usize {
        self.data.len() / PAGE_LEN - 1
    }

    /// Moves the array out, leaving an empty placeholder of no length (not
    /// to be read) in its place.
    pub(crate) fn take(&mut self) -> Self {
        let empty = self.empty;
        std::mem::replace(
            self,
            Self {
                dir: Vec::new(),
                data: Vec::new(),
                empty,
            },
        )
    }

    /// Frees every page: every id reads as `empty` again.
    pub(crate) fn release(&mut self) {
        if self.data.capacity() > PAGE_LEN {
            self.dir.fill(0);
            self.data.truncate(PAGE_LEN);
            self.data.shrink_to_fit();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pages_allocate_on_write_and_release() {
        let mut slots = PagedSlots::new(3 * PAGE_LEN + 5, u32::MAX);
        assert_eq!(slots.capacity(), 4 * PAGE_LEN);
        assert_eq!(slots.get(0), u32::MAX);
        assert_eq!(slots.get((3 * PAGE_LEN + 4) as u32), u32::MAX);
        assert_eq!(slots.page_bytes(), 0);
        slots.clear(7);
        assert_eq!(slots.page_bytes(), 0);
        slots.set(7, 1);
        slots.set((PAGE_LEN - 1) as u32, 2);
        assert_eq!(slots.pages_written(), 1);
        assert_eq!(slots.page_bytes(), PAGE_LEN * 4);
        *slots.get_mut((2 * PAGE_LEN) as u32) = 3;
        assert_eq!(slots.pages_written(), 2);
        assert_eq!(slots.get(7), 1);
        assert_eq!(slots.get(8), u32::MAX);
        assert_eq!(slots.get((PAGE_LEN - 1) as u32), 2);
        assert_eq!(slots.get((2 * PAGE_LEN) as u32), 3);
        slots.clear(7);
        assert_eq!(slots.get(7), u32::MAX);
        slots.release();
        assert_eq!(slots.page_bytes(), 0);
        assert_eq!(slots.pages_written(), 0);
        assert_eq!(slots.get((PAGE_LEN - 1) as u32), u32::MAX);
        slots.ensure_len(8 * PAGE_LEN);
        slots.set((8 * PAGE_LEN - 1) as u32, 4);
        assert_eq!(slots.get((8 * PAGE_LEN - 1) as u32), 4);
        assert_eq!(slots.get(7), u32::MAX);
        // Every page written: the arena holds them and the empty page only.
        for id in (0..8 * PAGE_LEN).step_by(PAGE_LEN) {
            slots.set(id as u32, 5);
        }
        assert_eq!(slots.pages_written(), 8);
        assert_eq!(slots.page_bytes(), 8 * PAGE_LEN * 4);
        assert_eq!(slots.get((8 * PAGE_LEN - 1) as u32), 4);
    }

    #[test]
    fn node_bounds_pages_hold_pairs() {
        let mut bounds = PagedSlots::new(10, (f64::NAN, f64::NAN));
        assert!(bounds.get(3).0.is_nan());
        bounds.set(3, (1.0, 2.0));
        assert_eq!(bounds.get(3), (1.0, 2.0));
        assert_eq!(bounds.page_bytes(), PAGE_LEN * 16);
    }
}
