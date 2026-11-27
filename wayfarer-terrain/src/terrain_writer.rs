//! A streaming writer of terrain files: the same bytes as [`Terrain::write`], with one block
//! in memory at a time.
//!
//! The header, the sources text and an index of zero lengths are written first, from the
//! block list known up front; payloads follow in key order as blocks are pushed, each block's
//! hash is folded as it goes, and [`TerrainWriter::finish`] patches the payload lengths into
//! the index and appends the terrain hash. Memory is the index (12 bytes a block) and one
//! block's hashes, whatever the region's size.
//!
//! [`Terrain::write`]: crate::terrain::Terrain::write

use crate::codec::put_varint;
use crate::lattice::{BLOCK_POSTS, BlockId, Spacing};
use crate::lines::BASE_BITS;
use crate::terrain::{
    MAGIC, MAJOR, MAX_SOURCES, MINOR, STEEP_NONE, block_hash, block_payload, terrain_hash,
};
use crate::{Error, Result};
use std::io::{Seek, SeekFrom, Write};

/// Writes a terrain file block by block to a seekable sink, such as a file.
pub struct TerrainWriter<W: Write + Seek> {
    out: W,
    spacing: Spacing,
    blocks: Vec<BlockId>,
    lens: Vec<u32>,
    hashes: Vec<[u8; 32]>,
    index_at: u64,
    bytes: u64,
}

fn io(e: std::io::Error) -> Error {
    Error::invalid(format!("terrain file i/o: {e}"))
}

impl<W: Write + Seek> TerrainWriter<W> {
    /// Starts a file for `blocks` (sorted and unique) at the sink's current position.
    pub fn new(mut out: W, spacing: Spacing, blocks: Vec<BlockId>, sources: &str) -> Result<Self> {
        if blocks.windows(2).any(|w| w[0] >= w[1]) {
            return Err(Error::invalid("terrain blocks must be sorted and unique"));
        }
        if sources.len() > MAX_SOURCES {
            return Err(Error::invalid(format!(
                "terrain sources text over {MAX_SOURCES} bytes"
            )));
        }
        let n =
            u32::try_from(blocks.len()).map_err(|_| Error::invalid("too many terrain blocks"))?;
        let start = out.stream_position().map_err(io)?;
        let mut head = Vec::with_capacity(32 + sources.len());
        head.extend_from_slice(MAGIC);
        head.extend_from_slice(&MAJOR.to_le_bytes());
        head.extend_from_slice(&MINOR.to_le_bytes());
        head.extend_from_slice(&(spacing.ticks() as u32).to_le_bytes());
        head.extend_from_slice(&n.to_le_bytes());
        put_varint(&mut head, sources.len() as u64);
        head.extend_from_slice(sources.as_bytes());
        out.write_all(&head).map_err(io)?;
        let mut index = Vec::with_capacity(12 * blocks.len());
        for b in &blocks {
            index.extend_from_slice(&b.key().to_le_bytes());
            index.extend_from_slice(&0u32.to_le_bytes());
        }
        out.write_all(&index).map_err(io)?;
        Ok(TerrainWriter {
            out,
            spacing,
            lens: Vec::with_capacity(blocks.len()),
            hashes: Vec::with_capacity(blocks.len()),
            blocks,
            index_at: start + head.len() as u64,
            bytes: (head.len() + index.len()) as u64,
        })
    }

    /// The blocks still to push, in order.
    pub fn remaining(&self) -> &[BlockId] {
        &self.blocks[self.lens.len()..]
    }

    /// Writes the next block's planes, `BLOCK_POSTS` posts each, with the value ranges
    /// [`crate::terrain::Terrain::new`] checks.
    pub fn push(
        &mut self,
        dm: &[i32],
        steep: &[u8],
        cover: &[u8],
        conf: &[u8],
        lines: &[u16],
    ) -> Result<()> {
        let Some(&id) = self.blocks.get(self.lens.len()) else {
            return Err(Error::invalid("more terrain blocks pushed than declared"));
        };
        if [dm.len(), steep.len(), cover.len(), conf.len(), lines.len()]
            .iter()
            .any(|&l| l != BLOCK_POSTS)
        {
            return Err(Error::invalid(format!(
                "terrain planes must hold {BLOCK_POSTS} posts"
            )));
        }
        if steep.iter().any(|&s| s > 15 && s != STEEP_NONE) {
            return Err(Error::invalid("steepness class above 15"));
        }
        if cover.iter().any(|&c| c > 15) || conf.iter().any(|&c| c > 1) {
            return Err(Error::invalid(
                "land cover is a 4-bit code and a confidence bit",
            ));
        }
        if lines.iter().any(|&l| l & !BASE_BITS != 0) {
            return Err(Error::invalid(
                "line mask uses bits reserved for surface effects",
            ));
        }
        let p = block_payload(dm, steep, cover, conf, lines);
        let len = u32::try_from(p.len()).map_err(|_| Error::invalid("terrain block too large"))?;
        self.out.write_all(&p).map_err(io)?;
        self.bytes += p.len() as u64;
        self.lens.push(len);
        self.hashes
            .push(block_hash(id, dm, steep, cover, conf, lines));
        Ok(())
    }

    /// Patches the index and appends the terrain hash. Returns the hash, the file's length in
    /// bytes and the sink, positioned at the file's end.
    pub fn finish(mut self) -> Result<([u8; 32], u64, W)> {
        if self.lens.len() != self.blocks.len() {
            return Err(Error::invalid(format!(
                "{} of {} terrain blocks pushed",
                self.lens.len(),
                self.blocks.len()
            )));
        }
        let hash = terrain_hash(self.spacing, &self.hashes);
        self.out.write_all(&hash).map_err(io)?;
        let end = self.out.stream_position().map_err(io)?;
        self.out.seek(SeekFrom::Start(self.index_at)).map_err(io)?;
        let mut index = Vec::with_capacity(12 * self.blocks.len());
        for (b, l) in self.blocks.iter().zip(&self.lens) {
            index.extend_from_slice(&b.key().to_le_bytes());
            index.extend_from_slice(&l.to_le_bytes());
        }
        self.out.write_all(&index).map_err(io)?;
        self.out.seek(SeekFrom::Start(end)).map_err(io)?;
        self.out.flush().map_err(io)?;
        Ok((hash, self.bytes + 32, self.out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::{DEM_NONE, Terrain};
    use std::io::Cursor;

    #[test]
    fn same_bytes_as_terrain_write() {
        let blocks = vec![
            BlockId { bi: -3, bj: 7 },
            BlockId { bi: -3, bj: 8 },
            BlockId { bi: 2, bj: 7 },
        ];
        let n = blocks.len() * BLOCK_POSTS;
        let dm: Vec<i32> = (0..n)
            .map(|i| {
                if i % 997 == 5 || i / BLOCK_POSTS == 1 {
                    DEM_NONE
                } else {
                    8_000 + (i % 64) as i32 * 3 - (i / 64 % 64) as i32 * 7
                }
            })
            .collect();
        let steep: Vec<u8> = (0..n)
            .map(|i| {
                if dm[i] == DEM_NONE {
                    STEEP_NONE
                } else {
                    (i % 16) as u8
                }
            })
            .collect();
        let cover: Vec<u8> = (0..n).map(|i| (i / 100 % 16) as u8).collect();
        let conf: Vec<u8> = (0..n).map(|i| (i / 300 % 2) as u8).collect();
        let lines: Vec<u16> = (0..n)
            .map(|i| if i % 64 == 10 { crate::lines::PATH } else { 0 })
            .collect();
        let s = Spacing::new(120).unwrap();
        let sources = "GEM-15 DTM (CC BY 4.0): produced by Google";
        let t = Terrain::new(
            s,
            blocks.clone(),
            dm.clone(),
            steep.clone(),
            cover.clone(),
            conf.clone(),
            lines.clone(),
        )
        .unwrap()
        .with_sources(sources)
        .unwrap();
        let mut w =
            TerrainWriter::new(Cursor::new(Vec::new()), s, blocks.clone(), sources).unwrap();
        for (k, b) in blocks.iter().enumerate() {
            assert_eq!(w.remaining()[0], *b);
            let r = k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS;
            w.push(
                &dm[r.clone()],
                &steep[r.clone()],
                &cover[r.clone()],
                &conf[r.clone()],
                &lines[r],
            )
            .unwrap();
        }
        let (hash, len, out) = w.finish().unwrap();
        let bytes = out.into_inner();
        assert_eq!(hash, t.hash());
        assert_eq!(len as usize, bytes.len());
        assert_eq!(bytes, t.write());
        assert_eq!(Terrain::read(&bytes).unwrap(), t);
    }

    #[test]
    fn refuses_wrong_pushes() {
        let s = Spacing::new(120).unwrap();
        let b = vec![BlockId { bi: 0, bj: 0 }];
        let z = vec![0u8; BLOCK_POSTS];
        let mut w = TerrainWriter::new(Cursor::new(Vec::new()), s, b.clone(), "").unwrap();
        assert!(
            w.push(
                &[0; BLOCK_POSTS],
                &[16; BLOCK_POSTS],
                &z,
                &z,
                &[0; BLOCK_POSTS]
            )
            .is_err()
        );
        assert!(w.push(&[0; 5], &z, &z, &z, &[0; BLOCK_POSTS]).is_err());
        assert!(TerrainWriter::new(Cursor::new(Vec::new()), s, vec![b[0], b[0]], "").is_err());
        let long = "x".repeat(MAX_SOURCES + 1);
        assert!(TerrainWriter::new(Cursor::new(Vec::new()), s, b.clone(), &long).is_err());
        let w = TerrainWriter::new(Cursor::new(Vec::new()), s, b, "").unwrap();
        assert!(w.finish().is_err());
    }
}
