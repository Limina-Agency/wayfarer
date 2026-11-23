//! A format-3 package taken apart into its header and table entries and put
//! back together, without the checks [`crate::format::PackageFile`] makes:
//! for tests, the fuzz targets and tools that add or strip sections. Not a
//! stable API.

use crate::format::{
    HEADER_FIXED_BYTES, HEADER_ROOT_CHECKSUM, HEADER_SECTION_COUNT, HEADER_V3_ROOT_CHECKSUM,
    HEADER_V3_TABLE_OFFSET, HEADER_VERSION, SECTION_ALIGN, TABLE_ENTRY_BYTES, TABLE_ENTRY_BYTES_V3,
};

/// One table entry with its section's bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawSection {
    pub id: u32,
    pub param: u32,
    pub flags: u32,
    pub element_bytes: u32,
    pub bytes: Vec<u8>,
}

/// A format-3 package as a header, entries in table order and where the
/// table goes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawPackage {
    /// The 256-byte header; [`Self::encode`] sets the entry count, the table
    /// offset and the root checksum.
    pub header: Vec<u8>,
    /// In table order, which is also their order in the file.
    pub sections: Vec<RawSection>,
    /// The table right after the header instead of after the sections.
    pub table_first: bool,
}

impl RawPackage {
    /// Takes a format-3 package apart; `None` when its header or table is
    /// out of bounds (nothing else is checked).
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let header = bytes.get(..HEADER_FIXED_BYTES)?.to_vec();
        let u32_at = |bytes: &[u8], at: usize| {
            Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?))
        };
        let u64_at = |bytes: &[u8], at: usize| {
            Some(u64::from_le_bytes(bytes.get(at..at + 8)?.try_into().ok()?))
        };
        if u32_at(&header, HEADER_VERSION)? != 3 {
            return None;
        }
        let count = u32_at(&header, HEADER_SECTION_COUNT)? as usize;
        let table_offset = usize::try_from(u64_at(&header, HEADER_V3_TABLE_OFFSET)?).ok()?;
        let table = bytes.get(
            table_offset..table_offset.checked_add(count.checked_mul(TABLE_ENTRY_BYTES_V3)?)?,
        )?;
        let mut sections = Vec::with_capacity(count);
        let mut first_offset = usize::MAX;
        for entry in table.as_chunks::<TABLE_ENTRY_BYTES_V3>().0 {
            let offset = usize::try_from(u64_at(entry, 16)?).ok()?;
            let len = usize::try_from(u64_at(entry, 24)?).ok()?;
            first_offset = first_offset.min(offset);
            sections.push(RawSection {
                id: u32_at(entry, 0)?,
                param: u32_at(entry, 4)?,
                flags: u32_at(entry, 8)?,
                element_bytes: u32_at(entry, 12)?,
                bytes: bytes.get(offset..offset.checked_add(len)?)?.to_vec(),
            });
        }
        Some(Self {
            header,
            sections,
            table_first: table_offset < first_offset,
        })
    }

    /// The position of the first entry with `id`.
    pub fn position(&self, id: u32) -> Option<usize> {
        self.sections.iter().position(|section| section.id == id)
    }

    /// Lays the package out: the header, then the sections in table order
    /// (each 64-aligned) with the table before or after them, every section
    /// hashed and the root checksum set.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = self.header.clone();
        out.resize(HEADER_FIXED_BYTES, 0);
        let pad = |out: &mut Vec<u8>| out.resize(out.len().next_multiple_of(SECTION_ALIGN), 0);
        let table_len = self.sections.len() * TABLE_ENTRY_BYTES_V3;
        if self.table_first {
            out.resize(HEADER_FIXED_BYTES + table_len, 0);
        }
        let mut offsets = Vec::with_capacity(self.sections.len());
        for section in &self.sections {
            pad(&mut out);
            offsets.push(out.len());
            out.extend_from_slice(&section.bytes);
        }
        pad(&mut out);
        let table_offset = if self.table_first {
            HEADER_FIXED_BYTES
        } else {
            out.len()
        };
        let mut table = Vec::with_capacity(table_len);
        for (section, offset) in self.sections.iter().zip(offsets) {
            table.extend_from_slice(&section.id.to_le_bytes());
            table.extend_from_slice(&section.param.to_le_bytes());
            table.extend_from_slice(&section.flags.to_le_bytes());
            table.extend_from_slice(&section.element_bytes.to_le_bytes());
            table.extend_from_slice(&(offset as u64).to_le_bytes());
            table.extend_from_slice(&(section.bytes.len() as u64).to_le_bytes());
            table.extend_from_slice(blake3::hash(&section.bytes).as_bytes());
        }
        if self.table_first {
            out[table_offset..table_offset + table_len].copy_from_slice(&table);
        } else {
            out.extend_from_slice(&table);
            pad(&mut out);
        }
        out[HEADER_SECTION_COUNT..HEADER_SECTION_COUNT + 4]
            .copy_from_slice(&(self.sections.len() as u32).to_le_bytes());
        out[HEADER_V3_TABLE_OFFSET..HEADER_V3_TABLE_OFFSET + 8]
            .copy_from_slice(&(table_offset as u64).to_le_bytes());
        resign(&mut out);
        out
    }
}

/// Sets the root checksum of a format-2 or format-3 package to match its
/// header and table, as a hostile package would carry (the checksum guards
/// against corruption only). Leaves `bytes` alone when its table is out of
/// bounds.
pub fn resign(bytes: &mut [u8]) {
    if bytes.len() < HEADER_FIXED_BYTES {
        return;
    }
    let u32_at = |at: usize| u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
    let (table, checksum_at) = match u32_at(HEADER_VERSION) {
        3 => {
            let offset = u64::from_le_bytes(
                bytes[HEADER_V3_TABLE_OFFSET..HEADER_V3_TABLE_OFFSET + 8]
                    .try_into()
                    .unwrap(),
            );
            let len = (u32_at(HEADER_SECTION_COUNT) as u64) * TABLE_ENTRY_BYTES_V3 as u64;
            (
                usize::try_from(offset)
                    .ok()
                    .zip(usize::try_from(len).ok())
                    .and_then(|(offset, len)| Some(offset..offset.checked_add(len)?)),
                HEADER_V3_ROOT_CHECKSUM,
            )
        }
        _ => {
            let len = (u32_at(HEADER_SECTION_COUNT) as usize).saturating_mul(TABLE_ENTRY_BYTES);
            (
                HEADER_FIXED_BYTES
                    .checked_add(len)
                    .map(|end| HEADER_FIXED_BYTES..end),
                HEADER_ROOT_CHECKSUM,
            )
        }
    };
    let Some(table) = table.filter(|table| table.end <= bytes.len()) else {
        return;
    };
    let mut header = bytes[..HEADER_FIXED_BYTES].to_vec();
    header[checksum_at..checksum_at + 32].fill(0);
    let mut root = blake3::Hasher::new();
    root.update(&header);
    root.update(&bytes[table]);
    bytes[checksum_at..checksum_at + 32].copy_from_slice(root.finalize().as_bytes());
}
