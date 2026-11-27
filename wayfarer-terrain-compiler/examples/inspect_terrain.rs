//! Prints a terrain file's plane statistics as JSON: `inspect_terrain <file.wtr>`.

use wayfarer_terrain::terrain::{DEM_NONE, STEEP_NONE, Terrain};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: inspect_terrain <file>")?;
    let t = Terrain::read(&std::fs::read(path)?)?;
    let (mut steep, mut cover) = ([0u64; 16], [0u64; 16]);
    let (mut lo, mut hi, mut none, mut no_steep, mut conf) = (i32::MAX, i32::MIN, 0u64, 0u64, 0u64);
    for k in 0..t.blocks().len() {
        for (&d, (&s, (&c, &f))) in t
            .dm(k)
            .iter()
            .zip(t.steep(k).iter().zip(t.cover(k).iter().zip(t.conf(k))))
        {
            if d == DEM_NONE {
                none += 1;
            } else {
                lo = lo.min(d);
                hi = hi.max(d);
            }
            if s == STEEP_NONE {
                no_steep += 1
            } else {
                steep[s as usize] += 1
            }
            cover[c as usize] += 1;
            conf += f as u64;
        }
    }
    println!(
        "{{\"blocks\": {}, \"dm_minmax\": [{lo}, {hi}], \"no_height\": {none}, \"no_steep\": {no_steep}, \"steep_hist\": {steep:?}, \"cover_hist\": {cover:?}, \"conf\": {conf}, \"hash\": \"{}\", \"sources\": {:?}}}",
        t.blocks().len(),
        wayfarer_terrain::hex(&t.hash()),
        t.sources()
    );
    Ok(())
}
