//! The TIFF reader on every coding it accepts, against the values written.

mod common;

use common::{Place, Sample, Spec, tiff};
use std::io::Cursor;
use wayfarer_terrain_compiler::tiff::{decode_chunk, read_chunk, read_info};

fn spec(sample: Sample, predictor: u16, big_endian: bool, bigtiff: bool) -> Spec {
    Spec {
        width: 70,
        height: 45,
        tile: 32,
        sample,
        compression: 8,
        predictor,
        big_endian,
        bigtiff,
        place: Place::Utm {
            zone: 33,
            x: 500_000.0,
            y: 5_600_000.0,
            px: 10.0,
        },
        nodata: Some("-9999"),
    }
}

fn value(sample: Sample, r: u32, c: u32) -> f64 {
    match sample {
        Sample::U8 => ((r * 7 + c * 3) % 251) as f64,
        Sample::I16 => (r as f64 * 37.0 - c as f64 * 411.0) % 30_000.0,
        Sample::F32 => (r as f64 * 0.37 - c as f64 * 1.25 + 1234.5) as f32 as f64,
    }
}

#[test]
fn every_coding_reads_back() {
    for sample in [Sample::U8, Sample::I16, Sample::F32] {
        for predictor in [1, if sample == Sample::F32 { 3 } else { 2 }] {
            for be in [false, true] {
                for big in [false, true] {
                    let s = spec(sample, predictor, be, big);
                    let bytes = tiff(&s, &|r, c| value(sample, r, c));
                    let mut cur = Cursor::new(bytes);
                    let info = read_info(&mut cur, "t").unwrap();
                    assert_eq!((info.width, info.height, info.chunk_w), (70, 45, 32));
                    assert_eq!(info.nodata, Some(-9999.0));
                    assert_eq!(info.geo_key(3072), Some(32633));
                    let across = info.chunks_across();
                    for idx in 0..(across * info.chunks_down()) as usize {
                        let mut buf = Vec::new();
                        let comp = read_chunk(&mut cur, &info, "t", idx, &mut buf).unwrap();
                        let v = decode_chunk(&info, "t", idx, &comp, f64::NAN, |x| x).unwrap();
                        let (tr, tc) = (idx as u32 / across, idx as u32 % across);
                        for y in 0..32 {
                            for x in 0..32 {
                                let (r, c) = (tr * 32 + y, tc * 32 + x);
                                if r < 45 && c < 70 {
                                    assert_eq!(
                                        v[(y * 32 + x) as usize],
                                        value(sample, r, c),
                                        "{sample:?} predictor {predictor} be {be} big {big} at {r},{c}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn refuses_what_it_does_not_read() {
    let mut s = spec(Sample::F32, 1, false, false);
    s.compression = 5; // LZW
    let b = tiff(&s, &|_, _| 0.0);
    assert!(read_info(&mut Cursor::new(b), "t").is_err());
    let mut s = spec(Sample::I16, 3, false, false);
    s.compression = 1;
    let b = tiff(&s, &|_, _| 0.0);
    assert!(
        read_info(&mut Cursor::new(b), "t").is_err(),
        "predictor 3 on integers"
    );
    assert!(read_info(&mut Cursor::new(b"MM\0*\0\0\0\0".to_vec()), "t").is_err());
    assert!(read_info(&mut Cursor::new(b"GIF89a..".to_vec()), "t").is_err());
}
