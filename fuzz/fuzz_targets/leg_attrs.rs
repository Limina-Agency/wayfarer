//! Attribute-class leg records across revisions (format-3 design section
//! 5.3): bytes as a record of any revision, then the record re-encoded,
//! lengthened and shortened.
//!
//! The first byte picks the revision the record claims (0 is format 2); the
//! rest is the record. Decoding must not panic. A record that decodes must
//! re-encode to bytes that decode to the same record at this build's
//! revision; with bytes appended it must still decode at a newer revision
//! (and not at this one); and every proper prefix of it must be refused.

#![no_main]

use libfuzzer_sys::fuzz_target;
use wayfarer_package::codec;
use wayfarer_package::format::LEG_ATTRS_REVISION;
use wayfarer_package::request::decode_leg_attrs;

fuzz_target!(|data: &[u8]| {
    let Some((&revision, record)) = data.split_first() else {
        return;
    };
    let revision = u32::from(revision % 4);
    let Some(attrs) = decode_leg_attrs(record, revision) else {
        return;
    };
    let bytes = codec::to_vec(&attrs).unwrap();
    let again = decode_leg_attrs(&bytes, LEG_ATTRS_REVISION).expect("a re-encoded record decodes");
    assert_eq!(codec::to_vec(&again).unwrap(), bytes, "decoding is not stable");
    let mut longer = bytes.clone();
    longer.extend_from_slice(record);
    longer.push(0);
    let newer = decode_leg_attrs(&longer, LEG_ATTRS_REVISION + 1).expect("a newer record decodes");
    assert_eq!(codec::to_vec(&newer).unwrap(), bytes, "appended bytes changed the fields");
    assert!(decode_leg_attrs(&longer, LEG_ATTRS_REVISION).is_none());
    for cut in 0..bytes.len() {
        for revision in [0, LEG_ATTRS_REVISION, LEG_ATTRS_REVISION + 1] {
            assert!(
                decode_leg_attrs(&bytes[..cut], revision).is_none(),
                "a record cut to {cut} bytes decoded at revision {revision}"
            );
        }
    }
});
