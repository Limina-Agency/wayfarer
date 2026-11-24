//! A compact binary encoding for serde types: the wire format of bincode 1.x
//! with `bincode::DefaultOptions` (variable-length integers, little endian).
//!
//! Packages store each attribute class's route-leg part ([`crate::request::LegAttrs`])
//! in this format, so it is frozen: a change here is a package format change.
//! It is our own implementation of that format (bincode is unmaintained),
//! checked byte for byte against bincode 1.3.3 by the tests below. The road
//! graph snapshot and the compiler's per-way facts store use it too.
//!
//! The format, as bincode 1 writes it:
//!
//! - `u8`, `i8` and `bool` are one byte; `f32` and `f64` are little-endian.
//! - `u16`, `u32` and `u64` (and lengths and enum variant indexes) are a
//!   varint: a value below 251 is one byte; otherwise a marker byte 251, 252
//!   or 253 followed by the value as a little-endian `u16`, `u32` or `u64`.
//! - `i16`, `i32` and `i64` are zigzag-mapped, then written as a varint.
//! - Strings and byte arrays are a varint length and the bytes; sequences
//!   and maps a varint length and their elements; `char` its UTF-8 bytes.
//! - `Option` is a byte 0 (`None`) or 1 followed by the value.
//! - Structs and tuples are their fields in order, with no framing.
//! - Enum variants are their index (a varint), then their fields.
//!
//! 128-bit integers and self-describing deserialization (`deserialize_any`)
//! are not supported.

use serde::de::{self, DeserializeSeed, IntoDeserializer, Visitor};
use serde::ser::{self, Serialize};
use std::fmt;
use std::io::Write;

/// An encoding or decoding failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error(String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl ser::Error for Error {
    fn custom<T: fmt::Display>(message: T) -> Self {
        Self(message.to_string())
    }
}

impl de::Error for Error {
    fn custom<T: fmt::Display>(message: T) -> Self {
        Self(message.to_string())
    }
}

fn error(message: &str) -> Error {
    Error(message.to_string())
}

/// Encodes `value`.
pub fn to_vec<T: Serialize + ?Sized>(value: &T) -> Result<Vec<u8>, Error> {
    let mut bytes = Vec::new();
    value.serialize(&mut Serializer { out: &mut bytes })?;
    Ok(bytes)
}

/// Encodes `value` into `writer`.
pub fn to_writer<W: Write, T: Serialize + ?Sized>(writer: W, value: &T) -> Result<(), Error> {
    value.serialize(&mut Serializer { out: writer })
}

/// Decodes a `T` that fills `bytes` exactly; trailing bytes are an error.
pub fn from_slice<'de, T: de::Deserialize<'de>>(bytes: &'de [u8]) -> Result<T, Error> {
    let (value, rest) = take_from_slice(bytes)?;
    if !rest.is_empty() {
        return Err(error("trailing bytes after the encoded value"));
    }
    Ok(value)
}

/// Decodes a `T` from the start of `bytes`, returning it and the bytes after
/// it.
pub fn take_from_slice<'de, T: de::Deserialize<'de>>(
    bytes: &'de [u8],
) -> Result<(T, &'de [u8]), Error> {
    let mut deserializer = Deserializer {
        input: bytes,
        field_limit: None,
    };
    let value = T::deserialize(&mut deserializer)?;
    Ok((value, deserializer.input))
}

/// Decodes the struct `T` from the start of `bytes`, reading only its first
/// `fields` fields, and returns it and the bytes after them. Fields past
/// `fields` (or past the struct's own count, if that is smaller) are not read
/// and take their serde defaults, so `T` needs `#[serde(default)]`. Nested
/// structs are read whole.
///
/// This is how a record that only ever gains fields at its end is read
/// across revisions: an older record has fewer fields, a newer one has bytes
/// after the reader's last field.
pub fn take_struct_prefix<'de, T: de::Deserialize<'de>>(
    bytes: &'de [u8],
    fields: usize,
) -> Result<(T, &'de [u8]), Error> {
    let mut deserializer = Deserializer {
        input: bytes,
        field_limit: Some(fields),
    };
    let value = T::deserialize(&mut deserializer)?;
    Ok((value, deserializer.input))
}

const U16_MARKER: u8 = 251;
const U32_MARKER: u8 = 252;
const U64_MARKER: u8 = 253;

fn zigzag(value: i64) -> u64 {
    ((value << 1) ^ (value >> 63)) as u64
}

fn unzigzag(value: u64) -> i64 {
    ((value >> 1) as i64) ^ -((value & 1) as i64)
}

struct Serializer<W> {
    out: W,
}

impl<W: Write> Serializer<W> {
    fn bytes(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.out
            .write_all(bytes)
            .map_err(|io| Error(io.to_string()))
    }

    fn varint(&mut self, value: u64) -> Result<(), Error> {
        if value < u64::from(U16_MARKER) {
            self.bytes(&[value as u8])
        } else if let Ok(value) = u16::try_from(value) {
            self.bytes(&[U16_MARKER])?;
            self.bytes(&value.to_le_bytes())
        } else if let Ok(value) = u32::try_from(value) {
            self.bytes(&[U32_MARKER])?;
            self.bytes(&value.to_le_bytes())
        } else {
            self.bytes(&[U64_MARKER])?;
            self.bytes(&value.to_le_bytes())
        }
    }

    fn length(&mut self, len: Option<usize>) -> Result<(), Error> {
        let len = len.ok_or_else(|| error("sequences and maps need a known length"))?;
        self.varint(len as u64)
    }
}

impl<W: Write> ser::Serializer for &mut Serializer<W> {
    type Ok = ();
    type Error = Error;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn is_human_readable(&self) -> bool {
        false
    }

    fn serialize_bool(self, value: bool) -> Result<(), Error> {
        self.bytes(&[u8::from(value)])
    }
    fn serialize_i8(self, value: i8) -> Result<(), Error> {
        self.bytes(&value.to_le_bytes())
    }
    fn serialize_i16(self, value: i16) -> Result<(), Error> {
        self.varint(zigzag(i64::from(value)))
    }
    fn serialize_i32(self, value: i32) -> Result<(), Error> {
        self.varint(zigzag(i64::from(value)))
    }
    fn serialize_i64(self, value: i64) -> Result<(), Error> {
        self.varint(zigzag(value))
    }
    fn serialize_i128(self, _: i128) -> Result<(), Error> {
        Err(error("128-bit integers are not supported"))
    }
    fn serialize_u8(self, value: u8) -> Result<(), Error> {
        self.bytes(&[value])
    }
    fn serialize_u16(self, value: u16) -> Result<(), Error> {
        self.varint(u64::from(value))
    }
    fn serialize_u32(self, value: u32) -> Result<(), Error> {
        self.varint(u64::from(value))
    }
    fn serialize_u64(self, value: u64) -> Result<(), Error> {
        self.varint(value)
    }
    fn serialize_u128(self, _: u128) -> Result<(), Error> {
        Err(error("128-bit integers are not supported"))
    }
    fn serialize_f32(self, value: f32) -> Result<(), Error> {
        self.bytes(&value.to_le_bytes())
    }
    fn serialize_f64(self, value: f64) -> Result<(), Error> {
        self.bytes(&value.to_le_bytes())
    }
    fn serialize_char(self, value: char) -> Result<(), Error> {
        self.bytes(value.encode_utf8(&mut [0; 4]).as_bytes())
    }
    fn serialize_str(self, value: &str) -> Result<(), Error> {
        self.serialize_bytes(value.as_bytes())
    }
    fn serialize_bytes(self, value: &[u8]) -> Result<(), Error> {
        self.varint(value.len() as u64)?;
        self.bytes(value)
    }
    fn serialize_none(self) -> Result<(), Error> {
        self.bytes(&[0])
    }
    fn serialize_some<T: Serialize + ?Sized>(self, value: &T) -> Result<(), Error> {
        self.bytes(&[1])?;
        value.serialize(self)
    }
    fn serialize_unit(self) -> Result<(), Error> {
        Ok(())
    }
    fn serialize_unit_struct(self, _: &'static str) -> Result<(), Error> {
        Ok(())
    }
    fn serialize_unit_variant(
        self,
        _: &'static str,
        index: u32,
        _: &'static str,
    ) -> Result<(), Error> {
        self.varint(u64::from(index))
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Error> {
        value.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _: &'static str,
        index: u32,
        _: &'static str,
        value: &T,
    ) -> Result<(), Error> {
        self.varint(u64::from(index))?;
        value.serialize(self)
    }
    fn serialize_seq(self, len: Option<usize>) -> Result<Self, Error> {
        self.length(len)?;
        Ok(self)
    }
    fn serialize_tuple(self, _: usize) -> Result<Self, Error> {
        Ok(self)
    }
    fn serialize_tuple_struct(self, _: &'static str, _: usize) -> Result<Self, Error> {
        Ok(self)
    }
    fn serialize_tuple_variant(
        self,
        _: &'static str,
        index: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Self, Error> {
        self.varint(u64::from(index))?;
        Ok(self)
    }
    fn serialize_map(self, len: Option<usize>) -> Result<Self, Error> {
        self.length(len)?;
        Ok(self)
    }
    fn serialize_struct(self, _: &'static str, _: usize) -> Result<Self, Error> {
        Ok(self)
    }
    fn serialize_struct_variant(
        self,
        _: &'static str,
        index: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Self, Error> {
        self.varint(u64::from(index))?;
        Ok(self)
    }
}

macro_rules! compound {
    ($trait:ident, $method:ident) => {
        impl<W: Write> ser::$trait for &mut Serializer<W> {
            type Ok = ();
            type Error = Error;
            fn $method<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Error> {
                value.serialize(&mut **self)
            }
            fn end(self) -> Result<(), Error> {
                Ok(())
            }
        }
    };
}
compound!(SerializeSeq, serialize_element);
compound!(SerializeTuple, serialize_element);
compound!(SerializeTupleStruct, serialize_field);
compound!(SerializeTupleVariant, serialize_field);

impl<W: Write> ser::SerializeMap for &mut Serializer<W> {
    type Ok = ();
    type Error = Error;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, key: &T) -> Result<(), Error> {
        key.serialize(&mut **self)
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, value: &T) -> Result<(), Error> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), Error> {
        Ok(())
    }
}

impl<W: Write> ser::SerializeStruct for &mut Serializer<W> {
    type Ok = ();
    type Error = Error;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Error> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), Error> {
        Ok(())
    }
}

impl<W: Write> ser::SerializeStructVariant for &mut Serializer<W> {
    type Ok = ();
    type Error = Error;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Error> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), Error> {
        Ok(())
    }
}

struct Deserializer<'de> {
    input: &'de [u8],
    /// How many fields the next struct read takes ([`take_struct_prefix`]);
    /// it applies once, to the outermost struct.
    field_limit: Option<usize>,
}

impl<'de> Deserializer<'de> {
    fn take(&mut self, len: usize) -> Result<&'de [u8], Error> {
        if self.input.len() < len {
            return Err(error("unexpected end of input"));
        }
        let (head, rest) = self.input.split_at(len);
        self.input = rest;
        Ok(head)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        let mut array = [0; N];
        array.copy_from_slice(self.take(N)?);
        Ok(array)
    }

    fn byte(&mut self) -> Result<u8, Error> {
        Ok(self.array::<1>()?[0])
    }

    fn varint(&mut self) -> Result<u64, Error> {
        match self.byte()? {
            U16_MARKER => Ok(u64::from(u16::from_le_bytes(self.array()?))),
            U32_MARKER => Ok(u64::from(u32::from_le_bytes(self.array()?))),
            U64_MARKER => Ok(u64::from_le_bytes(self.array()?)),
            byte if byte < U16_MARKER => Ok(u64::from(byte)),
            _ => Err(error("invalid varint marker")),
        }
    }

    fn unsigned<T: TryFrom<u64>>(&mut self) -> Result<T, Error> {
        T::try_from(self.varint()?).map_err(|_| error("integer out of range"))
    }

    fn signed<T: TryFrom<i64>>(&mut self) -> Result<T, Error> {
        T::try_from(unzigzag(self.varint()?)).map_err(|_| error("integer out of range"))
    }

    fn length(&mut self) -> Result<usize, Error> {
        self.unsigned()
    }

    fn str(&mut self) -> Result<&'de str, Error> {
        let len = self.length()?;
        std::str::from_utf8(self.take(len)?).map_err(|_| error("invalid UTF-8 in a string"))
    }
}

impl<'de> de::Deserializer<'de> for &mut Deserializer<'de> {
    type Error = Error;

    fn is_human_readable(&self) -> bool {
        false
    }

    fn deserialize_any<V: Visitor<'de>>(self, _: V) -> Result<V::Value, Error> {
        Err(error("the format is not self-describing"))
    }
    fn deserialize_ignored_any<V: Visitor<'de>>(self, _: V) -> Result<V::Value, Error> {
        Err(error("the format is not self-describing"))
    }
    fn deserialize_bool<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        match self.byte()? {
            0 => visitor.visit_bool(false),
            1 => visitor.visit_bool(true),
            _ => Err(error("invalid bool")),
        }
    }
    fn deserialize_i8<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_i8(i8::from_le_bytes(self.array()?))
    }
    fn deserialize_i16<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_i16(self.signed()?)
    }
    fn deserialize_i32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_i32(self.signed()?)
    }
    fn deserialize_i64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_i64(self.signed()?)
    }
    fn deserialize_u8<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_u8(self.byte()?)
    }
    fn deserialize_u16<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_u16(self.unsigned()?)
    }
    fn deserialize_u32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_u32(self.unsigned()?)
    }
    fn deserialize_u64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_u64(self.varint()?)
    }
    fn deserialize_f32<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_f32(f32::from_le_bytes(self.array()?))
    }
    fn deserialize_f64<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_f64(f64::from_le_bytes(self.array()?))
    }
    fn deserialize_char<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        let width = match self.input.first() {
            Some(byte) if *byte < 0x80 => 1,
            Some(byte) if *byte >> 5 == 0b110 => 2,
            Some(byte) if *byte >> 4 == 0b1110 => 3,
            Some(byte) if *byte >> 3 == 0b11110 => 4,
            Some(_) => return Err(error("invalid char")),
            None => return Err(error("unexpected end of input")),
        };
        let text = std::str::from_utf8(self.take(width)?).map_err(|_| error("invalid char"))?;
        visitor.visit_char(text.chars().next().ok_or_else(|| error("invalid char"))?)
    }
    fn deserialize_str<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_borrowed_str(self.str()?)
    }
    fn deserialize_string<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_borrowed_str(self.str()?)
    }
    fn deserialize_bytes<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        let len = self.length()?;
        visitor.visit_borrowed_bytes(self.take(len)?)
    }
    fn deserialize_byte_buf<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        self.deserialize_bytes(visitor)
    }
    fn deserialize_option<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        match self.byte()? {
            0 => visitor.visit_none(),
            1 => visitor.visit_some(self),
            _ => Err(error("invalid Option tag")),
        }
    }
    fn deserialize_unit<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_unit()
    }
    fn deserialize_unit_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        visitor: V,
    ) -> Result<V::Value, Error> {
        visitor.visit_unit()
    }
    fn deserialize_newtype_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        visitor: V,
    ) -> Result<V::Value, Error> {
        visitor.visit_newtype_struct(self)
    }
    fn deserialize_seq<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        let len = self.length()?;
        visitor.visit_seq(Elements { de: self, len })
    }
    fn deserialize_tuple<V: Visitor<'de>>(self, len: usize, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_seq(Elements { de: self, len })
    }
    fn deserialize_tuple_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        len: usize,
        visitor: V,
    ) -> Result<V::Value, Error> {
        visitor.visit_seq(Elements { de: self, len })
    }
    fn deserialize_map<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        let len = self.length()?;
        visitor.visit_map(Elements { de: self, len })
    }
    fn deserialize_struct<V: Visitor<'de>>(
        self,
        _: &'static str,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Error> {
        let len = match self.field_limit.take() {
            Some(limit) => limit.min(fields.len()),
            None => fields.len(),
        };
        visitor.visit_seq(Elements { de: self, len })
    }
    fn deserialize_enum<V: Visitor<'de>>(
        self,
        _: &'static str,
        _: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Error> {
        visitor.visit_enum(self)
    }
    fn deserialize_identifier<V: Visitor<'de>>(self, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_u32(self.unsigned()?)
    }
}

/// The elements of a sequence, tuple, struct or map: `len` of them.
struct Elements<'a, 'de> {
    de: &'a mut Deserializer<'de>,
    len: usize,
}

impl<'de> de::SeqAccess<'de> for Elements<'_, 'de> {
    type Error = Error;
    fn next_element_seed<T: DeserializeSeed<'de>>(
        &mut self,
        seed: T,
    ) -> Result<Option<T::Value>, Error> {
        if self.len == 0 {
            return Ok(None);
        }
        self.len -= 1;
        seed.deserialize(&mut *self.de).map(Some)
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.len)
    }
}

impl<'de> de::MapAccess<'de> for Elements<'_, 'de> {
    type Error = Error;
    fn next_key_seed<K: DeserializeSeed<'de>>(
        &mut self,
        seed: K,
    ) -> Result<Option<K::Value>, Error> {
        if self.len == 0 {
            return Ok(None);
        }
        self.len -= 1;
        seed.deserialize(&mut *self.de).map(Some)
    }
    fn next_value_seed<V: DeserializeSeed<'de>>(&mut self, seed: V) -> Result<V::Value, Error> {
        seed.deserialize(&mut *self.de)
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.len)
    }
}

impl<'de> de::EnumAccess<'de> for &mut Deserializer<'de> {
    type Error = Error;
    type Variant = Self;
    fn variant_seed<V: DeserializeSeed<'de>>(self, seed: V) -> Result<(V::Value, Self), Error> {
        let index: u32 = self.unsigned()?;
        let value = seed.deserialize(index.into_deserializer())?;
        Ok((value, self))
    }
}

impl<'de> de::VariantAccess<'de> for &mut Deserializer<'de> {
    type Error = Error;
    fn unit_variant(self) -> Result<(), Error> {
        Ok(())
    }
    fn newtype_variant_seed<T: DeserializeSeed<'de>>(self, seed: T) -> Result<T::Value, Error> {
        seed.deserialize(self)
    }
    fn tuple_variant<V: Visitor<'de>>(self, len: usize, visitor: V) -> Result<V::Value, Error> {
        visitor.visit_seq(Elements { de: self, len })
    }
    fn struct_variant<V: Visitor<'de>>(
        self,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value, Error> {
        visitor.visit_seq(Elements {
            de: self,
            len: fields.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    //! Byte-for-byte against bincode 1.3.3 (`DefaultOptions`, which packages
    //! were first written with), on edge-case values of every shape we store.
    use super::*;
    use crate::request::LegAttrs;
    use crate::{DirectionalAccess, RoadHierarchyTier, RoadType, SharedEdgeInfo};
    use bincode::Options;
    use serde::{Deserialize, Serialize};
    use std::collections::BTreeMap;
    use std::sync::Arc;

    fn bincode_bytes<T: Serialize + ?Sized>(value: &T) -> Vec<u8> {
        bincode::DefaultOptions::new()
            .reject_trailing_bytes()
            .serialize(value)
            .unwrap()
    }

    /// Encodes like bincode, and decodes bincode's bytes back to a value that
    /// encodes the same.
    fn check<T>(value: &T)
    where
        T: Serialize + for<'de> Deserialize<'de> + fmt::Debug,
    {
        let ours = to_vec(value).unwrap();
        assert_eq!(ours, bincode_bytes(value), "{value:?}");
        let back: T = from_slice(&ours).unwrap();
        assert_eq!(to_vec(&back).unwrap(), ours, "{value:?}");
    }

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
            xs[(self.next() % xs.len() as u64) as usize]
        }
        fn some<T>(&mut self, f: impl FnOnce(&mut Self) -> T) -> Option<T> {
            (!self.next().is_multiple_of(3)).then(|| f(self))
        }
        fn int(&mut self) -> u64 {
            const EDGES: [u64; 16] = [
                0,
                1,
                127,
                128,
                250,
                251,
                252,
                253,
                255,
                256,
                65535,
                65536,
                0xffff_fffe,
                0xffff_ffff,
                0x1_0000_0000,
                u64::MAX,
            ];
            if self.next().is_multiple_of(2) {
                self.pick(&EDGES)
            } else {
                self.next() >> (self.next() % 64)
            }
        }
        fn f32(&mut self) -> f32 {
            if self.next().is_multiple_of(2) {
                self.pick(&[0.0, -0.0, f32::NAN, f32::INFINITY, f32::MIN, 1e-40])
            } else {
                f32::from_bits(self.next() as u32)
            }
        }
        fn text(&mut self) -> String {
            let len = self.pick(&[0usize, 1, 5, 250, 251, 300]);
            (0..len)
                .map(|i| ['a', 'é', '€', '🚧'][(i + self.next() as usize) % 4])
                .collect()
        }
        fn opt_text(&mut self) -> Option<Arc<str>> {
            self.some(|r| Arc::from(r.text()))
        }
        fn ids(&mut self) -> Vec<u32> {
            let len = self.pick(&[0usize, 1, 3, 251]);
            (0..len).map(|_| self.int() as u32).collect()
        }
    }

    #[test]
    fn integers_at_every_varint_boundary() {
        let mut rng = Rng(1);
        for _ in 0..20_000 {
            let n = rng.int();
            check(&n);
            check(&(n as u32));
            check(&(n as u16));
            check(&(n as u8));
            check(&(n as i64));
            check(&(n as i32));
            check(&(n as i16));
            check(&(n as i8));
        }
        for n in [
            i64::MIN,
            i64::MIN + 1,
            -126,
            -125,
            -1,
            0,
            125,
            126,
            i64::MAX,
        ] {
            check(&n);
        }
    }

    #[test]
    fn floats_strings_options_sequences_and_maps() {
        let mut rng = Rng(2);
        for _ in 0..2_000 {
            check(&rng.f32().to_bits());
            let f = rng.f32();
            assert_eq!(to_vec(&f).unwrap(), bincode_bytes(&f));
            let d = f64::from_bits(rng.next());
            assert_eq!(to_vec(&d).unwrap(), bincode_bytes(&d));
            check(&rng.text());
            check(&rng.opt_text());
            check(&rng.ids());
            check(&(rng.next() as u8 as char, rng.pick(&['é', '€', '🚧'])));
            let map: BTreeMap<u32, String> = (0..rng.next() % 5)
                .map(|_| (rng.int() as u32, rng.text()))
                .collect();
            check(&map);
        }
    }

    #[derive(Debug, Serialize, Deserialize)]
    #[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
    enum Shape {
        Unit,
        Newtype(i16),
        Tuple(u8, String),
        Struct { a: Option<u64>, b: Vec<(f64, f64)> },
    }

    #[test]
    fn enums_of_every_kind() {
        for shape in [
            Shape::Unit,
            Shape::Newtype(-300),
            Shape::Tuple(7, "x".into()),
            Shape::Struct {
                a: Some(1 << 40),
                b: vec![(1.5, -2.25)],
            },
        ] {
            check(&shape);
        }
        check(&vec![RoadHierarchyTier::Highway; 3]);
    }

    fn random_leg_attrs(rng: &mut Rng) -> LegAttrs {
        let u32s = |rng: &mut Rng| rng.some(|r| r.int() as u32);
        let mut attrs = LegAttrs::from_info(SharedEdgeInfo::default(), &mut |_| 0);
        attrs.road_type = rng.pick(&[RoadType::Motorway, RoadType::Track, RoadType::Other]);
        attrs.directional_access = rng.pick(&[
            DirectionalAccess::Both,
            DirectionalAccess::Backward,
            DirectionalAccess::Closed,
        ]);
        attrs.highway = u32s(rng);
        attrs.name = u32s(rng);
        attrs.route_refs = rng.ids();
        attrs.maxspeed_kph = rng.some(Rng::f32);
        attrs.max_height_cm = rng.some(|r| r.int() as u16);
        attrs.max_weight_kg = u32s(rng);
        attrs.incline_percent = rng.some(|r| r.next() as i8);
        attrs.is_lit = rng.next().is_multiple_of(2);
        attrs.lane_count = rng.some(|r| r.next() as u8);
        attrs.conditional_restrictions = rng.ids();
        attrs.osm_forward_mode_mask = rng.int() as u32;
        attrs.hierarchy_tier =
            rng.some(|r| r.pick(&[RoadHierarchyTier::Local, RoadHierarchyTier::Highway]));
        attrs.layer = rng.some(|r| r.next() as i16);
        attrs.is_access_restricted = rng.next().is_multiple_of(2);
        attrs
    }

    #[test]
    fn leg_attrs_and_shared_edge_infos_match_bincode() {
        let mut rng = Rng(3);
        for _ in 0..5_000 {
            let attrs = random_leg_attrs(&mut rng);
            assert_eq!(to_vec(&attrs).unwrap(), bincode_bytes(&attrs));
            let back: LegAttrs = from_slice(&bincode_bytes(&attrs)).unwrap();
            assert_eq!(to_vec(&back).unwrap(), bincode_bytes(&attrs));

            let info = SharedEdgeInfo {
                shared_edge_info_id: rng.int(),
                osm_way_id: rng.int() as i64,
                geometry: (0..rng.next() % 4)
                    .map(|_| (f64::from_bits(rng.next()), -1.0))
                    .collect(),
                name: rng.opt_text(),
                ref_name: rng.opt_text(),
                route_refs: (0..rng.next() % 3).map(|_| rng.text()).collect(),
                maxspeed_forward_kph: rng.some(Rng::f32),
                physical_width_cm: rng.some(|r| r.int() as u16),
                ..SharedEdgeInfo::default()
            };
            assert_eq!(to_vec(&info).unwrap(), bincode_bytes(&info));
            let back: SharedEdgeInfo = from_slice(&bincode_bytes(&info)).unwrap();
            assert_eq!(to_vec(&back).unwrap(), bincode_bytes(&info));
        }
    }

    #[test]
    fn malformed_input_is_an_error_not_a_panic() {
        let bytes = to_vec(&("text".to_string(), Some(300u32), vec![1u16, 2])).unwrap();
        type Value = (String, Option<u32>, Vec<u16>);
        for cut in 0..bytes.len() {
            assert!(from_slice::<Value>(&bytes[..cut]).is_err(), "cut at {cut}");
        }
        let mut trailing = bytes.clone();
        trailing.push(0);
        assert!(from_slice::<Value>(&trailing).is_err());
        let (value, rest) = take_from_slice::<Value>(&trailing).unwrap();
        assert_eq!((value.1, rest), (Some(300), &[0u8][..]));
        assert!(from_slice::<bool>(&[2]).is_err());
        assert!(from_slice::<Option<u8>>(&[2, 0]).is_err());
        assert!(from_slice::<u8>(&[]).is_err());
        assert!(from_slice::<u16>(&[252, 0, 0, 1, 0]).is_err());
        assert!(from_slice::<u64>(&[254]).is_err());
        assert!(from_slice::<String>(&[2, 0xff, 0xfe]).is_err());
        // A huge length claim fails on the missing bytes, without allocating.
        assert!(
            from_slice::<Vec<u8>>(&[253, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f]).is_err()
        );
    }

    /// A struct that only gains fields at its end reads across revisions:
    /// the older record leaves the new fields at their defaults, and the
    /// newer one leaves its extra fields unread.
    #[test]
    fn a_struct_prefix_reads_older_and_newer_records() {
        #[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq, Default)]
        #[serde(default)]
        struct Inner {
            x: u32,
            y: Option<u16>,
        }
        #[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq, Default)]
        #[serde(default)]
        struct Old {
            a: u32,
            inner: Inner,
        }
        #[derive(serde::Serialize, serde::Deserialize, Debug, PartialEq, Default)]
        #[serde(default)]
        struct New {
            a: u32,
            inner: Inner,
            b: Option<String>,
            c: bool,
        }
        let inner = || Inner { x: 300, y: Some(7) };
        let old = to_vec(&Old {
            a: 5,
            inner: inner(),
        })
        .unwrap();
        let new = to_vec(&New {
            a: 5,
            inner: inner(),
            b: Some("x".into()),
            c: true,
        })
        .unwrap();
        // The newer reader on an older record: two fields, nested struct whole.
        let (read, rest) = take_struct_prefix::<New>(&old, 2).unwrap();
        assert_eq!(
            read,
            New {
                a: 5,
                inner: inner(),
                b: None,
                c: false
            }
        );
        assert!(rest.is_empty());
        // The older reader on a newer record: its two fields, then the rest.
        let (read, rest) = take_struct_prefix::<Old>(&new, 4).unwrap();
        assert_eq!(
            read,
            Old {
                a: 5,
                inner: inner()
            }
        );
        assert_eq!(rest, &new[old.len()..]);
        // Too few bytes for the fields asked for.
        assert!(take_struct_prefix::<New>(&old, 3).is_err());
        assert!(take_struct_prefix::<New>(&old[..old.len() - 1], 2).is_err());
    }
}
