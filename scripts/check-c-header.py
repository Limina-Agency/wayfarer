#!/usr/bin/env python3
"""Check the hand-written C header against the Rust C ABI it declares.

usage: scripts/check-c-header.py [header] [rust_source ...]

Defaults: wayfarer-ffi/include/wayfarer.h and every .rs file under
wayfarer-ffi/src. Every `extern "C" fn wayfarer_*` in the Rust sources must be
declared in the header with the same return and parameter types (Rust types
translated to C: `*const c_char` is `const char *`, `i32` is `int32_t`,
`*const u16` is `const wchar_t *` on Windows, ...), the header must declare no
function the library does not export, and every `pub const WAYFARER_*` must be
`#define`d to the same value. Parameter names are not compared.

Exit status: 0 the header matches, 1 it does not, 2 a type this script does
not know how to translate (teach it in `RUST_TO_C`).
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent

RUST_TO_C = {
    '()': 'void',
    'i8': 'int8_t', 'u8': 'uint8_t', 'i16': 'int16_t', 'u16': 'uint16_t',
    'i32': 'int32_t', 'u32': 'uint32_t', 'i64': 'int64_t', 'u64': 'uint64_t',
    'f32': 'float', 'f64': 'double', 'usize': 'size_t', 'isize': 'ptrdiff_t',
    'bool': 'bool', 'c_char': 'char', 'c_int': 'int', 'c_void': 'void',
}
# Pointees whose C spelling differs from the pointer-free translation.
POINTEE_TO_C = {'u16': 'wchar_t'}  # the `_w` functions' UTF-16 paths (Windows)


class UnknownType(Exception):
    pass


def rust_type_to_c(rust):
    rust = original = re.sub(r'\s+', ' ', rust.strip())
    stars = []
    while True:
        match = re.match(r'\*(const|mut) (.*)', rust)
        if not match:
            break
        stars.append(match.group(1))
        rust = match.group(2)
    base = rust.split('::')[-1]
    if stars and base in POINTEE_TO_C:
        base = POINTEE_TO_C[base]
    elif base in RUST_TO_C:
        base = RUST_TO_C[base]
    elif not re.fullmatch(r'Wayfarer\w+', base):
        raise UnknownType(original)
    # Outermost pointer first in `stars`; C spells the innermost qualifier
    # next to the base type.
    c = base
    for index, kind in enumerate(reversed(stars)):
        if kind == 'const' and index == 0:
            c = f'const {c}'
        c += ' *' if index == 0 else '*'
        if kind == 'const' and index > 0:
            c += ' const'
    return normalize_c(c)


def normalize_c(c):
    c = re.sub(r'\s+', ' ', c.strip())
    c = re.sub(r'\s*\*\s*', '*', c)
    return c


def split_params(text):
    text = text.strip()
    if text in ('', 'void'):
        return []
    return [part.strip() for part in text.split(',') if part.strip()]


def rust_functions(sources):
    pattern = re.compile(
        r'extern "C" fn (wayfarer_\w+)\s*\(([^)]*)\)\s*(?:->\s*([^{]+?))?\s*\{', re.S)
    functions = {}
    for path in sources:
        for name, params, ret in pattern.findall(path.read_text(encoding='utf-8')):
            types = []
            for param in split_params(params):
                types.append(rust_type_to_c(param.split(':', 1)[1]))
            functions[name] = (rust_type_to_c(ret or '()'), types)
    return functions


def c_functions(header):
    text = re.sub(r'/\*.*?\*/', '', header.read_text(encoding='utf-8'), flags=re.S)
    text = re.sub(r'//[^\n]*', '', text)
    pattern = re.compile(r'([A-Za-z_][\w \t\*]*?)\b(wayfarer_\w+)\s*\(([^)]*)\)\s*;', re.S)
    functions = {}
    for ret, name, params in pattern.findall(text):
        types = []
        for param in split_params(params):
            # Drop the parameter name: the last identifier not part of the type.
            param = re.sub(r'\s+', ' ', param)
            match = re.fullmatch(r'(.*?[\s\*])([A-Za-z_]\w*)', param)
            types.append(normalize_c(match.group(1) if match else param))
        functions[name] = (normalize_c(ret), types)
    return functions


def constants(header, sources):
    rust = {}
    for path in sources:
        for name, value in re.findall(
                r'pub const (WAYFARER_\w+)\s*:\s*\w+\s*=\s*(-?\d+)\s*;',
                path.read_text(encoding='utf-8')):
            rust[name] = int(value)
    c = {name: int(value) for name, value in re.findall(
        r'#define (WAYFARER_\w+)\s+(-?\d+)\b', header.read_text(encoding='utf-8'))}
    return rust, c


def main(argv):
    header = pathlib.Path(argv[1]) if len(argv) > 1 else ROOT / 'wayfarer-ffi/include/wayfarer.h'
    sources = [pathlib.Path(p) for p in argv[2:]] or sorted((ROOT / 'wayfarer-ffi/src').rglob('*.rs'))
    try:
        rust = rust_functions(sources)
    except UnknownType as error:
        print(f'error: no C translation for the Rust type `{error}`; add it to RUST_TO_C', file=sys.stderr)
        return 2
    c = c_functions(header)
    problems = []
    for name in sorted(rust.keys() - c.keys()):
        problems.append(f'{name}: exported by the library, not declared in the header')
    for name in sorted(c.keys() - rust.keys()):
        problems.append(f'{name}: declared in the header, not exported by the library')
    for name in sorted(rust.keys() & c.keys()):
        (rust_ret, rust_params), (c_ret, c_params) = rust[name], c[name]
        if rust_ret != c_ret:
            problems.append(f'{name}: returns `{rust_ret}` in Rust, `{c_ret}` in the header')
        if len(rust_params) != len(c_params):
            problems.append(f'{name}: {len(rust_params)} parameters in Rust, {len(c_params)} in the header')
            continue
        for index, (r, h) in enumerate(zip(rust_params, c_params)):
            if r != h:
                problems.append(f'{name}: parameter {index + 1} is `{r}` in Rust, `{h}` in the header')
    rust_constants, c_constants = constants(header, sources)
    for name in sorted(rust_constants.keys() | c_constants.keys()):
        r, h = rust_constants.get(name), c_constants.get(name)
        if r != h:
            problems.append(f'{name}: {r} in Rust, {h} in the header')
    for problem in problems:
        print(f'error: {problem}')
    if problems:
        return 1
    print(f'{header.name}: {len(c)} functions and {len(c_constants)} constants match the Rust C ABI')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
