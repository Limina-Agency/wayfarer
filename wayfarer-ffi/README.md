# wayfarer-ffi

A C ABI for the offline routing engine, so any language with a C FFI can use it.
Open a routing package to get an opaque handle, then send route, matrix,
isochrone, waypoint workflow and map matching requests as JSON and get JSON
back; navigation progress runs on a prepared-route handle. The request and
result shapes are the engine's serde types; errors come back as
`{"code": ..., "message": ...}` with a status code, and no panic crosses the
boundary. Handles may be used from several threads at once, and a cancel token
stops a request from any thread.

The crate builds as a static library, a dynamic library and a Rust `rlib`. The
interface is declared in [`include/wayfarer.h`](include/wayfarer.h)
(with thread safety, ownership and JSON casing);
[`examples/route.c`](examples/route.c) is a complete C client and
[`examples/cancel.c`](examples/cancel.c) cancels routes from another thread.

## Using it from C

From a checkout of the repository:

```sh
cargo build --release -p wayfarer-ffi
cc wayfarer-ffi/examples/route.c -Iwayfarer-ffi/include \
   target/release/libwayfarer_ffi.a \
   -framework CoreFoundation -o route          # macOS; on Linux: -lpthread -ldl -lm
./route routing.mrt 43.7397 7.4275 43.7290 7.4160
```

The core of the client:

```c
WayfarerPackage *package = NULL;
char *text = NULL;
if (wayfarer_open("routing.mrt", NULL, &package, &text) != WAYFARER_OK) {
    fprintf(stderr, "open failed: %s\n", text);   /* JSON error object */
    wayfarer_string_free(text);
    return 1;
}
wayfarer_string_free(text);               /* the package info */

const char *request =
    "{\"origin\": {\"lat\": 43.7397, \"lng\": 7.4275},"
    " \"destination\": {\"lat\": 43.7290, \"lng\": 7.4160},"
    " \"profile\": \"wheeled\"}";
int32_t status = wayfarer_route(package, request, &text);
printf("%s\n", text);                              /* route, or error, as JSON */
wayfarer_string_free(text);
wayfarer_close(package);
```

On Windows, with MSVC (from a Developer Command Prompt), link the DLL's
import library, or the static library with `/MD`:

```bat
cargo build --release -p wayfarer-ffi
rem Against the DLL (wayfarer_ffi.dll next to route.exe at run time):
cl wayfarer-ffi\examples\route.c /Iwayfarer-ffi\include target\release\wayfarer_ffi.dll.lib
rem Static:
cl /MD wayfarer-ffi\examples\route.c /Iwayfarer-ffi\include target\release\wayfarer_ffi.lib ^
   ws2_32.lib userenv.lib ntdll.lib bcrypt.lib advapi32.lib
route.exe routing.mrt 43.7397 7.4275 43.7290 7.4160
```

The static library needs `/MD` (or `/MDd`), not `cl`'s default `/MT`: Rust's
MSVC targets build against the DLL C runtime, and so does the bundled zstd C
code, so with `/MT` the link fails with unresolved `__imp_clock`,
`__imp__wassert` and `__imp_qsort_s`. A host that has to use `/MT` builds the
library with `RUSTFLAGS="-C target-feature=+crt-static"`. The system
libraries a static link needs are printed by
`cargo rustc --release -p wayfarer-ffi --crate-type staticlib -- --print native-static-libs`.

**Paths are UTF-8, also on Windows.** A Windows C program gets `argv` and
the Win32 "A" functions' strings in the ANSI code page, which can't spell
every path, and `wayfarer_open` then fails with "path is not UTF-8". Take
the arguments from `wmain` and convert them with
`WideCharToMultiByte(CP_UTF8, ...)`, as the examples do (with MinGW, link
with `-municode`), or pass a `wchar_t` path to `wayfarer_open_w` or
`wayfarer_verify_w` (Windows only).

**Replacing a package that is open.** A compile, or any other package write
through Wayfarer, writes `<path>.tmp` and moves it over the old file: an
open handle keeps reading the old package and the next `wayfarer_open` gets
the new one. On Windows a memory-mapped file can't be deleted, so the open
package is renamed aside to `<path>.replaced-<unique>` and deleted once no
handle has it open (at the latest by the next replacement). A host that
downloads packages itself should do the same, or write each version to a new
path and open that.

Every string the library returns is freed with `wayfarer_string_free`,
and a handle is closed only after every call on it has returned.
`wayfarer_versions` returns the library, ABI, readable package formats
and costing model as JSON.

## Status

Version 0.x for the crate; the C interface is versioned on its own.

- **ABI version.** `WAYFARER_ABI_VERSION` (1 today) changes when
  `wayfarer.h` changes incompatibly. Check `wayfarer_abi_version()` against
  the header you built with.
- **Package format and costing model.** The library opens the formats reported
  by `wayfarer_versions` (`readableFormats`) and only packages of its
  own `costingModel`; other packages fail to open with a package error.
- The JSON request and result shapes are the engine's types and follow the
  engine's compatibility, not the ABI version.

## Data

Routes are computed on packages compiled from OpenStreetMap data, which is under
the Open Database License 1.0. When you display or distribute results or
packages, credit "© OpenStreetMap contributors"
(<https://www.openstreetmap.org/copyright>).

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).
