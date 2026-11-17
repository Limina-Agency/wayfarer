//! Shared setup for the fuzz targets: a Monaco package compiled once per run.

use std::path::{Path, PathBuf};
use std::sync::OnceLock;

/// The Monaco fixture compiled into a temporary package, once per process.
/// Its coordinates: Monaco-Ville 43.7316, 7.4199; the port 43.7350, 7.4210.
pub fn monaco_package() -> &'static Path {
    static PACKAGE: OnceLock<(tempfile::TempDir, PathBuf)> = OnceLock::new();
    let (_dir, path) = PACKAGE.get_or_init(|| {
        let dir = tempfile::tempdir().expect("temporary directory");
        let output = dir.path().join("monaco.mrt");
        let input = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf");
        let request = wayfarer_compiler::CompileRequest::new(&input, &output);
        wayfarer_compiler::compile(&request, &wayfarer::CancelToken::new(), |_| {})
            .expect("the Monaco fixture compiles");
        (dir, output)
    });
    path
}
