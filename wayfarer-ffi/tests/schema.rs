//! The JSON Schemas of the C ABI's requests and results, in `schema/` at the
//! repository root, generated from the engine's types. After an intended
//! change to a type, rerun with `UPDATE_SCHEMA=1`.

use std::path::Path;

use schemars::generate::SchemaSettings;
use schemars::{JsonSchema, Schema};
use wayfarer::{
    MapMatchRequest, MapMatchResult, NavigationPositionSample, ProjectRequest,
    RouteIsochroneRequest, RouteIsochroneResult, RouteMatrixRequest, RouteMatrixResult,
    RouteNavigationInput, RouteNavigationProgress, RouteNavigationProgressOptions,
    RouteOverlayGeometryProjection, SnapRequest, SnapResult,
};
use wayfarer_compiler::{CompileProgress, CompileRequest, CompileResult};
use wayfarer_service::overlay_set::RouteOverlaySet;
use wayfarer_service::{RouteWorkflowRequest, RouteWorkflowResult};

fn request<T: JsonSchema>() -> Schema {
    SchemaSettings::draft2020_12()
        .for_deserialize()
        .into_generator()
        .into_root_schema_for::<T>()
}

fn result<T: JsonSchema>() -> Schema {
    SchemaSettings::draft2020_12()
        .for_serialize()
        .into_generator()
        .into_root_schema_for::<T>()
}

#[test]
fn schemas_match_the_types() {
    let schemas = [
        ("route-request", request::<RouteWorkflowRequest>()),
        ("route-result", result::<RouteWorkflowResult>()),
        ("matrix-request", request::<RouteMatrixRequest>()),
        ("matrix-result", result::<RouteMatrixResult>()),
        ("isochrone-request", request::<RouteIsochroneRequest>()),
        ("isochrone-result", result::<RouteIsochroneResult>()),
        ("map-match-request", request::<MapMatchRequest>()),
        ("map-match-result", result::<MapMatchResult>()),
        ("snap-request", request::<SnapRequest>()),
        ("snap-result", result::<SnapResult>()),
        ("project-request", request::<ProjectRequest>()),
        ("project-result", result::<RouteOverlayGeometryProjection>()),
        ("navigation-input", request::<RouteNavigationInput>()),
        ("navigation-sample", request::<NavigationPositionSample>()),
        (
            "navigation-progress-options",
            request::<RouteNavigationProgressOptions>(),
        ),
        ("navigation-progress", result::<RouteNavigationProgress>()),
        ("overlay-set", request::<RouteOverlaySet>()),
        ("compile-request", request::<CompileRequest>()),
        ("compile-progress", result::<CompileProgress>()),
        ("compile-result", result::<CompileResult>()),
    ];
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../schema");
    let update = std::env::var_os("UPDATE_SCHEMA").is_some();
    let mut stale = Vec::new();
    for (name, schema) in schemas {
        let path = dir.join(format!("{name}.json"));
        let text = serde_json::to_string_pretty(&schema).unwrap() + "\n";
        if update {
            std::fs::write(&path, &text).unwrap();
        } else if std::fs::read_to_string(&path).ok().as_deref() != Some(text.as_str()) {
            stale.push(name);
        }
    }
    assert!(
        stale.is_empty(),
        "schema/ is out of date for {stale:?}; rerun with UPDATE_SCHEMA=1 if the change is intended"
    );
}
