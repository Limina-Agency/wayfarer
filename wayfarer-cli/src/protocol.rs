//! The host protocol (version 1): an application runs
//! `wayfarer compile --protocol --request <file> [--parent-stdin]` as a child
//! process and reads one JSON event per line on stdout: `ingest` and
//! `graph_progress` reports, then `complete` or `error`. The wayfarer-cli
//! README documents the events.

use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use serde::Deserialize;
use serde_json::json;
#[cfg(test)]
use wayfarer_compiler::MobilityProfile;
use wayfarer_compiler::host_protocol::{self, Detail};
use wayfarer_compiler::{
    CancelToken, CompileError, CompileProgress, CompileRequest, CompileStage, MemoryProfile,
};

use crate::Failure;
use crate::compile::{CANCELLED_STATUS, read_request, stop_soon};

const PROTOCOL_VERSION: u32 = 1;

/// The protocol's own request, as hosts sent it before the protocol took a
/// `CompileRequest`. Deprecated: still read, with a warning on stderr, and to
/// be removed before 1.0. It has no field for options added since.
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct ProtocolRequest {
    input_pbf_path: PathBuf,
    /// The scratch parent: the compile works in a fresh directory inside it.
    temp_dir: PathBuf,
    graph_id: String,
    country_code: Option<String>,
    build_profile: BuildProfile,
    route_archive_path: PathBuf,
    /// Optional.
    #[serde(default)]
    threads: Option<usize>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum BuildProfile {
    DesktopFull,
    MobileOperational,
}

impl From<ProtocolRequest> for CompileRequest {
    fn from(request: ProtocolRequest) -> Self {
        let mut compile = CompileRequest::new(request.input_pbf_path, request.route_archive_path)
            .scratch_dir(request.temp_dir)
            .graph_id(request.graph_id)
            .memory(match request.build_profile {
                BuildProfile::DesktopFull => MemoryProfile::Standard,
                BuildProfile::MobileOperational => MemoryProfile::Low,
            });
        compile.country = request.country_code;
        compile.threads = request.threads;
        compile
    }
}

/// What a host sending the protocol's own request is told on stderr.
pub(crate) const DEPRECATED_REQUEST: &str = "the host protocol's own request \
     (inputPbfPath, tempDir, routeArchivePath, buildProfile) is deprecated and will be removed \
     before 1.0: send a CompileRequest ({\"input\", \"output\", ...}), the JSON the C ABI takes";

/// A request as JSON: a `CompileRequest`, or the protocol's own (it has
/// `inputPbfPath`), and whether it was the protocol's own.
pub(crate) fn parse_request(bytes: &[u8]) -> Result<(CompileRequest, bool), serde_json::Error> {
    let value: serde_json::Value = serde_json::from_slice(bytes)?;
    if value.get("inputPbfPath").is_some() {
        Ok((ProtocolRequest::deserialize(value)?.into(), true))
    } else {
        Ok((CompileRequest::deserialize(value)?, false))
    }
}

fn emit(event: serde_json::Value) {
    let stdout = io::stdout();
    let mut output = stdout.lock();
    // A host that has gone cannot read events; the compile carries on until
    // it is cancelled.
    let _ = serde_json::to_writer(&mut output, &event)
        .map_err(io::Error::from)
        .and_then(|()| output.write_all(b"\n"))
        .and_then(|()| output.flush());
}

fn emit_error(message: &str) {
    emit(json!({
        "protocolVersion": PROTOCOL_VERSION,
        "event": "error",
        "message": message,
    }));
}

fn emit_progress(progress: &CompileProgress, detail: &Detail<'_>) {
    match progress.stage {
        CompileStage::IndexNodes | CompileStage::CollectRoads => {
            // The compiler's own start and end reports of the ingest stages
            // carry no ingest counts and are not sent.
            if let Some(ingest) = &detail.ingest {
                emit(json!({
                    "protocolVersion": PROTOCOL_VERSION,
                    "event": "ingest",
                    "stage": ingest.stage,
                    "batchesRead": ingest.batches_read,
                    "objectsRead": ingest.objects_read,
                    "message": ingest.message,
                    "fraction": ingest.fraction,
                    "percent": progress.overall,
                }));
            }
        }
        _ => emit(json!({
            "protocolVersion": PROTOCOL_VERSION,
            "event": "graph_progress",
            "stage": progress.stage,
            "rowsWritten": detail.rows_written,
            "message": progress.message.as_deref().unwrap_or(""),
            "fraction": progress.stage_fraction,
            "percent": progress.overall,
        })),
    }
}

/// Runs the compile `source` describes, as events on stdout. Failures are
/// `error` events, not stderr text.
pub(crate) fn run(source: &Path, parent_stdin: bool, cancel: &CancelToken) -> Result<(), Failure> {
    let parent_gone = Arc::new(AtomicBool::new(false));
    if parent_stdin {
        if source == Path::new("-") {
            emit_error("--parent-stdin needs the request in a file, not on stdin");
            return Err(silent(1));
        }
        // The host holds stdin open for the lifetime of the job: EOF (the
        // host quit, crashed or was force-quit) cancels the compile.
        let cancel = cancel.clone();
        let gone = Arc::clone(&parent_gone);
        std::thread::spawn(move || {
            let mut byte = [0_u8; 1];
            while let Ok(count) = io::stdin().read(&mut byte) {
                if count == 0 {
                    break;
                }
            }
            gone.store(true, Ordering::Relaxed);
            stop_soon(&cancel);
        });
    }
    let request = match read_request(source) {
        Ok(request) => request,
        Err(failure) => {
            emit_error(&failure.message);
            return Err(silent(1));
        }
    };
    match host_protocol::compile(&request, cancel, emit_progress) {
        Ok(result) => {
            emit(json!({
                "protocolVersion": PROTOCOL_VERSION,
                "event": "complete",
                "graphNodeCount": result.graph_manifest.node_count,
                "directedEdgeCount": result.graph_manifest.directed_edge_count,
                "packageBytes": result.byte_len,
                "packageDatasetHash": result.dataset_hash,
            }));
            Ok(())
        }
        Err(CompileError::Cancelled) if parent_gone.load(Ordering::Relaxed) => {
            Err(silent(CANCELLED_STATUS))
        }
        Err(error) => {
            emit_error(&error.to_string());
            Err(silent(if error.is_cancelled() {
                CANCELLED_STATUS
            } else {
                1
            }))
        }
    }
}

/// A failure already reported as an event.
fn silent(status: i32) -> Failure {
    Failure {
        message: String::new(),
        status,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_3_requests_still_parse() {
        let (request, deprecated) = parse_request(
            br#"{"inputPbfPath": "/data/monaco.osm.pbf", "tempDir": "/tmp/work",
                 "graphId": "monaco", "countryCode": "MC",
                 "buildProfile": "mobile_operational", "routeArchivePath": "/data/routing.mrt"}"#,
        )
        .unwrap();
        assert!(deprecated);
        assert_eq!(
            request,
            CompileRequest::new("/data/monaco.osm.pbf", "/data/routing.mrt")
                .scratch_dir("/tmp/work")
                .graph_id("monaco")
                .country("MC")
                .memory(MemoryProfile::Low)
        );
        let (request, _) = parse_request(
            br#"{"inputPbfPath": "a.osm.pbf", "tempDir": "t", "graphId": "a",
                 "countryCode": null, "buildProfile": "desktop_full",
                 "routeArchivePath": "a.mrt", "threads": 4}"#,
        )
        .unwrap();
        assert_eq!(request.memory, MemoryProfile::Standard);
        assert_eq!(request.threads, Some(4));
        assert_eq!(request.country, None);
        // Unknown fields and profiles are refused, as before.
        assert!(
            parse_request(
                br#"{"inputPbfPath": "a.osm.pbf", "tempDir": "t", "graphId": "a",
                     "buildProfile": "desktop_full", "routeArchivePath": "a.mrt", "x": 1}"#
            )
            .is_err()
        );
        assert!(
            parse_request(
                br#"{"inputPbfPath": "a.osm.pbf", "tempDir": "t", "graphId": "a",
                     "buildProfile": "huge", "routeArchivePath": "a.mrt"}"#
            )
            .is_err()
        );
    }

    #[test]
    fn compile_requests_parse_too() {
        let (request, deprecated) = parse_request(
            br#"{"input": "a.osm.pbf", "output": "a.mrt", "memory": "low",
                 "profiles": ["legged", "wheeled"]}"#,
        )
        .unwrap();
        assert!(!deprecated);
        assert_eq!(
            request,
            CompileRequest::new("a.osm.pbf", "a.mrt")
                .memory(MemoryProfile::Low)
                .profiles([MobilityProfile::Legged, MobilityProfile::Wheeled])
        );
        assert!(parse_request(br#"{"input": "a.osm.pbf"}"#).is_err());
    }
}
