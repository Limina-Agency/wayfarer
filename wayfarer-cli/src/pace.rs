//! `wayfarer pace`: the walking pace model. `default` prints the built-in
//! model; `calibrate` fits one to a team's GPX tracks and prints it with a
//! report of what was used.

use std::path::PathBuf;

use clap::{Args, Subcommand, ValueEnum};
use serde_json::json;
use wayfarer::pace::{PaceModel, PaceTrailClass};
use wayfarer::pace_calibration::{
    PaceCalibrationError, PaceCalibrationOptions, calibrate_pace, parse_gpx,
};

use crate::{Failure, print_json};

/// The walking pace model: print the default, or calibrate one from GPX tracks.
#[derive(Debug, Args)]
pub(crate) struct PaceArgs {
    #[command(subcommand)]
    command: PaceCommand,
}

#[derive(Debug, Subcommand)]
enum PaceCommand {
    /// Print the built-in pace model as JSON.
    Default,
    /// Fit a pace model to a team's GPX tracks; print `{model, report}` as JSON.
    Calibrate(CalibrateArgs),
}

#[derive(Debug, Args)]
struct CalibrateArgs {
    /// GPX files with timestamped track points (and heights in `<ele>`).
    #[arg(value_name = "track.gpx", required = true)]
    gpx: Vec<PathBuf>,
    /// The model to start from (a calibration file); the built-in one by default.
    #[arg(long, value_name = "model.json")]
    base: Option<PathBuf>,
    /// A label stored in the model (team, date).
    #[arg(long, default_value = "")]
    label: String,
    /// What the tracks were walked on.
    #[arg(long, value_enum, default_value_t = TrailArg::Unknown)]
    trail: TrailArg,
    /// Only scale the flat speed; keep the base slope curve.
    #[arg(long)]
    no_curve: bool,
    /// Also write the model alone to this file.
    #[arg(long, short = 'o', value_name = "model.json")]
    output: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum TrailArg {
    Paved,
    Track,
    Path,
    MountainPath,
    AlpinePath,
    Steps,
    Unknown,
}

impl From<TrailArg> for PaceTrailClass {
    fn from(value: TrailArg) -> Self {
        match value {
            TrailArg::Paved => Self::Paved,
            TrailArg::Track => Self::Track,
            TrailArg::Path => Self::Path,
            TrailArg::MountainPath => Self::MountainPath,
            TrailArg::AlpinePath => Self::AlpinePath,
            TrailArg::Steps => Self::Steps,
            TrailArg::Unknown => Self::Unknown,
        }
    }
}

pub(crate) fn run(args: PaceArgs) -> Result<(), Failure> {
    match args.command {
        PaceCommand::Default => print_json(&PaceModel::default()),
        PaceCommand::Calibrate(args) => calibrate(args),
    }
}

fn read(path: &PathBuf) -> Result<String, Failure> {
    std::fs::read_to_string(path)
        .map_err(|error| Failure::new(format!("{}: {error}", path.display())))
}

fn calibrate(args: CalibrateArgs) -> Result<(), Failure> {
    let base = match &args.base {
        Some(path) => PaceModel::from_json(&read(path)?)
            .map_err(|error| Failure::new(format!("{}: {error}", path.display())))?,
        None => PaceModel::default(),
    };
    let mut tracks = Vec::new();
    for path in &args.gpx {
        let parsed = parse_gpx(&read(path)?)
            .map_err(|error| Failure::new(format!("{}: {error}", path.display())))?;
        tracks.extend(parsed);
    }
    let options = PaceCalibrationOptions {
        label: args.label,
        class: args.trail.into(),
        refit_curve: !args.no_curve,
        ..PaceCalibrationOptions::default()
    };
    let (model, report) =
        calibrate_pace(&tracks, &base, None, &options).map_err(|error| match error {
            PaceCalibrationError::NotEnoughData { ref report, .. } => Failure::new(format!(
                "{error}; report: {}",
                serde_json::to_string(report).unwrap_or_default()
            )),
            other => Failure::new(other.to_string()),
        })?;
    if let Some(path) = &args.output {
        let text = serde_json::to_string_pretty(&model)
            .map_err(|error| Failure::new(format!("Failed to encode the model: {error}")))?;
        std::fs::write(path, text + "\n")
            .map_err(|error| Failure::new(format!("{}: {error}", path.display())))?;
    }
    print_json(&json!({ "model": model, "report": report }))
}
