use bevy::prelude::Resource;
use clap::Parser;
use std::path::PathBuf;

/// Helios: A professional-grade 3D robotics simulator.
///
/// This struct defines the command-line arguments that can be passed to any
/// binary application that uses the Helios simulation library.
#[derive(Parser, Debug, Resource, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Cli {
    /// The path to the scenario TOML file to run.
    #[arg(
        short,
        long,
        default_value = "configs/sim/scenarios/00_tutorial_showcase.toml"
    )]
    pub scenario: PathBuf,

    /// Root directory for all Helios config files (catalog, scenarios, hardware).
    /// Defaults to the `configs/` directory at the workspace root.
    #[arg(long, default_value = "configs")]
    pub config_root: PathBuf,

    /// Run the simulation in headless mode (without a graphical window).
    #[arg(long, default_value_t = false)]
    pub headless: bool,

    /// Master RNG seed for a reproducible run. Overrides the scenario file's
    /// `[simulation] seed`. Omit to fall back to the file value, or to OS
    /// entropy when the file leaves it unset.
    #[arg(long)]
    pub seed: Option<u64>,
}
