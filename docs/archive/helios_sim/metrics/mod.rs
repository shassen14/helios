// helios_sim/src/simulation/plugins/metrics/
//
// ControlMetricsPlugin: accumulates control-quality metrics during a run and
// prints a summary table on exit.  Optionally writes CSV to `metrics.output_path`.

pub mod control_metrics;

pub use control_metrics::ControlMetricsPlugin;
