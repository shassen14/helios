use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "kind")]
#[serde(rename_all = "PascalCase")]
pub enum SlamConfig {
    EkfSlam(EkfSlamConfig),
    FactorGraphSlam(FactorGraphSlamConfig),
}

impl SlamConfig {
    pub fn get_kind_str(&self) -> &str {
        match self {
            SlamConfig::EkfSlam(_) => "EkfSlam",
            SlamConfig::FactorGraphSlam(_) => "FactorGraphSlam",
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct EkfSlamConfig {/* TODO: params for EKF-SLAM */}

#[derive(Debug, Deserialize, Clone)]
pub struct FactorGraphSlamConfig {/* TODO: params for FG-SLAM */}
