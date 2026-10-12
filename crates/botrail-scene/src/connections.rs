//! Authored interface requirements. These reference existing residents;
//! they neither add BOM lines nor drive the rollout or the I/O map.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::part::PartTargetKind;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum Medium {
    Power,
    Pneumatic,
    Signal,
    Network,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum Role {
    Supply,
    Load,
    Input,
    Output,
    Peer,
}

/// Values apply to this endpoint in total, not to an inferred BOM quantity.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Specs {
    pub voltage_v: Option<f64>,
    pub voltage_min_v: Option<f64>,
    pub voltage_max_v: Option<f64>,
    pub current_a: Option<f64>,
    pub capacity_a: Option<f64>,
    pub pressure_bar: Option<f64>,
    pub pressure_min_bar: Option<f64>,
    pub pressure_max_bar: Option<f64>,
    pub flow_l_min: Option<f64>,
    pub capacity_l_min: Option<f64>,
    /// The reference conditions used for both flow and capacity numbers.
    pub flow_reference: Option<String>,
    pub protocol: Option<String>,
    /// digital, safe_digital, analog or word; not a second channel table.
    pub signal_type: Option<String>,
    pub logic: Option<String>,
}

/// Where a port (or a clamp) sits: on a resident, at a local offset. The
/// world point follows the resident — an obstacle's pose, a robot link's
/// forward kinematics, a frame — so a route re-resolves when things move.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum Attach {
    Obstacle {
        name: String,
        #[serde(default)]
        offset_m: [f64; 3],
    },
    Link {
        robot: String,
        link: String,
        #[serde(default)]
        offset_m: [f64; 3],
    },
    Frame {
        name: String,
        #[serde(default)]
        offset_m: [f64; 3],
    },
}

impl Attach {
    pub fn offset_m(&self) -> [f64; 3] {
        match self {
            Attach::Obstacle { offset_m, .. }
            | Attach::Link { offset_m, .. }
            | Attach::Frame { offset_m, .. } => *offset_m,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        let named = match self {
            Attach::Obstacle { name, .. } | Attach::Frame { name, .. } => !name.trim().is_empty(),
            Attach::Link { robot, link, .. } => !robot.trim().is_empty() && !link.trim().is_empty(),
        };
        if !named {
            return Err("an attachment names its resident".to_string());
        }
        if self.offset_m().iter().any(|v| !v.is_finite()) {
            return Err("attachment offset must be finite".to_string());
        }
        Ok(())
    }
}

/// The geometry of a port: where it sits, which way the line leaves it, and
/// how much of the line must stay straight out of the connector (the boot,
/// the gland). Optional — a port without it still takes part in the
/// connection checks; only a route needs it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct PortGeometry {
    pub at: Attach,
    /// Exit direction in the resident's frame (normalized on use).
    pub exit: [f64; 3],
    #[serde(default)]
    pub straight_m: f64,
}

impl PortGeometry {
    pub fn validate(&self) -> Result<(), String> {
        self.at.validate()?;
        if self.exit.iter().any(|v| !v.is_finite()) || self.exit.iter().all(|v| v.abs() < 1e-12) {
            return Err("exit must be a finite, nonzero direction".to_string());
        }
        if !self.straight_m.is_finite() || self.straight_m < 0.0 {
            return Err("straight_m must be finite and nonnegative".to_string());
        }
        Ok(())
    }
}

/// A reference into the existing I/O map. The assigned channel is derived.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct IoRef {
    pub point: String,
    pub direction: crate::iomap::IoDirection,
    pub node: String,
    #[serde(default)]
    pub aspect: Option<crate::iomap::Aspect>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Port {
    pub name: String,
    pub target: String,
    pub target_kind: PartTargetKind,
    pub medium: Medium,
    pub role: Role,
    pub required: bool,
    #[serde(default)]
    pub specs: Specs,
    #[serde(default)]
    pub io: Option<IoRef>,
    /// External drawing/terminal reference only; never a channel address.
    #[serde(default)]
    pub terminal: Option<String>,
    #[serde(default)]
    pub reference: Option<String>,
    /// Where the port sits and which way its line leaves (absent in files
    /// written before routing existed, and on ports nobody routes).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry: Option<PortGeometry>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Connection {
    pub name: String,
    pub source: String,
    pub target: String,
    #[serde(default)]
    pub cable: Option<String>,
    #[serde(default)]
    pub reference: Option<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct ConnectionPlan {
    pub ports: Vec<Port>,
    pub links: Vec<Connection>,
}

impl ConnectionPlan {
    pub fn is_empty(&self) -> bool {
        self.ports.is_empty() && self.links.is_empty()
    }

    /// Validate representation, not design correctness. Broken references
    /// survive deletion and save/load so the engineering review can name them.
    pub fn validate(&self) -> Result<(), String> {
        let mut names = BTreeSet::new();
        for port in &self.ports {
            if port.name.trim().is_empty()
                || port.target.trim().is_empty()
                || !names.insert(&port.name)
            {
                return Err(format!(
                    "invalid or duplicate connection port: {:?}",
                    port.name
                ));
            }
            if !matches!(
                (port.medium, port.role),
                (Medium::Power | Medium::Pneumatic, Role::Supply | Role::Load)
                    | (Medium::Signal, Role::Input | Role::Output)
                    | (Medium::Network, Role::Peer)
            ) {
                return Err(format!(
                    "{}: role does not match the port medium",
                    port.name
                ));
            }
            for value in [
                port.specs.voltage_v,
                port.specs.voltage_min_v,
                port.specs.voltage_max_v,
                port.specs.current_a,
                port.specs.capacity_a,
                port.specs.pressure_bar,
                port.specs.pressure_min_bar,
                port.specs.pressure_max_bar,
                port.specs.flow_l_min,
                port.specs.capacity_l_min,
            ]
            .into_iter()
            .flatten()
            {
                if !value.is_finite() || value < 0.0 {
                    return Err(format!(
                        "{}: specifications must be finite and nonnegative",
                        port.name
                    ));
                }
            }
            if port
                .specs
                .signal_type
                .as_deref()
                .is_some_and(|s| !matches!(s, "digital" | "safe_digital" | "analog" | "word"))
            {
                return Err(format!("{}: unknown signal_type", port.name));
            }
            if port
                .specs
                .logic
                .as_deref()
                .is_some_and(|s| !matches!(s, "pnp" | "npn"))
            {
                return Err(format!("{}: logic must be pnp or npn", port.name));
            }
            if matches!(port.role, Role::Supply)
                && (port.specs.current_a.is_some() || port.specs.flow_l_min.is_some())
                || matches!(port.role, Role::Load)
                    && (port.specs.capacity_a.is_some() || port.specs.capacity_l_min.is_some())
            {
                return Err(format!(
                    "{}: consumption belongs to loads, capacity to supplies",
                    port.name
                ));
            }
            let specs = serde_json::to_value(&port.specs).map_err(|e| e.to_string())?;
            for (key, value) in specs.as_object().expect("spec object") {
                if value.is_null() {
                    continue;
                }
                let allowed = match port.medium {
                    Medium::Power => {
                        key.starts_with("voltage_")
                            || matches!(key.as_str(), "current_a" | "capacity_a")
                    }
                    Medium::Pneumatic => {
                        key.starts_with("pressure_")
                            || matches!(
                                key.as_str(),
                                "flow_l_min" | "capacity_l_min" | "flow_reference"
                            )
                    }
                    Medium::Signal => {
                        key.starts_with("voltage_")
                            || matches!(key.as_str(), "signal_type" | "logic")
                    }
                    Medium::Network => key == "protocol",
                };
                if !allowed {
                    return Err(format!(
                        "{}: {key} is not a property of this medium",
                        port.name
                    ));
                }
                if let Some(v) = value.as_f64() {
                    if !v.is_finite() || v < 0.0 {
                        return Err(format!(
                            "{}: {key} must be finite and nonnegative",
                            port.name
                        ));
                    }
                }
                if value.as_str().is_some_and(|v| v.trim().is_empty()) {
                    return Err(format!("{}: {key} must be nonempty", port.name));
                }
            }
            for (low, high) in [
                (port.specs.voltage_min_v, port.specs.voltage_max_v),
                (port.specs.pressure_min_bar, port.specs.pressure_max_bar),
            ] {
                if matches!((low, high), (Some(a), Some(b)) if a > b) {
                    return Err(format!("{}: minimum exceeds maximum", port.name));
                }
            }
            if let Some(geometry) = &port.geometry {
                geometry
                    .validate()
                    .map_err(|e| format!("{}: {e}", port.name))?;
            }
            if let Some(io) = &port.io {
                if port.medium != Medium::Signal || port.target_kind != PartTargetKind::IoNode {
                    return Err(format!(
                        "{}: io references belong to signal ports on I/O nodes",
                        port.name
                    ));
                }
                if io.point.trim().is_empty() || io.node.trim().is_empty() {
                    return Err(format!("{}: io point and node must be nonempty", port.name));
                }
            }
        }
        names.clear();
        for link in &self.links {
            if [&link.name, &link.source, &link.target]
                .iter()
                .any(|s| s.trim().is_empty())
                || !names.insert(&link.name)
            {
                return Err(format!("invalid or duplicate connection: {:?}", link.name));
            }
        }
        Ok(())
    }
}

impl crate::Scene {
    pub fn connection_plan(&self) -> &ConnectionPlan {
        &self.connection_plan
    }

    pub fn set_connection_plan(&mut self, plan: ConnectionPlan) -> Result<(), String> {
        plan.validate()?;
        self.connection_plan = plan;
        self.plan_gen = self.plan_gen.wrapping_add(1);
        Ok(())
    }
}
