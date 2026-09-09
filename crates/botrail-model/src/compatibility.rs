//! Bounded, documentary connection conditions. A pass compares declarations;
//! it does not certify the wiring, mechanical fit or controller operation.
use crate::mounting::{CatalogOrder, CatalogSource, MountEvidence};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Compatibility {
    #[serde(default)]
    pub mounts: Vec<String>,
    #[serde(default)]
    pub requires_adapter: Vec<String>,
    #[serde(default)]
    pub programs: Vec<ProgramListing>,
    #[serde(default)]
    pub connections: Vec<ConnectionProfile>,
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct ProgramListing {
    pub name: String,
    pub status: ListingStatus,
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum ListingStatus {
    Listed,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum Support {
    Supported,
    Unsupported,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum Scope {
    Electrical,
    Communication,
    Software,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum ConnectionField {
    Controller,
    WristConnector,
    Pinout,
    VoltageV,
    CurrentA,
    PeakCurrentA,
    Protocol,
    SignalLogic,
    SoftwareFamily,
    SoftwareVersion,
    Plugin,
    PluginVersion,
}
impl ConnectionField {
    pub fn scope(self) -> Scope {
        match self {
            Self::Protocol | Self::SignalLogic => Scope::Communication,
            Self::Controller
            | Self::SoftwareFamily
            | Self::SoftwareVersion
            | Self::Plugin
            | Self::PluginVersion => Scope::Software,
            _ => Scope::Electrical,
        }
    }
    pub fn numeric(self) -> bool {
        matches!(self, Self::VoltageV | Self::CurrentA | Self::PeakCurrentA)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct ConnectionCondition {
    pub field: ConnectionField,
    #[serde(default)]
    pub accepted: Vec<String>,
    #[serde(default)]
    pub minimum: Option<f64>,
    #[serde(default)]
    pub maximum: Option<f64>,
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct ConnectionProfile {
    pub id: String,
    pub host: String,
    pub part_number: String,
    /// Exact upstream adapters, ordered from the robot host toward the product.
    #[serde(default)]
    pub via: Vec<String>,
    pub support: Support,
    pub evidence: Vec<MountEvidence>,
    #[serde(default)]
    pub conditions: Vec<ConnectionCondition>,
    #[serde(default)]
    pub not_applicable: Vec<Scope>,
    pub note: String,
}

/// Recorded on the existing scene ConnectionPlan. Identity is pinned so
/// replacing an arm or kit cannot reuse the former configuration silently.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(deny_unknown_fields)]
pub struct ConnectionSelection {
    pub target: String,
    pub catalog: String,
    pub host: String,
    pub profile: String,
    #[serde(default)]
    pub values: BTreeMap<ConnectionField, Value>,
    #[serde(default)]
    pub reference: Option<String>,
}

fn catalog_id(s: &str) -> bool {
    let parts: Vec<_> = s.split('/').collect();
    parts.len() == 4
        && parts.iter().enumerate().all(|(i, p)| {
            !p.is_empty()
                && p.bytes()
                    .next()
                    .is_some_and(|c| c.is_ascii_lowercase() || c.is_ascii_digit())
                && p.bytes().all(|c| {
                    c.is_ascii_lowercase()
                        || c.is_ascii_digit()
                        || b"_-".contains(&c)
                        || (i == 2 && c == b'.')
                })
        })
        && parts[3].strip_prefix('r').is_some_and(|r| {
            !r.starts_with('0') && !r.is_empty() && r.bytes().all(|c| c.is_ascii_digit())
        })
}

impl ConnectionSelection {
    pub fn validate(&self) -> Result<(), String> {
        if self.target.trim().is_empty()
            || self.profile.trim().is_empty()
            || !catalog_id(&self.catalog)
            || !catalog_id(&self.host)
        {
            return Err(
                "connection selection requires target, profile and exact catalog/host IDs".into(),
            );
        }
        for (field, value) in &self.values {
            if !(value.is_null()
                || if field.numeric() {
                    value.as_f64().is_some_and(|n| n.is_finite() && n >= 0.0)
                } else {
                    value.as_str().is_some_and(|s| !s.trim().is_empty())
                })
            {
                return Err(format!("invalid connection value for {field:?}"));
            }
        }
        Ok(())
    }
}

impl Compatibility {
    pub fn validate(
        &self,
        sources: &[CatalogSource],
        order: Option<&CatalogOrder>,
    ) -> Result<(), String> {
        let evidence = |refs: &[MountEvidence]| {
            !refs.is_empty()
                && refs.iter().all(|e| {
                    !e.section.trim().is_empty()
                        && sources
                            .get(e.source)
                            .is_some_and(|s| !s.url.trim().is_empty())
                })
                && refs.iter().any(|e| {
                    sources
                        .get(e.source)
                        .is_some_and(|s| s.kind == "manufacturer_datasheet")
                })
        };
        if self
            .programs
            .iter()
            .any(|p| p.name.trim().is_empty() || !evidence(&p.evidence))
        {
            return Err("program listing requires name and documentary evidence".into());
        }
        let mut ids = BTreeSet::new();
        for p in &self.connections {
            if !p
                .id
                .bytes()
                .next()
                .is_some_and(|c| c.is_ascii_lowercase() || c.is_ascii_digit())
                || !p
                    .id
                    .bytes()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || b"_-".contains(&c))
                || !ids.insert(&p.id)
                || !catalog_id(&p.host)
                || p.via.iter().any(|id| !catalog_id(id))
                || p.part_number.trim().is_empty()
                || order.and_then(|o| o.part_number.as_deref()) != Some(p.part_number.as_str())
                || p.note.trim().is_empty()
                || !evidence(&p.evidence)
            {
                return Err("connection profile requires unique ID, exact host/order part_number and documentary evidence".into());
            }
            let mut fields = BTreeSet::new();
            for c in &p.conditions {
                let bounds = [c.minimum, c.maximum];
                if !fields.insert(c.field)
                    || p.not_applicable.contains(&c.field.scope())
                    || !evidence(&c.evidence)
                    || bounds.iter().flatten().any(|n| !n.is_finite() || *n < 0.0)
                    || c.minimum.zip(c.maximum).is_some_and(|(a, b)| a > b)
                    || (c.field.numeric() && !c.accepted.is_empty())
                    || (!c.field.numeric() && bounds.iter().any(Option::is_some))
                    || c.accepted.iter().any(|s| s.trim().is_empty())
                {
                    return Err(
                        "invalid connection condition: fields, bounds, scope or evidence".into(),
                    );
                }
            }
        }
        Ok(())
    }

    pub fn review(
        &self,
        catalog: &str,
        host: Option<&str>,
        selected: Option<&ConnectionSelection>,
        sources: &[CatalogSource],
    ) -> Value {
        let profile = selected.and_then(|s| self.connections.iter().find(|p| p.id == s.profile));
        let (configuration, message) = match (selected, profile) {
            (None, _) => ("unknown", "No connection configuration selected"),
            (Some(_), None) => ("fail", "Selected profile is absent from this product"),
            (Some(s), Some(p))
                if s.catalog != catalog || host != Some(s.host.as_str()) || s.host != p.host =>
            {
                (
                    "fail",
                    "Selected product or host differs from the actual attachment",
                )
            }
            (_, Some(p)) if p.support == Support::Unsupported => (
                "fail",
                "Manufacturer explicitly does not support this configuration",
            ),
            (_, Some(p)) if p.support == Support::Unknown => {
                ("unknown", "Manufacturer support is unconfirmed")
            }
            _ => (
                "pass",
                "Exact product and host match the documented configuration",
            ),
        };
        let expand = |refs: &[MountEvidence]| {
            refs.iter()
                .map(|e| json!({"source": sources.get(e.source), "section": e.section}))
                .collect::<Vec<_>>()
        };
        let mut checks: Vec<Value> = profile
            .into_iter()
            .flat_map(|p| &p.conditions)
            .map(|c| {
                let value = selected.and_then(|s| s.values.get(&c.field));
                let known = if c.field.numeric() {
                    c.minimum.is_some() || c.maximum.is_some()
                } else {
                    !c.accepted.is_empty()
                };
                let result = if configuration != "pass" {
                    "not_applicable"
                } else if !known || value.is_none_or(Value::is_null) {
                    "unknown"
                } else if c.field.numeric() {
                    if value.and_then(Value::as_f64).is_some_and(|n|
                        c.minimum.is_none_or(|v| n >= v) && c.maximum.is_none_or(|v| n <= v)) {
                        "pass"
                    } else {
                        "fail"
                    }
                } else if value.and_then(Value::as_str).is_some_and(|s| c.accepted.iter().any(|v| v == s)) {
                    "pass"
                } else if matches!(c.field, ConnectionField::SoftwareVersion | ConnectionField::PluginVersion) {
                    // A documentary allowlist is not proof that other releases
                    // are incompatible. Keep an unqualified release unknown.
                    "unknown"
                } else {
                    "fail"
                };
                json!({"field": c.field, "scope": c.field.scope(), "status": result, "actual": value,
                    "accepted": c.accepted, "minimum": c.minimum, "maximum": c.maximum,
                    "note": c.note, "evidence": expand(&c.evidence)})
            }).collect();
        // Omitted input dimensions are not silently outside the review.
        if let Some(p) = profile {
            let mut required = vec![
                ConnectionField::WristConnector,
                ConnectionField::Pinout,
                ConnectionField::VoltageV,
                ConnectionField::CurrentA,
                ConnectionField::PeakCurrentA,
                ConnectionField::Protocol,
                ConnectionField::Controller,
                ConnectionField::SoftwareFamily,
                ConnectionField::SoftwareVersion,
                ConnectionField::Plugin,
                ConnectionField::PluginVersion,
            ];
            if p.conditions.iter().any(|c| {
                c.field == ConnectionField::Protocol && c.accepted.iter().any(|s| s == "digital_io")
            }) {
                required.push(ConnectionField::SignalLogic);
            }
            for field in required {
                if !p.not_applicable.contains(&field.scope())
                    && !p.conditions.iter().any(|c| c.field == field)
                {
                    checks.push(json!({"field": field, "scope": field.scope(),
                        "status": if configuration == "pass" { "unknown" } else { "not_applicable" },
                        "actual": selected.and_then(|s| s.values.get(&field)), "accepted": [],
                        "note": "Required connection condition is not documented", "evidence": []}));
                }
            }
        }
        let mut result = json!({"configuration": {"status": configuration, "message": message},
            "profile": profile, "selected": selected, "host": host, "catalog": catalog,
            "manufacturer_support": profile.map(|p| p.support), "programs": self.programs,
            "evidence": profile.map(|p| expand(&p.evidence)),
            "scope": "declared_connection_conditions", "validator_version": "connection/1"});
        for (name, scope) in [
            ("electrical", Scope::Electrical),
            ("communication", Scope::Communication),
            ("software", Scope::Software),
        ] {
            let rows: Vec<_> = checks.iter().filter(|c| c["scope"] == name).collect();
            let status = if configuration != "pass" {
                if configuration == "fail" {
                    "not_applicable"
                } else {
                    "unknown"
                }
            } else if profile.is_some_and(|p| p.not_applicable.contains(&scope)) {
                "not_applicable"
            } else if rows.iter().any(|c| c["status"] == "fail") {
                "fail"
            } else if rows.is_empty() || rows.iter().any(|c| c["status"] == "unknown") {
                "unknown"
            } else {
                "pass"
            };
            result[name] = json!({"status": status, "checks": rows});
        }
        result
    }
}
