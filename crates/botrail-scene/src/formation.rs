//! Formations: several vehicles carrying one load as one machine
//! (design-cooperative-transport.md). A formation is a vehicle whose path
//! is the load's; its members are ordinary vehicles that drive in under the
//! load on their own, ride the formation's motion from their slots while
//! it travels, and drive off on their own again. The rollout half — the
//! boarding, the ride, the sideways check — lives in `rollout.rs` beside
//! the lift's, whose cargo vehicles ride the same way.

use nalgebra::Isometry3;

use crate::seq::{Device, DeviceKind, Drive, FormationMember};
use crate::Scene;

/// How far a member may stand off its slot when the formation leaves,
/// metres — a platform's published stop accuracy (KUKA KMP 3000P: ±5 mm).
pub const SLOT_TOL: f64 = 5e-3;
/// How far a member may face off its slot when the formation leaves (1°).
pub const SLOT_YAW_TOL: f64 = std::f64::consts::PI / 180.0;
/// The sideways speed a differential member may be asked for, m/s: none,
/// to float dust.
pub const SIDEWAYS_TOL: f64 = 1e-3;

impl Scene {
    /// A formation the rollout can carry: members that are vehicles of a
    /// ground drive, each listed once, none itself a formation, and rates
    /// no member has to exceed — a formation goes no faster than its
    /// slowest member, and turning about its centre drives the outer ones
    /// fastest. Run with the other device checks on every simulate.
    pub(crate) fn validate_formation(&self, device: &Device) -> Result<(), String> {
        let DeviceKind::Vehicle {
            members,
            drive,
            speed,
            turn_speed,
            ..
        } = &device.kind
        else {
            return Ok(());
        };
        if members.is_empty() {
            return Ok(());
        }
        let name = &device.name;
        let turns = match drive {
            Drive::Differential { .. } => true,
            Drive::Holonomic { .. } => false,
            Drive::Steered { .. } | Drive::Aerial { .. } => {
                return Err(format!(
                    "formation `{name}` needs a differential or holonomic drive — it pivots \
                     about its centre or translates, never rounds an arc or flies"
                ))
            }
        };
        let fastest = drive
            .reverse_speed_stated()
            .map_or(*speed, |v| v.max(*speed));
        for (i, member) in members.iter().enumerate() {
            let m = &member.vehicle;
            if m == name {
                return Err(format!("formation `{name}` lists itself as a member"));
            }
            if members[..i].iter().any(|o| &o.vehicle == m) {
                return Err(format!("formation `{name}` lists member `{m}` twice"));
            }
            let (t, r) = (member.slot.translation.vector, member.slot.rotation);
            if !(t.iter().all(|v| v.is_finite()) && r.coords.iter().all(|v| v.is_finite())) {
                return Err(format!(
                    "formation `{name}` member `{m}` has a non-finite slot"
                ));
            }
            let found = self.devices.iter().find(|d| &d.name == m);
            let Some(Device {
                kind:
                    DeviceKind::Vehicle {
                        members: nested,
                        drive: member_drive,
                        speed: member_speed,
                        turn_speed: member_turn,
                        ..
                    },
                ..
            }) = found
            else {
                return Err(match found {
                    Some(_) => format!("formation `{name}` member `{m}` is not a vehicle"),
                    None => format!("formation `{name}` names unknown member `{m}`"),
                });
            };
            if !nested.is_empty() {
                return Err(format!(
                    "formation `{name}` member `{m}` is itself a formation — formations do not nest"
                ));
            }
            if !matches!(
                member_drive,
                Drive::Differential { .. } | Drive::Holonomic { .. }
            ) {
                return Err(format!(
                    "formation `{name}` member `{m}` needs a differential or holonomic drive \
                     to ride a formation"
                ));
            }
            if let Some(walker) = self.robots.iter().find(|r| {
                r.mount
                    .as_ref()
                    .is_some_and(|mount| &mount.device == m && mount.gait.is_some())
            }) {
                return Err(format!(
                    "formation `{name}` member `{m}` is walked by robot `{}` — a formation \
                     carries wheeled members",
                    walker.name
                ));
            }
            if fastest > member_speed + 1e-9 {
                return Err(format!(
                    "formation `{name}` drives at {fastest:.3} m/s, over member `{m}`'s \
                     {member_speed:.3} m/s — a formation goes no faster than its slowest member"
                ));
            }
            if !turns {
                continue;
            }
            if *turn_speed > member_turn + 1e-9 {
                return Err(format!(
                    "formation `{name}` turns at {turn_speed:.3} rad/s, over member `{m}`'s \
                     turn_speed {member_turn:.3} rad/s"
                ));
            }
            // Turning about its centre, the formation carries each member
            // round a circle of the slot's radius.
            let radius = t.x.hypot(t.y);
            if turn_speed * radius > member_speed + 1e-9 {
                return Err(format!(
                    "formation `{name}` turns at {turn_speed:.3} rad/s, which drives member \
                     `{m}` ({radius:.3} m from the centre) at {:.3} m/s, over its \
                     {member_speed:.3} m/s — turn_speed ≤ {:.3} rad/s",
                    turn_speed * radius,
                    member_speed / radius
                ));
            }
        }
        Ok(())
    }

    /// Makes vehicle `formation` a formation of `members` (an empty list
    /// makes it an ordinary vehicle again). Checked on simulate, so the
    /// members may be added later.
    pub fn set_formation_members(
        &mut self,
        formation: &str,
        members: Vec<FormationMember>,
    ) -> Result<(), String> {
        let Some(device) = self.devices.iter_mut().find(|d| d.name == formation) else {
            return Err(format!("unknown device `{formation}`"));
        };
        let DeviceKind::Vehicle { members: slots, .. } = &mut device.kind else {
            return Err(format!("`{formation}` is not a vehicle"));
        };
        *slots = members;
        Ok(())
    }

    /// Where member `member` stands — its vehicle frame — when formation
    /// `formation` has driven to `station`: from its start, or from its
    /// start by way of `via`. The frame the goto parks the formation at
    /// (the legs a goto builds, walked), composed with the member's slot:
    /// the waypoint a member's own path needs, so it is back on its path
    /// when the formation lets it go.
    pub fn formation_slot(
        &self,
        formation: &str,
        member: &str,
        station: &str,
        via: Option<&str>,
    ) -> Result<Isometry3<f64>, String> {
        let Some(device) = self.devices.iter().find(|d| d.name == formation) else {
            return Err(format!("unknown device `{formation}`"));
        };
        let DeviceKind::Vehicle {
            path,
            drive,
            speed,
            turn_speed,
            start,
            members,
            ..
        } = &device.kind
        else {
            return Err(format!("`{formation}` is not a vehicle"));
        };
        let Some(slot) = members.iter().find(|m| m.vehicle == member).map(|m| m.slot) else {
            return Err(format!(
                "formation `{formation}` has no member `{member}` (members: {})",
                members
                    .iter()
                    .map(|m| format!("`{}`", m.vehicle))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        };
        let frame =
            crate::rollout::arrival_frame(path, drive, *speed, *turn_speed, start, via, station)
                .map_err(|m| format!("formation `{formation}`: {m}"))?;
        Ok(frame * slot)
    }

    /// Where each formation member is set down, member -> floor points: its
    /// slot at every station of every formation (the frame a goto from the
    /// start parks there, as [`Scene::formation_slot`] gives it). A member's
    /// own path holds these points; the leg between two of them is ridden,
    /// not driven — what a drawing of the member's tape leaves out.
    pub fn formation_set_downs(&self) -> std::collections::HashMap<String, Vec<[f64; 2]>> {
        let mut out: std::collections::HashMap<String, Vec<[f64; 2]>> = Default::default();
        for device in &self.devices {
            let DeviceKind::Vehicle { path, members, .. } = &device.kind else {
                continue;
            };
            for (station, _) in &path.stations {
                for member in members {
                    if let Ok(slot) =
                        self.formation_slot(&device.name, &member.vehicle, station, None)
                    {
                        let t = slot.translation.vector;
                        out.entry(member.vehicle.clone())
                            .or_default()
                            .push([t.x, t.y]);
                    }
                }
            }
        }
        out
    }
}
