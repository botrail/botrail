//! Wheel appearance derived from the guided vehicle's checked motion.
//! No wheel forces, slip or collision geometry are introduced.

use nalgebra::{Point3, Unit, Vector3};
use serde::{Deserialize, Serialize};

use crate::rollout::{ObjectTrack, TrackSpan};
use crate::seq::DeviceKind;
use crate::Scene;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct VehicleWheel {
    pub object: String,
    /// Effective rolling radius in metres.
    pub radius: f64,
    /// Positive axle and its centre, both in the visual obstacle's local frame.
    pub axis: [f64; 3],
    pub pivot: [f64; 3],
    /// Lateral travel / rolling travel coupling: 0 for an ordinary wheel;
    /// -1 (front-left, rear-right) or +1 (front-right, rear-left) for
    /// the usual 45-degree mecanum arrangement, with axles pointing +Y.
    pub lateral_ratio: f64,
}

impl VehicleWheel {
    pub(crate) fn validate(&self, scene: &Scene, body: &[String]) -> Result<(), String> {
        let fail = |why: &str| format!("wheel `{}`: {why}", self.object);
        if !self.radius.is_finite() || self.radius <= 0.0 {
            return Err(fail("radius must be finite and positive"));
        }
        let axis_length = Vector3::from(self.axis).norm();
        if !self.axis.iter().chain(&self.pivot).all(|v| v.is_finite())
            || !self.lateral_ratio.is_finite()
            || !axis_length.is_finite()
            || axis_length < 1e-9
        {
            return Err(fail(
                "axis must be non-zero; axis, pivot and lateral_ratio must be finite",
            ));
        }
        if !body.contains(&self.object) {
            return Err(fail("visual must belong to the vehicle body"));
        }
        let Some(obstacle) = scene.obstacles().iter().find(|o| o.name == self.object) else {
            return Err(fail("unknown visual obstacle"));
        };
        if obstacle.enabled
            || obstacle
                .physics
                .as_ref()
                .is_some_and(|p| p.kind == botrail_physics::BodyKind::Dynamic)
        {
            return Err(fail(
                "use a disabled, non-dynamic visual; keep collisions separate",
            ));
        }
        let axle = obstacle.pose.rotation * Vector3::from(self.axis).normalize();
        if axle.z.abs() > 1e-6 {
            return Err(fail("the axle must be horizontal"));
        }
        Ok(())
    }
}

impl Scene {
    /// Register or replace one display wheel on an existing vehicle.
    pub fn set_vehicle_wheel(&mut self, vehicle: &str, wheel: VehicleWheel) -> Result<(), String> {
        let index = self
            .devices
            .iter()
            .position(|d| d.name == vehicle)
            .ok_or_else(|| format!("unknown vehicle `{vehicle}`"))?;
        let DeviceKind::Vehicle { body, .. } = &self.devices[index].kind else {
            return Err(format!("device `{vehicle}` is not a vehicle"));
        };
        wheel.validate(self, body)?;
        let DeviceKind::Vehicle { wheels, .. } = &mut self.devices[index].kind else {
            unreachable!()
        };
        if let Some(existing) = wheels.iter_mut().find(|w| w.object == wheel.object) {
            *existing = wheel;
        } else {
            wheels.push(wheel);
        }
        Ok(())
    }
}

/// Decorate the finished object tracks. The scan, sensors, planning and
/// collision checks all used the original rigid body poses.
pub(crate) fn animate(scene: &Scene, tracks: &mut [ObjectTrack]) {
    for device in scene.devices() {
        let DeviceKind::Vehicle { wheels, .. } = &device.kind else {
            continue;
        };
        for wheel in wheels {
            let Some(track) = tracks.iter_mut().find(|t| t.name == wheel.object) else {
                continue;
            };
            let axis = Unit::new_normalize(Vector3::from(wheel.axis));
            let pivot = Point3::from(wheel.pivot);
            let mut angle = 0.0;
            for span in &mut track.spans {
                let rate = angular_rate(span, wheel);
                let (t0, t1) = span.range();
                *span = TrackSpan::Wheel {
                    motion: Box::new(span.clone()),
                    axis,
                    pivot,
                    angle,
                    rate,
                };
                angle += rate * (t1 - t0);
            }
        }
    }
}

/// The centre's velocity projected onto forward + k * lateral, divided
/// by radius. A pivot's centre velocity includes its distance from the
/// turn centre; both axle and velocity rotate together, so the rate is
/// constant over each exact Linear/Pivot span.
fn angular_rate(span: &TrackSpan, wheel: &VehicleWheel) -> f64 {
    let (from, velocity) = match span {
        TrackSpan::Linear { from, velocity, .. } => (from, *velocity),
        TrackSpan::Pivot {
            from,
            center,
            omega,
            ..
        } => {
            let hub = from * Point3::from(wheel.pivot);
            (from, Vector3::z().cross(&(hub - center)) * *omega)
        }
        _ => return 0.0,
    };
    let axle = from.rotation * Vector3::from(wheel.axis).normalize();
    let forward = axle.cross(&Vector3::z());
    velocity.dot(&(forward + axle * wheel.lateral_ratio)) / wheel.radius
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::SequenceTimeline;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    #[test]
    fn mecanum_rates_include_sideways_travel_and_turn_radius() {
        for (x, y, lateral) in [
            (0.2, 0.3, -1.0),
            (0.2, -0.3, 1.0),
            (-0.2, 0.3, 1.0),
            (-0.2, -0.3, -1.0),
        ] {
            let wheel = VehicleWheel {
                object: "wheel".into(),
                radius: 0.1,
                axis: [0.0, 1.0, 0.0],
                pivot: [0.0; 3],
                lateral_ratio: lateral,
            };
            let from = Isometry3::translation(x, y, 0.1);
            let straight = TrackSpan::Linear {
                t0: 0.0,
                t1: 1.0,
                from,
                velocity: Vector3::new(-0.5, 0.2, 0.0),
            };
            assert!((angular_rate(&straight, &wheel) - (-5.0 + lateral * 2.0)).abs() < 1e-9);
            let turn = TrackSpan::Pivot {
                t0: 0.0,
                t1: 1.0,
                from,
                center: Point3::origin(),
                omega: 1.0,
            };
            // A positive yaw turns both left wheels backwards and both
            // right wheels forwards, at (half_length + half_width) / r.
            let expected = if y > 0.0 { -5.0 } else { 5.0 };
            assert!((angular_rate(&turn, &wheel) - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn local_pivot_and_axle_work_in_a_rotated_visual_frame() {
        let from = Isometry3::from_parts(
            Translation3::new(0.2, 0.3, 0.1),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let pivot = Point3::new(0.02, 0.0, 0.0);
        let motion = TrackSpan::Linear {
            t0: 2.0,
            t1: 3.0,
            from,
            velocity: Vector3::new(0.4, 0.0, 0.0),
        };
        let wheel = VehicleWheel {
            object: "wheel".into(),
            radius: 0.1,
            axis: [1.0, 0.0, 0.0],
            pivot: pivot.into(),
            lateral_ratio: 0.0,
        };
        let rate = angular_rate(&motion, &wheel);
        assert!((rate - 4.0).abs() < 1e-9);
        let span = TrackSpan::Wheel {
            motion: Box::new(motion),
            axis: Vector3::x_axis(),
            pivot,
            angle: 1.3,
            rate,
        };
        for t in [2.8, 2.0, 2.4, 3.0, 3.5] {
            let pose = SequenceTimeline::span_pose(std::slice::from_ref(&span), &[], t).unwrap();
            let elapsed = (t - 2.0).clamp(0.0, 1.0);
            let expected_hub = from * pivot + Vector3::new(0.4 * elapsed, 0.0, 0.0);
            assert!((pose * pivot - expected_hub).norm() < 1e-9);
            let relative = from.rotation.inverse() * pose.rotation;
            let expected =
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.3 + rate * elapsed);
            assert!(relative.angle_to(&expected) < 1e-9);
        }
    }
}
