//! Scenery that moves on a hinge or a slide: a fence door, a refrigerator
//! door, a dishwasher rack, a drawer.
//!
//! A [`SceneryJoint`] names the obstacles that move together (the leaf, its
//! handle, whatever rides on it), the axis they move on and how far. It is
//! *passive*: nothing in a sequence drives it and no rollout reads it — it
//! is a way to pose scenery, the way a gizmo is, with the mechanism's one
//! degree of freedom instead of six. Setting it rewrites the members'
//! poses, so collision checking, planning and a bake see the door where it
//! stands.
//!
//! The axis lives in the frame of a *base* obstacle (the carcass, the
//! post the leaf hangs on), so carrying the whole unit somewhere else
//! carries its hinges along. A joint whose members hold another joint's
//! base (a bin on a door) lists that joint's members too: opening the door
//! moves the bin's slide with it.

use nalgebra::{Isometry3, Point3, Translation3, Unit, UnitQuaternion, Vector3};

use crate::{Scene, SceneError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneryJointKind {
    /// Turns about the axis; the position is an angle (rad).
    Hinge,
    /// Travels along the axis; the position is a distance (m).
    Slide,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SceneryJoint {
    pub name: String,
    pub kind: SceneryJointKind,
    /// The obstacle the joint is mounted on. `axis` and `origin` are in
    /// its frame; without one they are world coordinates.
    pub base: Option<String>,
    pub axis: Unit<Vector3<f64>>,
    /// A point on the axis (a hinge turns about it; a slide only shows it).
    pub origin: Point3<f64>,
    /// `(lower, upper)` of the position.
    pub range: (f64, f64),
    /// Where the members stand now.
    pub position: f64,
    /// The obstacles that move with it.
    pub objects: Vec<String>,
}

impl Scene {
    pub fn scenery_joints(&self) -> &[SceneryJoint] {
        &self.scenery_joints
    }

    pub fn scenery_joint(&self, name: &str) -> Option<&SceneryJoint> {
        self.scenery_joints.iter().find(|j| j.name == name)
    }

    /// Adds a joint, or replaces the one of that name. `origin` and `axis`
    /// are world coordinates as the scene stands; they are kept in the
    /// base's frame. `position` is what the members' current poses count
    /// as (a door authored shut is at 0).
    #[allow(clippy::too_many_arguments)]
    pub fn add_scenery_joint(
        &mut self,
        name: &str,
        kind: SceneryJointKind,
        objects: Vec<String>,
        origin: Point3<f64>,
        axis: Vector3<f64>,
        range: (f64, f64),
        position: f64,
        base: Option<String>,
    ) -> Result<(), SceneError> {
        let bad =
            |why: String| SceneError::BadSceneryJoint(format!("scenery joint `{name}`: {why}"));
        if objects.is_empty() {
            return Err(bad("no objects move with it".to_string()));
        }
        for object in &objects {
            self.obstacle_index(object)?;
        }
        let axis = Unit::try_new(axis, 1e-9)
            .filter(|a| a.iter().all(|v| v.is_finite()))
            .ok_or_else(|| bad("the axis has no direction".to_string()))?;
        if !(range.0.is_finite() && range.1.is_finite() && range.0 <= range.1) {
            return Err(bad(format!(
                "range ({}, {}) is not an interval",
                range.0, range.1
            )));
        }
        if !position.is_finite() || !origin.coords.iter().all(|v| v.is_finite()) {
            return Err(bad("position and origin must be finite".to_string()));
        }
        let (origin, axis) = match &base {
            Some(base) => {
                if objects.contains(base) {
                    return Err(bad(format!("its base `{base}` moves with it")));
                }
                let frame = self.obstacles[self.obstacle_index(base)?].pose.inverse();
                (
                    frame * origin,
                    Unit::new_unchecked(frame * axis.into_inner()),
                )
            }
            None => (origin, axis),
        };
        let joint = SceneryJoint {
            name: name.to_string(),
            kind,
            base,
            axis,
            origin,
            range,
            position,
            objects,
        };
        match self.scenery_joints.iter_mut().find(|j| j.name == name) {
            Some(slot) => *slot = joint,
            None => self.scenery_joints.push(joint),
        }
        Ok(())
    }

    pub fn remove_scenery_joint(&mut self, name: &str) -> bool {
        let before = self.scenery_joints.len();
        self.scenery_joints.retain(|j| j.name != name);
        self.scenery_joints.len() != before
    }

    /// The joint's origin and axis in the world, as the scene stands.
    pub fn scenery_joint_axis(&self, joint: &SceneryJoint) -> (Point3<f64>, Unit<Vector3<f64>>) {
        let base = joint
            .base
            .as_deref()
            .and_then(|base| self.obstacles.iter().find(|o| o.name == base));
        match base {
            Some(base) => (
                base.pose * joint.origin,
                Unit::new_unchecked(base.pose * joint.axis.into_inner()),
            ),
            None => (joint.origin, joint.axis),
        }
    }

    /// Moves the joint to `position`, held within its range, and its
    /// members with it. Returns where it stands.
    pub fn set_scenery_joint(&mut self, name: &str, position: f64) -> Result<f64, SceneError> {
        let joint = self
            .scenery_joint(name)
            .ok_or_else(|| SceneError::UnknownSceneryJoint(name.to_string()))?
            .clone();
        if !position.is_finite() {
            return Err(SceneError::BadSceneryJoint(format!(
                "scenery joint `{name}`: position {position} is not a number"
            )));
        }
        let to = position.clamp(joint.range.0, joint.range.1);
        let by = to - joint.position;
        let (origin, axis) = self.scenery_joint_axis(&joint);
        let motion = match joint.kind {
            SceneryJointKind::Hinge => {
                let turn = UnitQuaternion::from_axis_angle(&axis, by);
                Isometry3::from_parts(
                    Translation3::from(origin.coords - turn * origin.coords),
                    turn,
                )
            }
            SceneryJointKind::Slide => Isometry3::from_parts(
                Translation3::from(axis.into_inner() * by),
                UnitQuaternion::identity(),
            ),
        };
        for object in &joint.objects {
            if let Ok(index) = self.obstacle_index(object) {
                let mut pose = motion * self.obstacles[index].pose;
                // Thousands of small turns must not let the quaternion drift.
                pose.rotation.renormalize();
                self.set_obstacle_pose(object, pose)?;
            }
        }
        if let Some(slot) = self.scenery_joints.iter_mut().find(|j| j.name == name) {
            slot.position = to;
        }
        Ok(to)
    }

    /// Replaces every joint (a project load). What no longer fits the
    /// scene — a member or a base that is not there — is dropped.
    pub fn set_scenery_joints(&mut self, joints: Vec<SceneryJoint>) {
        self.scenery_joints = joints;
        self.prune_scenery_joints();
    }

    /// Drops what a removed obstacle leaves behind: its place among a
    /// joint's members, and the joints that hung on it or moved only it.
    pub(crate) fn prune_scenery_joints(&mut self) {
        let obstacles = &self.obstacles;
        let there = |name: &str| obstacles.iter().any(|o| o.name == name);
        self.scenery_joints.retain_mut(|joint| {
            joint.objects.retain(|object| there(object));
            !joint.objects.is_empty() && joint.base.as_deref().is_none_or(there)
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_model::Geometry;
    use std::f64::consts::FRAC_PI_2;

    fn cube() -> Geometry {
        Geometry::Box {
            size: Vector3::new(0.1, 0.1, 0.1),
        }
    }

    /// A post at the origin and a leaf a metre along +X of it.
    fn door() -> Scene {
        let mut scene = Scene::empty();
        scene
            .add_obstacle("post", cube(), Isometry3::identity())
            .unwrap();
        scene
            .add_obstacle("leaf", cube(), Isometry3::translation(1.0, 0.0, 0.5))
            .unwrap();
        scene
            .add_obstacle("handle", cube(), Isometry3::translation(0.9, -0.05, 1.0))
            .unwrap();
        scene
            .add_scenery_joint(
                "door",
                SceneryJointKind::Hinge,
                vec!["leaf".to_string(), "handle".to_string()],
                Point3::origin(),
                Vector3::z(),
                (0.0, 2.0),
                0.0,
                Some("post".to_string()),
            )
            .unwrap();
        scene
    }

    fn at(scene: &Scene, name: &str) -> Vector3<f64> {
        scene
            .obstacles()
            .iter()
            .find(|o| o.name == name)
            .unwrap()
            .pose
            .translation
            .vector
    }

    #[test]
    fn a_hinge_turns_its_members_about_the_axis() {
        let mut scene = door();
        assert_eq!(
            scene.set_scenery_joint("door", FRAC_PI_2).unwrap(),
            FRAC_PI_2
        );
        assert!((at(&scene, "leaf") - Vector3::new(0.0, 1.0, 0.5)).norm() < 1e-12);
        assert!((at(&scene, "handle") - Vector3::new(0.05, 0.9, 1.0)).norm() < 1e-12);
        assert_eq!(
            at(&scene, "post"),
            Vector3::zeros(),
            "the post it hangs on stays"
        );
        // The leaf turned with it: its +X now points along +Y.
        let leaf = &scene
            .obstacles()
            .iter()
            .find(|o| o.name == "leaf")
            .unwrap()
            .pose;
        assert!((leaf.rotation * Vector3::x() - Vector3::y()).norm() < 1e-12);
        // Back shut, by the way it came.
        scene.set_scenery_joint("door", 0.0).unwrap();
        assert!((at(&scene, "leaf") - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-12);
    }

    #[test]
    fn the_position_is_held_within_the_range() {
        let mut scene = door();
        assert_eq!(scene.set_scenery_joint("door", 9.0).unwrap(), 2.0);
        assert_eq!(scene.set_scenery_joint("door", -1.0).unwrap(), 0.0);
        assert!((at(&scene, "leaf") - Vector3::new(1.0, 0.0, 0.5)).norm() < 1e-12);
        assert!(scene.set_scenery_joint("door", f64::NAN).is_err());
        assert!(matches!(
            scene.set_scenery_joint("gate", 0.1),
            Err(SceneError::UnknownSceneryJoint(_))
        ));
    }

    #[test]
    fn a_slide_travels_along_the_axis() {
        let mut scene = door();
        scene
            .add_scenery_joint(
                "rack",
                SceneryJointKind::Slide,
                vec!["handle".to_string()],
                Point3::new(0.0, 0.0, 1.0),
                Vector3::new(0.0, -2.0, 0.0),
                (0.0, 0.4),
                0.0,
                Some("post".to_string()),
            )
            .unwrap();
        assert_eq!(scene.set_scenery_joint("rack", 0.5).unwrap(), 0.4);
        assert!((at(&scene, "handle") - Vector3::new(0.9, -0.45, 1.0)).norm() < 1e-12);
    }

    #[test]
    fn the_axis_goes_where_its_base_is_carried() {
        let mut scene = door();
        // The whole unit two metres along Y and a quarter turn about Z.
        let carry = Isometry3::new(Vector3::new(0.0, 2.0, 0.0), Vector3::z() * FRAC_PI_2);
        for name in ["post", "leaf", "handle"] {
            let pose = carry
                * scene
                    .obstacles()
                    .iter()
                    .find(|o| o.name == name)
                    .unwrap()
                    .pose;
            scene.set_obstacle_pose(name, pose).unwrap();
        }
        assert!((at(&scene, "leaf") - Vector3::new(0.0, 3.0, 0.5)).norm() < 1e-12);
        scene.set_scenery_joint("door", FRAC_PI_2).unwrap();
        // About the post where it now stands, not about the world's origin.
        assert!((at(&scene, "leaf") - Vector3::new(-1.0, 2.0, 0.5)).norm() < 1e-12);
        let (origin, axis) = scene.scenery_joint_axis(scene.scenery_joint("door").unwrap());
        assert!((origin.coords - Vector3::new(0.0, 2.0, 0.0)).norm() < 1e-12);
        assert!((axis.into_inner() - Vector3::z()).norm() < 1e-12);
    }

    #[test]
    fn a_removed_obstacle_takes_its_part_of_the_joint_with_it() {
        let mut scene = door();
        scene.remove_obstacle("handle").unwrap();
        assert_eq!(
            scene.scenery_joint("door").unwrap().objects,
            vec!["leaf".to_string()]
        );
        scene.remove_obstacle("post").unwrap();
        assert!(scene.scenery_joints().is_empty(), "no base, no joint");
    }

    #[test]
    fn what_cannot_be_a_joint_is_refused() {
        let mut scene = door();
        let mut add =
            |objects: Vec<&str>, axis: Vector3<f64>, range: (f64, f64), base: Option<&str>| {
                scene.add_scenery_joint(
                    "bad",
                    SceneryJointKind::Hinge,
                    objects.into_iter().map(String::from).collect(),
                    Point3::origin(),
                    axis,
                    range,
                    0.0,
                    base.map(String::from),
                )
            };
        assert!(
            add(vec![], Vector3::z(), (0.0, 1.0), None).is_err(),
            "nothing moves"
        );
        assert!(
            add(vec!["ghost"], Vector3::z(), (0.0, 1.0), None).is_err(),
            "unknown member"
        );
        assert!(
            add(vec!["leaf"], Vector3::zeros(), (0.0, 1.0), None).is_err(),
            "no axis"
        );
        assert!(
            add(vec!["leaf"], Vector3::z(), (1.0, 0.0), None).is_err(),
            "inverted range"
        );
        assert!(
            add(vec!["leaf"], Vector3::z(), (0.0, 1.0), Some("leaf")).is_err(),
            "its own base"
        );
        assert!(
            add(vec!["leaf"], Vector3::z(), (0.0, 1.0), Some("ghost")).is_err(),
            "unknown base"
        );
        // Replacing by name keeps one.
        add(vec!["leaf"], Vector3::z(), (0.0, 1.0), None).unwrap();
        add(vec!["leaf"], Vector3::x(), (0.0, 1.0), None).unwrap();
        assert_eq!(
            scene
                .scenery_joints()
                .iter()
                .filter(|j| j.name == "bad")
                .count(),
            1
        );
    }
}
