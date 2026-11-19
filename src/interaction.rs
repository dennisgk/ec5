use eframe::egui::{Pos2, Rect, pos2};
use glam::{Mat4, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct SurfacePoint {
    /// Rest-pose coordinates. Moving points rotate with the encoder pose.
    pub point: Vec3,
    pub moving: bool,
}
pub struct Triangle {
    pub points: [Vec3; 3],
    pub moving: bool,
}
pub struct Geometry {
    pub pivot: Vec3,
    pub center: Vec3,
    pub radius: f32,
    pub front: f32,
    pub triangles: Vec<Triangle>,
}
#[derive(Clone, Copy)]
struct Ray {
    origin: Vec3,
    direction: Vec3,
}

impl Geometry {
    pub fn view(&self) -> Mat4 {
        let r = self.radius;
        let eye = self.center + Vec3::new(r * 0.12, r * 3.0, r * 0.05);
        // Rotate the lateral camera 180 degrees: the forearm points down at 0°.
        Mat4::perspective_rh(42_f32.to_radians(), 1., r * 0.01, r * 10.)
            * Mat4::look_at_rh(eye, self.center, Vec3::NEG_Z)
    }
    pub fn project(&self, point: Vec3, rect: Rect) -> Pos2 {
        let p = self.view().project_point3(point);
        pos2(
            rect.left() + (p.x + 1.) * rect.width() * 0.5,
            rect.top() + (1. - p.y) * rect.height() * 0.5,
        )
    }
    pub fn world_point(&self, point: SurfacePoint, degrees: f32) -> Vec3 {
        if point.moving {
            self.pivot
                + Mat4::from_rotation_y(degrees.to_radians())
                    .transform_vector3(point.point - self.pivot)
        } else {
            point.point
        }
    }
    fn ray(&self, pointer: Pos2, rect: Rect) -> Ray {
        let x = 2. * (pointer.x - rect.left()) / rect.width() - 1.;
        let y = 1. - 2. * (pointer.y - rect.top()) / rect.height();
        let inverse = self.view().inverse();
        let near = inverse.project_point3(Vec3::new(x, y, 0.));
        let far = inverse.project_point3(Vec3::new(x, y, 1.));
        Ray {
            origin: near,
            direction: (far - near).normalize(),
        }
    }
    pub fn pick(&self, pointer: Pos2, rect: Rect, measured: f32) -> Option<SurfacePoint> {
        if !rect.contains(pointer) {
            return None;
        }
        let world = self.ray(pointer, rect);
        let inverse = Mat4::from_rotation_y(-measured.to_radians());
        let moving = Ray {
            origin: self.pivot + inverse.transform_vector3(world.origin - self.pivot),
            direction: inverse.transform_vector3(world.direction),
        };
        let mut nearest = f32::INFINITY;
        let mut hit = None;
        for triangle in &self.triangles {
            let ray = if triangle.moving { moving } else { world };
            if let Some(distance) = triangle_hit(ray, triangle.points)
                && distance < nearest
            {
                nearest = distance;
                hit = Some(SurfacePoint {
                    point: ray.origin + distance * ray.direction,
                    moving: triangle.moving,
                });
            }
        }
        hit
    }
    pub fn target_at_pointer(
        &self,
        point: SurfacePoint,
        pointer: Pos2,
        rect: Rect,
        reference: f32,
    ) -> Option<f32> {
        if !point.moving {
            return None;
        }
        let ray = self.ray(pointer, rect);
        if ray.direction.y.abs() < 1e-5 {
            return None;
        }
        let distance = (point.point.y - ray.origin.y) / ray.direction.y;
        if distance <= 0. {
            return None;
        }
        let desired = ray.origin + distance * ray.direction - self.pivot;
        // Direction is undefined at the elbow itself. Avoid target jumps there.
        if desired.x * desired.x + desired.z * desired.z < self.radius.powi(2) * 0.0004 {
            return None;
        }
        let rest = point.point - self.pivot;
        if rest.x * rest.x + rest.z * rest.z < self.radius.powi(2) * 0.0001 {
            return None;
        }
        let degrees = (desired.x.atan2(desired.z) - rest.x.atan2(rest.z)).to_degrees();
        let delta = (degrees - reference).to_radians();
        Some(reference + delta.sin().atan2(delta.cos()).to_degrees())
    }
    pub fn handle(&self) -> SurfacePoint {
        SurfacePoint {
            point: Vec3::new(self.pivot.x, self.front, self.pivot.z + self.radius * 0.25),
            moving: true,
        }
    }
    pub fn dial_center(&self, rect: Rect) -> Pos2 {
        self.project(Vec3::new(self.pivot.x, self.front, self.pivot.z), rect)
    }
    pub fn handle_position(&self, degrees: f32, rect: Rect) -> Pos2 {
        self.project(self.world_point(self.handle(), degrees), rect)
    }
}

fn triangle_hit(ray: Ray, points: [Vec3; 3]) -> Option<f32> {
    let edge1 = points[1] - points[0];
    let edge2 = points[2] - points[0];
    let p = ray.direction.cross(edge2);
    let determinant = edge1.dot(p);
    if determinant.abs() < 1e-7 {
        return None;
    }
    let inverse = determinant.recip();
    let t = ray.origin - points[0];
    let u = t.dot(p) * inverse;
    if !(0.0..=1.).contains(&u) {
        return None;
    }
    let q = t.cross(edge1);
    let v = ray.direction.dot(q) * inverse;
    if v < 0. || u + v > 1. {
        return None;
    }
    let distance = edge2.dot(q) * inverse;
    (distance > 0.).then_some(distance)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn geometry() -> Geometry {
        Geometry {
            pivot: Vec3::ZERO,
            center: Vec3::ZERO,
            radius: 2.,
            front: 0.1,
            triangles: vec![Triangle {
                points: [
                    Vec3::new(-0.3, 0., 0.7),
                    Vec3::new(0.3, 0., 0.7),
                    Vec3::new(0., 0., 1.3),
                ],
                moving: true,
            }],
        }
    }
    #[test]
    fn raycast_and_drag_match_encoder_pose_after_camera_flip() {
        let g = geometry();
        let rect = Rect::from_min_size(pos2(0., 0.), eframe::egui::vec2(600., 600.));
        let point = SurfacePoint {
            point: Vec3::new(0., 0., 1.),
            moving: true,
        };
        assert!(g.project(point.point, rect).y > g.dial_center(rect).y);
        for measured in [0., 30., 90., 120.] {
            let screen = g.project(g.world_point(point, measured), rect);
            let hit = g.pick(screen, rect, measured).unwrap();
            assert!(hit.moving && hit.point.distance(point.point) < 0.001);
            let desired = g.project(g.world_point(point, measured + 10.), rect);
            let target = g.target_at_pointer(hit, desired, rect, measured).unwrap();
            assert!((target - measured - 10.).abs() < 0.01);
        }
        assert!(g.pick(pos2(0., 0.), rect, 0.).is_none());
        assert!(
            g.target_at_pointer(g.handle(), g.dial_center(rect), rect, 0.)
                .is_none()
        );
    }
    #[test]
    fn nearest_surface_and_fixed_parts_are_respected() {
        let mut g = geometry();
        g.triangles.push(Triangle {
            points: [
                Vec3::new(-1., 0.5, 0.),
                Vec3::new(1., 0.5, 0.),
                Vec3::new(0., 0.5, 2.),
            ],
            moving: false,
        });
        let rect = Rect::from_min_size(pos2(0., 0.), eframe::egui::vec2(600., 600.));
        let screen = g.project(Vec3::new(0., 0.5, 1.), rect);
        assert!(!g.pick(screen, rect, 0.).unwrap().moving);
    }
    #[test]
    fn plane_drag_points_toward_cursor_at_any_radius() {
        let g = geometry();
        let rect = Rect::from_min_size(pos2(0., 0.), eframe::egui::vec2(600., 600.));
        let handle = g.handle();
        for degrees in [0., 30., 90., 120.] {
            let mut point = g.world_point(handle, degrees);
            point.x = g.pivot.x + (point.x - g.pivot.x) * 3.;
            point.z = g.pivot.z + (point.z - g.pivot.z) * 3.;
            let cursor = g.project(point, rect);
            let actual = g.target_at_pointer(handle, cursor, rect, degrees).unwrap();
            assert!((actual - degrees).abs() < 0.01);
        }
    }
}
