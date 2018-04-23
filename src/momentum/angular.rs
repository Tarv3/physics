use nalgebra::{Vector2, Unit};
use nalgebra;

#[derive(Debug, PartialEq)]
pub struct AngularMomentum {
    // counter clockwise
    pub rps: f32,
    pub moi: f32,
    pub inv_moi: f32,
}
impl AngularMomentum {
    pub fn new(moi: f32, rps: f32) -> Self {
        Self {
            rps,
            moi,
            inv_moi: 1.0 / moi,
        }
    }
    pub fn apply_force(&mut self, force: Vector2<f32>, at: Vector2<f32>) {
        let clockwise = nalgebra::dot(&perp(at), &force).signum();
        self.rps += clockwise * nalgebra::dot(&force, &force).sqrt()
            * nalgebra::dot(&at, &at).sqrt() * self.inv_moi;
    }
    pub fn add_rotation(&mut self, rot: f32) {
        self.rps += rot;
    }
    pub fn momentum_at(&self, at: Vector2<f32>) -> Vector2<f32> {
        perp(at) * self.moi * self.rps
    }
    pub fn get_linear_velocity(&self, at: Vector2<f32>, inv_mass: f32) -> Vector2<f32> {
        perp(at) * self.moi * inv_mass * self.rps
    }
}