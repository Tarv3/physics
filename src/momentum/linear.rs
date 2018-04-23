use nalgebra::{Vector2, Unit};
use nalgebra;
use momentum::*;


#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinearMomentum {
    pub velocity: Vector2<f32>,
    pub mass: f32,
    pub inv_mass: f32,
}
impl LinearMomentum {
    pub fn new(velocity: Vector2<f32>, mass: f32) -> Self {
        Self {
            velocity,
            mass,
            inv_mass: 1.0 / mass,
        }
    }
    pub fn momentum(&self) -> Vector2<f32> {
        self.mass * self.velocity
    }
    pub fn add_velocity(&mut self, accel: Vector2<f32>) {
        self.velocity += accel;
    }
    pub fn apply_force(&mut self, force: Vector2<f32>) {
        self.velocity += force * self.inv_mass;
    }
    pub fn reflect(&self, normal: Unit<Vector2<f32>>) -> Vector2<f32> {
        -2.0 * vector_projection(self.velocity, *normal.as_ref())
    }
    // return required change to self
    pub fn change(&self, other: LinearMomentum, normal: Unit<Vector2<f32>>) -> Vector2<f32> {
        -2.0 * other.mass / (self.mass + other.mass)
            * nalgebra::dot(&(self.velocity - other.velocity), &normal)
            * normal.as_ref()
    }
}