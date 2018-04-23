use nalgebra::{Vector2};
use nalgebra;
use self::{angular::AngularMomentum, linear::LinearMomentum};

pub mod linear;
pub mod angular;
mod tests;

pub trait AMomentum {
    fn moi(&self, mass: f32) -> f32;
}
pub trait Momentum {
    fn momentum_at(&self, at: Vector2<f32>) -> LinearMomentum;
}

#[derive(Debug, PartialEq)]
pub struct Moment {
    pub angular_moment: AngularMomentum,
    pub linear_moment: LinearMomentum,
}
impl Moment {
    pub fn new(velocity: Vector2<f32>, rps: f32, mass: f32, moi: f32) -> Self {
        Self {
            angular_moment: AngularMomentum::new(moi, rps),
            linear_moment: LinearMomentum::new(velocity, mass)
        }
    }
    pub fn add_velocity(&mut self, accel: Vector2<f32>) {
        self.linear_moment.add_velocity(accel);
    }
    pub fn add_rotation(&mut self, rot: f32) {
        self.angular_moment.add_rotation(rot);
    }
    pub fn apply_force(&mut self, force: Vector2<f32>, _at: Vector2<f32>) {
        // let lin_force = vector_projection(force, at);
        // let angular_force = force - lin_force;
        self.linear_moment.apply_force(force);
        // self.angular_moment.apply_force(angular_force, at);
    }
    pub fn lin_momentum_at(&self, _at: Vector2<f32>) -> LinearMomentum{
        // let velocity = self.linear_moment.velocity
        //     + self.angular_moment
        //         .get_linear_velocity(at, self.linear_moment.inv_mass);
        // LinearMomentum::new(velocity, self.linear_moment.mass)
        self.linear_moment
    }
    pub fn apply_change_velocity(&mut self, change: Vector2<f32>, at: Vector2<f32>) {
        let mass = self.linear_moment.mass;
        self.apply_force(change * mass, at);
    }
    pub fn momentum_at(&self, at: Vector2<f32>) -> Vector2<f32> {
        self.linear_moment.momentum() + self.angular_moment.momentum_at(at)
    }
}

pub fn vector_projection(vector: Vector2<f32>, onto: Vector2<f32>) -> Vector2<f32> {
    nalgebra::dot(&vector, &onto) / nalgebra::dot(&onto, &onto) * onto
}
pub fn vector_rejection(vector: Vector2<f32>, onto: Vector2<f32>) -> Vector2<f32> {
    vector - vector_projection(vector, onto)
}
pub fn perp(vector: Vector2<f32>) -> Vector2<f32> {
    Vector2::new(-vector.y, vector.x)
}
