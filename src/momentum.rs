use nalgebra::{Vector2, Unit};
use nalgebra;

pub trait Momentum {
    fn moi(&self, mass: f32) -> f32;
}

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
    // return required change to self
    pub fn change(&self, other: LinearMomentum, normal: Unit<Vector2<f32>>) -> Vector2<f32> {
        -2.0 * other.mass / (self.mass + other.mass)
            * nalgebra::dot(&(self.velocity - other.velocity), &normal)
            * normal.as_ref()
    }
}

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
    pub fn apply_force(&mut self, force: Vector2<f32>, at: Vector2<f32>) {
        let lin_force = vector_projection(force, at);
        let angular_force = force - lin_force;
        self.linear_moment.apply_force(lin_force);
        self.angular_moment.apply_force(angular_force, at);
    }
    pub fn lin_momentum_at(&self, at: Vector2<f32>) -> LinearMomentum{
        let velocity = self.linear_moment.velocity
            + self.angular_moment
                .get_linear_velocity(at, self.linear_moment.mass);
        LinearMomentum::new(velocity, self.linear_moment.mass)
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

#[cfg(test)]
mod tests {
    use momentum;
    use nalgebra::{Vector2, Unit};
    #[test]
    fn simple() {
        let moment = momentum::Moment::new(Vector2::new(1.0, 0.0), 1.0, 10.0, 500.0);
        let mut moment2 = momentum::Moment::new(Vector2::new(1.0, 0.0), 0.0, 10.0, 500.0);
        moment2.apply_force(Vector2::new(0.0, 1.0), Vector2::new(10.0, 0.0));
        assert_eq!(
            momentum::Moment::new(Vector2::new(1.0, 0.0), 0.020000001, 10.0, 500.0),
            moment2
        );
    }
    #[test]
    fn collisions() {
        let moment = momentum::LinearMomentum::new(Vector2::new(1.0, 1.0), 10.0);
        let moment2 = momentum::LinearMomentum::new(Vector2::new(-1.0, 1.0), 10.0);

        assert_eq!(Vector2::new(-2.0, 0.0), moment.change(moment2, Unit::new_normalize(Vector2::new(-1.0, 0.0))));
    }

}
