use nalgebra::Vector2;
use nalgebra;

pub struct Circle {
    radius: f32,
}
impl Circle {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
    pub fn moi(&self, mass: f32) -> f32 {
        self.radius * self.radius * mass * 0.5
    }
}
#[derive(Debug, PartialEq)]
pub struct Moment {
    pub velocity: Vector2<f32>,
    // counter clockwise
    pub rps: f32,
    pub mass: f32,
    pub moi: f32,
    pub inv_moi: f32,
    pub inv_mass: f32,
}
#[derive(Copy, Clone)]
pub struct LinearMomentum {
    pub velocity: Vector2<f32>,
    pub mass: f32,
}
impl LinearMomentum {
    pub fn new(velocity: Vector2<f32>, mass: f32) -> Self {
        Self { velocity, mass }
    }
    pub fn change(&self, other: LinearMomentum, normal: Vector2<f32>) -> Vector2<f32> {
        -2.0 * other.mass / (self.mass + other.mass)
            * nalgebra::dot(&(self.velocity - other.velocity), &normal)
            / nalgebra::dot(&normal, &normal)
            * normal
    }
}

impl Moment {
    pub fn new(velocity: Vector2<f32>, rps: f32, mass: f32, moi: f32) -> Self {
        Self {
            velocity,
            rps,
            mass,
            moi,
            inv_moi: 1.0 / moi,
            inv_mass: 1.0 / mass,
        }
    }
    pub fn add_velocity(&mut self, accel: Vector2<f32>) {
        self.velocity += accel;
    }
    pub fn apply_force(&mut self, force: Vector2<f32>, at: Vector2<f32>) {
        let lin_force = vector_projection(force, at);
        let angular_force = force - lin_force;
        let line_norm = perp(at);
        let clockwise = nalgebra::dot(&line_norm, &force).signum();
        self.velocity += self.inv_mass * lin_force;
        self.rps += clockwise * nalgebra::dot(&angular_force, &angular_force).sqrt()
            * nalgebra::dot(&at, &at).sqrt() * self.inv_moi;
    }
    pub fn momentum_at(&self, at: Vector2<f32>, normal: Vector2<f32>) -> Vector2<f32> {
        let lin_force = vector_projection(self.velocity, normal) * self.mass;
        let angular_force = perp(at) * self.moi * self.rps;
        let applied_force = vector_projection(angular_force, normal);
        lin_force + applied_force
    }
    pub fn velocity_at(&self, at: Vector2<f32>, normal: Vector2<f32>) -> Vector2<f32> {
        let lin_force = vector_projection(self.velocity, normal);
        let angular_force = perp(at) * self.moi * self.inv_mass * self.rps;
        let applied_force = vector_projection(angular_force, normal);
        lin_force + applied_force
    }
    pub fn lin_momentum_at(&self, at: Vector2<f32>, normal: Vector2<f32>) -> LinearMomentum {
        LinearMomentum::new(self.velocity_at(at, normal), self.mass)
    }
    pub fn apply_change_velocity(&mut self, change: Vector2<f32>, at: Vector2<f32>) {
        let mass = self.mass;
        self.apply_force(change * mass, at);
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
    use nalgebra::Vector2;
    #[test]
    fn simple() {
        let circle = momentum::Circle::new(10.0);
        let moment = momentum::Moment::new(Vector2::new(1.0, 0.0), 1.0, 10.0, circle.moi(10.0));
        let mut moment2 =
            momentum::Moment::new(Vector2::new(1.0, 0.0), 0.0, 10.0, circle.moi(10.0));
        moment2.apply_force(Vector2::new(0.0, 1.0), Vector2::new(10.0, 0.0));
        assert_eq!(
            Vector2::new(10.0, 0.0),
            moment.momentum_at(Vector2::new(10.0, 0.0), Vector2::new(-1.0, 0.0))
        );
        assert_eq!(
            Vector2::new(0.0, 5000.0),
            moment.momentum_at(Vector2::new(10.0, 0.0), Vector2::new(0.0, -1.0))
        );
        assert_eq!(
            momentum::Moment::new(Vector2::new(1.0, 0.0), 0.020000001, 10.0, 500.0),
            moment2
        );
    }

}
