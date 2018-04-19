use object::*;
use momentum::*;
use nalgebra::{Isometry2, Point2, Unit, Vector2};
use ncollide::shape::{Ball, Cuboid};

pub struct MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    object: Object<T>,
    momentum: Moment,
}

impl<T> ObjectContact for MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    fn transformation(&self) -> &Transformation {
        self.object.transformation()
    }
    fn transformation_mut(&mut self) -> &mut Transformation {
        self.object.transformation_mut()
    }
    fn shape(&self) -> &Shape<Point2<f32>, Isometry2<f32>> {
        self.object.shape()
    }
}

impl<T> CollisionTime for MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    fn velocity(&self) -> Vector2<f32> {
        self.momentum.linear_moment.velocity
    }
}

impl<T> Collidable for MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    fn collision(
        &self,
        momentum: LinearMomentum,
        normal: Unit<Vector2<f32>>,
        at: Point2<f32>,
    ) -> Vector2<f32> {
        let at = self.object.transformation.vector_to(at);
        let momentum_at = self.momentum.lin_momentum_at(at);

        momentum.change(momentum_at, normal)
    }
    fn change_velocity(&mut self, velocity: Vector2<f32>, at: Point2<f32>) {
        self.momentum.apply_change_velocity(velocity, at.coords);
    }
}

impl<T> Momentum for MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    fn momentum_at(&self, at: Vector2<f32>) -> LinearMomentum {
        LinearMomentum::new(
            self.momentum.momentum_at(at) * self.momentum.linear_moment.inv_mass,
            self.momentum.linear_moment.mass,
        )
    }
}

pub type PCuboid = MovingObject<Cuboid<Vector2<f32>>>;

impl PCuboid {
    pub fn new(
        half_extents: Vector2<f32>,
        transformation: Transformation,
        momentum: Moment,
    ) -> Self {
        let shape = Cuboid::new(half_extents);
        let object = Object::new_from_transformation(shape, transformation);
        MovingObject { object, momentum }
    }
}

pub type PBall = MovingObject<Ball<f32>>;

impl PBall {
    pub fn new(radius: f32, transformation: Transformation, momentum: Moment) -> Self {
        let shape = Ball::new(radius);
        let object = Object::new_from_transformation(shape, transformation);
        MovingObject { object, momentum }
    }
}
