use object::*;
use momentum::*;
use ncollide::query::Contact;
use nalgebra::{Isometry2, Point2, Unit, Vector2};
use collision::*;

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
        let at = self.object.transformation.vector_to(at);
        self.momentum.apply_change_velocity(velocity, at);
    }
}
