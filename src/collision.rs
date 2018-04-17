use nalgebra::{Isometry, Isometry2, Point2, Translation, Unit, UnitComplex, Vector2,
               core::dimension::U2};
use ncollide::{query, shape, query::Contact, shape::Shape};
use momentum::{LinearMomentum, Momentum};
use object::Transformation;

pub trait ObjectContact {
    fn transformation(&self) -> &Transformation;
    fn shape(&self) -> &Shape<Point2<f32>, Isometry2<f32>>;
    fn contact_point<T: ObjectContact>(&self, other: &T, prediction: f32) -> Option<Contact<Point2<f32>>> {
        query::contact(
            &self.transformation().get_iso(),
            self.shape(),
            &other.transformation().get_iso(),
            other.shape(),
            prediction
        )
    }
}

pub trait CollisionTime: ObjectContact {
    fn velocity(&self) -> Vector2<f32>;
    fn toi<T: CollisionTime>(&self, other: &T) -> Option<f32> {
        query::time_of_impact(
            &self.transformation().get_iso(),
            &self.velocity(),
            self.shape(),
            &other.transformation().get_iso(),
            &other.velocity(),
            other.shape(),
        )
    }
}

pub trait Collidable: CollisionTime + ObjectContact {
    // Must return the required change of velocity to the input linear momentum
    // "at" is in world coordinate system not int reference to object
    fn collision (
        &self,
        momentum: LinearMomentum,
        normal: Unit<Vector2<f32>>,
        at: Point2<f32>
    ) -> Vector2<f32>;
    fn change_velocity(&mut self, vector: Vector2<f32>, at: Point2<f32>);
}
