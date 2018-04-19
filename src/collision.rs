use nalgebra::{Isometry2, Point, Point2, Unit, Vector2};
use ncollide::{query, query::Contact, shape::Shape};
use momentum::{LinearMomentum, Momentum};
use object::transformation::Transformation;

pub trait ObjectContact {
    fn transformation(&self) -> &Transformation;
    fn transformation_mut(&mut self) -> &mut Transformation;
    fn shape(&self) -> &Shape<Point2<f32>, Isometry2<f32>>;
    fn contact_point<T: ObjectContact>(
        &self,
        other: &T,
        prediction: f32,
    ) -> Option<Contact<Point2<f32>>> {
        query::contact(
            &self.transformation().get_iso(),
            self.shape(),
            &other.transformation().get_iso(),
            other.shape(),
            prediction,
        )
    }
    fn move_pos(&mut self, vector: Vector2<f32>) {
        self.transformation_mut().translate(vector);
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
    fn move_self(&mut self, time: f32) {
        let move_amount = self.velocity() * time;
        self.move_pos(move_amount);
    }
}

pub trait Collidable: CollisionTime + ObjectContact {
    // Must return the required change of velocity to the input linear momentum
    // "at" is in world coordinate system not int reference to object
    fn collision(
        &self,
        momentum: LinearMomentum,
        normal: Unit<Vector2<f32>>,
        at: Point2<f32>,
    ) -> Vector2<f32>;
    fn change_velocity(&mut self, vector: Vector2<f32>, at: Point2<f32>);
}

pub fn collide<T, U>(first: &mut T, second: &mut U, time: f32)
where
    T: Momentum + Collidable,
    U: Collidable,
{
    let toi = first.toi(second);
    if let Some(impact_time) = toi {
        if time >= impact_time {
            first.move_self(impact_time);
            second.move_self(impact_time);
            let contact = first.contact_point(second, 0.5);
            if let Some(collision) = contact {
                let first_to =
                    Point::from_coordinates(first.transformation().vector_to(collision.world1));
                let second_to =
                    Point::from_coordinates(second.transformation().vector_to(collision.world2));
                let first_momentum = first.momentum_at(first_to.coords);
                let dv = second.collision(first_momentum, collision.normal, second_to);
                first.change_velocity(dv, first_to);
                second.change_velocity(-dv, second_to);
                first.move_self(time - impact_time);
                second.move_self(time - impact_time);
            } else {
                first.move_self(time);
                second.move_self(time);
            }
        }
    }
}
