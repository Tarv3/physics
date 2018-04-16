use object::*;
use momentum::*;
use ncollide::query::Contact;
use nalgebra::{Isometry2, Point2, Unit, Vector2};

pub struct MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
    object: Object<T>,
    momentum: Moment,
}

impl<T> MovingObject<T>
where
    T: Shape<Point2<f32>, Isometry2<f32>>,
{
}
