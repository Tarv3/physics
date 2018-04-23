use nalgebra::{Isometry2, Point2, Unit, Vector2};
use ncollide::{query, shape, query::Contact, shape::Shape};
use momentum::{AMomentum, linear::LinearMomentum};
use self::transformation::Transformation;
use collision::*;

mod test;
pub mod transformation;
pub mod moving;

pub trait Vertices
{
    // first point must be [0,0] must be in triangle fan form
    fn vertices(&self) -> Vec<[f32; 2]> ;
    fn number_of_vertices(&self) -> usize;
}

pub struct Object<T: Shape<Point2<f32>, Isometry2<f32>>> {
    shape: T,
    transformation: Transformation,
}

impl<T: Shape<Point2<f32>, Isometry2<f32>>> Object<T> {
    pub fn new(shape: T, position: Point2<f32>, rotation: f32) -> Self {
        Self {
            shape,
            transformation: Transformation::new(position, rotation),
        }
    }
    pub fn new_from_transformation(shape: T, transformation: Transformation) -> Self {
        Self {
            shape,
            transformation,
        }
    }
    pub fn translate(&mut self, vector: Vector2<f32>) {
        self.transformation.translate(vector);
    }
    pub fn rotate(&mut self, angle: f32) {
        self.transformation.rotate(angle);
    }
    pub fn contact_with<U: Shape<Point2<f32>, Isometry2<f32>>>(
        &self,
        other: &Object<U>,
        prediction: f32,
    ) -> Option<Contact<Point2<f32>>> {
        query::contact(
            &self.transformation.get_iso(),
            &self.shape,
            &other.transformation.get_iso(),
            &other.shape,
            prediction,
        )
    }
}

impl<T: Shape<Point2<f32>, Isometry2<f32>>> ObjectContact for Object<T> {
    fn transformation(&self) -> &Transformation {
        &self.transformation
    }
    fn transformation_mut(&mut self) -> &mut Transformation {
        &mut self.transformation
    }
    fn shape(&self) -> &Shape<Point2<f32>, Isometry2<f32>> {
        &self.shape
    }
}

impl<T: Shape<Point2<f32>, Isometry2<f32>>> CollisionTime for Object<T> {
    fn velocity(&self) -> Vector2<f32> {
        Vector2::new(0.0, 0.0)
    }
}

impl<T: Shape<Point2<f32>, Isometry2<f32>>> Collidable for Object<T> {
    fn collision(
        &self,
        momentum: LinearMomentum,
        normal: Unit<Vector2<f32>>,
        _: Point2<f32>,
    ) -> Vector2<f32> {
        momentum.reflect(normal)
    }
    fn change_velocity(&mut self, _: Vector2<f32>, _: Point2<f32>) {}
}

impl AMomentum for Object<shape::Cuboid<Vector2<f32>>> {
    fn moi(&self, mass: f32) -> f32 {
        let dimensions = self.shape.half_extents();
        0.5 * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y)
    }
}
impl AMomentum for Object<shape::Ball<f32>> {
    fn moi(&self, mass: f32) -> f32 {
        0.5 * mass * self.shape.radius() * self.shape.radius()
    }
}

impl Vertices for Object<shape::Cuboid<Vector2<f32>>> {
    fn number_of_vertices(&self) -> usize {
        4
    }
    fn vertices(&self) -> Vec<[f32; 2]> {
        let t = self.transformation.get_iso();
        let centre = self.transformation.position.coords;
        let tr = *self.shape.half_extents();
        let tl = Vector2::new(-tr.x, tr.y);
        let points = [
            t * Point2::from_coordinates(tr),
            t * Point2::from_coordinates(tl),
            t * Point2::from_coordinates(-tr),
            t * Point2::from_coordinates(-tl),
        ];
        return vec![
            [centre.x, centre.y],
            [points[0].x, points[0].y],
            [points[1].x, points[1].y],
            [points[2].x, points[2].y],
            [points[3].x, points[3].y],
            [points[0].x, points[0].y],
        ]
    }
}
impl Vertices for Object<shape::Ball<f32>> {
    fn number_of_vertices(&self) -> usize {
        8
    }
    fn vertices(&self) -> Vec<[f32; 2]> {
        let t = self.transformation.get_iso();
        let centre = self.transformation.position.coords;
        let rad = self.shape.radius();
        let up = Vector2::new(0.0, rad);
        let right = Vector2::new(rad, 0.0);
        let mid = rad * Vector2::new(0.5_f32.sqrt(), 0.5_f32.sqrt());
        let rmid = Vector2::new(-mid.x, mid.y);
        let points = [
            t * Point2::from_coordinates(up),
            t * Point2::from_coordinates(mid),
            t * Point2::from_coordinates(right),
            t * Point2::from_coordinates(-rmid),
            t * Point2::from_coordinates(-up),
            t * Point2::from_coordinates(-mid),
            t * Point2::from_coordinates(-right),
            t * Point2::from_coordinates(rmid),
        ];
        return vec![
            [centre.x, centre.y],
            [points[0].x, points[0].y],
            [points[1].x, points[1].y],
            [points[2].x, points[2].y],
            [points[3].x, points[3].y],
            [points[4].x, points[4].y],
            [points[5].x, points[5].y],
            [points[6].x, points[6].y],
            [points[7].x, points[7].y],
            [points[0].x, points[0].y],
        ]
    }
}