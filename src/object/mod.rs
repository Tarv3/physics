use nalgebra::{Isometry, Isometry2, Point2, Translation, Unit, UnitComplex, Vector2,
               core::dimension::U2};
use ncollide::{query, shape, query::Contact, shape::Shape};
use momentum::{LinearMomentum, AMomentum};
use collision::*;

mod test;
pub mod moving;

pub struct Transformation {
    pub position: Point2<f32>,
    pub rotation: UnitComplex<f32>,
}

impl Transformation {
    pub fn new(position: Point2<f32>, rotation: f32) -> Self {
        let rotation = UnitComplex::new(rotation);
        Self { position, rotation }
    }
    pub fn get_iso(&self) -> Isometry2<f32> {
        Isometry::from_parts(
            Translation::from_vector(self.position.coords),
            self.rotation,
        )
    }
    pub fn rotate(&mut self, angle: f32) {
        self.rotation *= UnitComplex::new(angle);
    }
    pub fn translate(&mut self, vector: Vector2<f32>) {
        self.position += vector;
    }
    pub fn vector_to(&self, point: Point2<f32>) -> Vector2<f32> {
        point - self.position
    }
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
