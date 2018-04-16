use nalgebra::{Isometry, Isometry2, Point2, Translation, Unit, UnitComplex, Vector2,
               core::dimension::U2};
use ncollide::{query, query::Contact, shape::Shape, shape};
use momentum::{LinearMomentum, Momentum};

mod test;
pub mod moving;
// called by a collision with an object that has some momentum



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

impl Momentum for Object<shape::Cuboid<Vector2<f32>>> {
    fn moi(&self, mass: f32) -> f32 {
        let dimensions = self.shape.half_extents();
        0.5 * mass * (dimensions.x * dimensions.x + dimensions.y * dimensions.y)
    }
}
impl Momentum for Object<shape::Ball<f32>> {
    fn moi(&self, mass: f32) -> f32 {
        0.5 * mass * self.shape.radius() * self.shape.radius()
    }
}
