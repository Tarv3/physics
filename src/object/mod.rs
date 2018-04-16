use nalgebra::{Isometry, Isometry2, Point2, Translation, UnitComplex, Vector2,
               core::dimension::U2};
use ncollide::{query, query::Contact, shape::Shape};

pub struct Transformation {
    pub position: Point2<f32>,
    pub rotation: UnitComplex<f32>,
}

impl Transformation {
    pub fn new(position: Point2<f32>, rotation: f32) -> Self {
        let rotation = UnitComplex::new(rotation);
        Self { position, rotation }
    }
    pub fn get_iso(&self) -> Isometry<f32, U2, UnitComplex<f32>> {
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

pub struct Object {
    shape: Box<Shape<Point2<f32>, Isometry2<f32>>>,
    transformation: Transformation,
}

impl Object {
    pub fn new(
        shape: Box<Shape<Point2<f32>, Isometry2<f32>>>,
        position: Point2<f32>,
        rotation: f32,
    ) -> Self
where {
        Self {
            shape,
            transformation: Transformation::new(position, rotation),
        }
    }
    pub fn contact_with(&self, other: &Self, prediction: f32) -> Option<Contact<Point2<f32>>> {
        query::contact(
            &self.transformation.get_iso(),
            self.shape.as_ref(),
            &other.transformation.get_iso(),
            other.shape.as_ref(),
            prediction,
        )
    }
}

#[cfg(test)]
mod tests {
    use ncollide::{query, query::Contact, shape::{Cuboid, Ball, Plane}};
    use object::*;
    use nalgebra::{Vector2, core::Unit};
    #[test]
    fn simple() {
        let object = Object::new(Box::new(Cuboid::new(
            Vector2::new(1.0, 1.0))),
            Point2::new(0.0, 0.0),
            0.0,
        );
        let object2 = Object::new(Box::new(Ball::new(
            1.0)),
            Point2::new(2.1, 0.0),
            0.0,
        );
        let object3 = Object::new(Box::new(Plane::new(
            Unit::new_normalize(Vector2::new(0.0, 1.0)))),
            Point2::new(0.0, -1.0),
            0.0,
        );
        let contact1 = object.contact_with(&object2, 1.0);
        let contact2 = object.contact_with(&object3, 1.0);
        println!("{:?}", contact1);
        println!("{:?}", contact2);
        assert!(contact1.unwrap().depth < 0.0);
        assert!(contact2.unwrap().depth == 0.0);
    }

}
