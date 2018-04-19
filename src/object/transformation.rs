use nalgebra::{Isometry, Isometry2, Point2, Translation, UnitComplex, Vector2};

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