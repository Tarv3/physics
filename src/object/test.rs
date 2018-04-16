#![cfg(test)]
use ncollide::{query, query::Contact, shape::{Ball, Cuboid, Plane}};
use object::*;
use nalgebra::{Vector2, core::Unit};
#[test]
fn object() {
    let object = Object::new(
        Cuboid::new(Vector2::new(1.0, 1.0)),
        Point2::new(0.0, 0.0),
        0.0,
    );
    let object2 = Object::new(Ball::new(1.0), Point2::new(2.1, 0.0), 0.0);
    let object3 = Object::new(
        Plane::new(Unit::new_normalize(Vector2::new(0.0, 1.0))),
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
