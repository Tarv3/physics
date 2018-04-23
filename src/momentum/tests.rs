#![cfg(test)]
use momentum::{linear::*, angluar::*, Moment};
use nalgebra::{Unit, Vector2};
#[test]
fn simple() {
    let moment = Moment::new(Vector2::new(1.0, 0.0), 1.0, 10.0, 500.0);
    let mut moment2 = Moment::new(Vector2::new(1.0, 0.0), 0.0, 10.0, 500.0);
    moment2.apply_force(Vector2::new(0.0, 1.0), Vector2::new(10.0, 0.0));
    assert_eq!(
        Moment::new(Vector2::new(1.0, 0.0), 0.020000001, 10.0, 500.0),
        moment2
    );
}
#[test]
fn collisions() {
    let moment = LinearMoment::new(Vector2::new(1.0, 1.0), 10.0);
    let moment2 = LinearMoment::new(Vector2::new(-1.0, 1.0), 10.0);
    let moment3 = LinearMoment::new(Vector2::new(1.0, -1.0), 10.0);

    assert_eq!(
        Vector2::new(-2.0, 0.0),
        moment.change(moment2, Unit::new_normalize(Vector2::new(-1.0, 0.0)))
    );
    assert_eq!(
        Vector2::new(0.0, 2.0),
        moment3.reflect(Unit::new_normalize(Vector2::new(0.0, 1.0)))
    );
}
