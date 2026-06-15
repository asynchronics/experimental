//! Angle types and operations.
//!
//! This module provides strongly typed angle units (`Degrees` and `Radians`)
//! with safe conversions and arithmetic operations. It ensures that degrees
//! and radians are not accidentally mixed, while offering convenient methods
//! and operator overloads for addition, subtraction, negation, scalar
//! multiplication and division, and comparison. Dividing an angle by an angle
//! yields a dimensionless `f64` ratio.
//!
//! Both types also provide trigonometric functions (`sin`, `cos`, `tan`,
//! `sin_cos`), normalization to the common ranges (`normalize_360` /
//! `normalize_180` for degrees, `normalize_two_pi` / `normalize_pi` for
//! radians), tolerance-based comparison with `is_nearly_equal`, and `Display`
//! formatting with a unit suffix.
//!
//! It also provides an `AngleLiteral` trait for converting `f64` values to
//! `Degrees` and `Radians` angles.
//!
//! # Examples
//!
//! ```rust
//! use utilities::angles::*;
//! use utilities::universal_constants::*;
//!
//! let d = 90.0f64.deg();
//! let r = Radians::new(PI_OVER_TWO);
//!
//! let sum = d + r;                            // Degrees
//! let doubled = 2.0 * d;                      // Degrees
//! let value = doubled.value();                // f64
//! let rad: Radians = (d + doubled).into();    // Radians
//!
//! let (sin, cos) = d.sin_cos();               // (f64, f64)
//! let ratio = doubled / d;                    // f64, 2.0
//! let wrapped = (d * 5.0).normalize_360();    // Degrees, 90 deg
//! assert!(wrapped.is_nearly_equal(d, SMALL_NUMBER));
//! assert_eq!(format!("{}", wrapped), "90 deg");
//! ```

use crate::universal_constants::{FULL_ANGLE_DEG, FULL_ANGLE_RAD, HALF_ANGLE_DEG, HALF_ANGLE_RAD};
use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Type for angles in degrees.
#[repr(transparent)]
#[derive(Default, Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Degrees {
    value: f64,
}

/// Type for angles in radians.
#[repr(transparent)]
#[derive(Default, Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Radians {
    value: f64,
}

impl Degrees {
    /// Creates new `Degrees` angle.
    pub fn new(value: f64) -> Self {
        Self { value }
    }

    /// Returns the value of the angle.
    pub fn value(self) -> f64 {
        self.value
    }

    /// Converts the angle to radians.
    pub fn to_radians(self) -> Radians {
        Radians::new(self.value.to_radians())
    }

    /// Returns the absolute value of the angle.
    pub fn abs(self) -> Self {
        Self::new(self.value.abs())
    }

    /// Computes the sine of the angle.
    pub fn sin(self) -> f64 {
        self.value.to_radians().sin()
    }

    /// Computes the cosine of the angle.
    pub fn cos(self) -> f64 {
        self.value.to_radians().cos()
    }

    /// Computes the tangent of the angle.
    pub fn tan(self) -> f64 {
        self.value.to_radians().tan()
    }

    /// Simultaneously computes the sine and cosine of the angle.
    /// Returns `(sin(angle), cos(angle))`.
    pub fn sin_cos(self) -> (f64, f64) {
        self.value.to_radians().sin_cos()
    }

    /// Normalizes the angle to the range `[0, 360)` degrees.
    pub fn normalize_360(self) -> Self {
        Self::new(self.value.rem_euclid(FULL_ANGLE_DEG))
    }

    /// Normalizes the angle to the range `[-180, 180)` degrees.
    /// Note that an input of exactly 180 maps to -180.
    pub fn normalize_180(self) -> Self {
        Self::new((self.value + HALF_ANGLE_DEG).rem_euclid(FULL_ANGLE_DEG) - HALF_ANGLE_DEG)
    }

    /// Returns `true` if the two angles differ by less than `tolerance` degrees.
    pub fn is_nearly_equal(self, other: Self, tolerance: f64) -> bool {
        (self.value - other.value).abs() < tolerance
    }
}

impl Radians {
    /// Creates new `Radians` angle.
    pub fn new(value: f64) -> Self {
        Self { value }
    }

    /// Returns the value of the angle.
    pub fn value(self) -> f64 {
        self.value
    }

    /// Converts the angle to degrees.
    pub fn to_degrees(self) -> Degrees {
        Degrees::new(self.value.to_degrees())
    }

    /// Returns the absolute value of the angle.
    pub fn abs(self) -> Self {
        Self::new(self.value.abs())
    }

    /// Computes the sine of the angle.
    pub fn sin(self) -> f64 {
        self.value.sin()
    }

    /// Computes the cosine of the angle.
    pub fn cos(self) -> f64 {
        self.value.cos()
    }

    /// Computes the tangent of the angle.
    pub fn tan(self) -> f64 {
        self.value.tan()
    }

    /// Simultaneously computes the sine and cosine of the angle.
    /// Returns `(sin(angle), cos(angle))`.
    pub fn sin_cos(self) -> (f64, f64) {
        self.value.sin_cos()
    }

    /// Normalizes the angle to the range `[0, 2*PI)` radians.
    pub fn normalize_two_pi(self) -> Self {
        Self::new(self.value.rem_euclid(FULL_ANGLE_RAD))
    }

    /// Normalizes the angle to the range `[-PI, PI)` radians.
    /// Note that an input of exactly PI maps to -PI.
    pub fn normalize_pi(self) -> Self {
        Self::new((self.value + HALF_ANGLE_RAD).rem_euclid(FULL_ANGLE_RAD) - HALF_ANGLE_RAD)
    }

    /// Returns `true` if the two angles differ by less than `tolerance` radians.
    pub fn is_nearly_equal(self, other: Self, tolerance: f64) -> bool {
        (self.value - other.value).abs() < tolerance
    }
}

// Degrees + Degrees -> Degrees
impl Add for Degrees {
    type Output = Degrees;

    fn add(self, angle: Degrees) -> Degrees {
        Degrees::new(self.value + angle.value)
    }
}

// Degrees - Degrees -> Degrees
impl Sub for Degrees {
    type Output = Degrees;

    fn sub(self, angle: Degrees) -> Degrees {
        Degrees::new(self.value - angle.value)
    }
}

// Degrees + Radians -> Degrees
impl Add<Radians> for Degrees {
    type Output = Degrees;

    fn add(self, angle: Radians) -> Degrees {
        Degrees::new(self.value + angle.value.to_degrees())
    }
}

// Degrees - Radians -> Degrees
impl Sub<Radians> for Degrees {
    type Output = Degrees;

    fn sub(self, angle: Radians) -> Degrees {
        Degrees::new(self.value - angle.value.to_degrees())
    }
}

// Degrees += Degrees
impl AddAssign for Degrees {
    fn add_assign(&mut self, angle: Degrees) {
        *self = *self + angle;
    }
}

// Degrees -= Degrees
impl SubAssign for Degrees {
    fn sub_assign(&mut self, angle: Degrees) {
        *self = *self - angle;
    }
}

// Degrees += Radians
impl AddAssign<Radians> for Degrees {
    fn add_assign(&mut self, angle: Radians) {
        *self = *self + angle;
    }
}

// Degrees -= Radians
impl SubAssign<Radians> for Degrees {
    fn sub_assign(&mut self, angle: Radians) {
        *self = *self - angle;
    }
}

// Degrees * f64 -> Degrees
impl Mul<f64> for Degrees {
    type Output = Degrees;

    fn mul(self, scalar: f64) -> Degrees {
        Degrees::new(self.value * scalar)
    }
}

// f64 * Degrees -> Degrees
impl Mul<Degrees> for f64 {
    type Output = Degrees;

    fn mul(self, angle: Degrees) -> Degrees {
        angle * self
    }
}

// Degrees / f64 -> Degrees
impl Div<f64> for Degrees {
    type Output = Degrees;

    fn div(self, scalar: f64) -> Degrees {
        Degrees::new(self.value / scalar)
    }
}

// Degrees *= f64
impl MulAssign<f64> for Degrees {
    fn mul_assign(&mut self, scalar: f64) {
        *self = *self * scalar;
    }
}

// Degrees /= f64
impl DivAssign<f64> for Degrees {
    fn div_assign(&mut self, scalar: f64) {
        *self = *self / scalar;
    }
}

// Degrees / Degrees -> f64 (dimensionless ratio)
impl Div for Degrees {
    type Output = f64;

    fn div(self, angle: Degrees) -> f64 {
        self.value / angle.value
    }
}

// -Degrees -> Degrees
impl Neg for Degrees {
    type Output = Degrees;

    fn neg(self) -> Degrees {
        Degrees::new(-self.value)
    }
}

// Radians + Radians -> Radians
impl Add for Radians {
    type Output = Radians;

    fn add(self, angle: Radians) -> Radians {
        Radians::new(self.value + angle.value)
    }
}

// Radians - Radians -> Radians
impl Sub for Radians {
    type Output = Radians;

    fn sub(self, angle: Radians) -> Radians {
        Radians::new(self.value - angle.value)
    }
}

// Radians + Degrees -> Radians
impl Add<Degrees> for Radians {
    type Output = Radians;

    fn add(self, angle: Degrees) -> Radians {
        Radians::new(self.value + angle.value.to_radians())
    }
}

// Radians - Degrees -> Radians
impl Sub<Degrees> for Radians {
    type Output = Radians;

    fn sub(self, angle: Degrees) -> Radians {
        Radians::new(self.value - angle.value.to_radians())
    }
}

// Radians += Radians
impl AddAssign for Radians {
    fn add_assign(&mut self, angle: Radians) {
        *self = *self + angle;
    }
}

// Radians -= Radians
impl SubAssign for Radians {
    fn sub_assign(&mut self, angle: Radians) {
        *self = *self - angle;
    }
}

// Radians += Degrees
impl AddAssign<Degrees> for Radians {
    fn add_assign(&mut self, angle: Degrees) {
        *self = *self + angle;
    }
}

// Radians -= Degrees
impl SubAssign<Degrees> for Radians {
    fn sub_assign(&mut self, angle: Degrees) {
        *self = *self - angle;
    }
}

// Radians * f64 -> Radians
impl Mul<f64> for Radians {
    type Output = Radians;

    fn mul(self, scalar: f64) -> Radians {
        Radians::new(self.value * scalar)
    }
}

// f64 * Radians -> Radians
impl Mul<Radians> for f64 {
    type Output = Radians;

    fn mul(self, angle: Radians) -> Radians {
        angle * self
    }
}

// Radians / f64 -> Radians
impl Div<f64> for Radians {
    type Output = Radians;

    fn div(self, scalar: f64) -> Radians {
        Radians::new(self.value / scalar)
    }
}

// Radians *= f64
impl MulAssign<f64> for Radians {
    fn mul_assign(&mut self, scalar: f64) {
        *self = *self * scalar;
    }
}

// Radians /= f64
impl DivAssign<f64> for Radians {
    fn div_assign(&mut self, scalar: f64) {
        *self = *self / scalar;
    }
}

// Radians / Radians -> f64 (dimensionless ratio)
impl Div for Radians {
    type Output = f64;

    fn div(self, angle: Radians) -> f64 {
        self.value / angle.value
    }
}

// -Radians -> Radians
impl Neg for Radians {
    type Output = Radians;

    fn neg(self) -> Radians {
        Radians::new(-self.value)
    }
}

impl fmt::Display for Degrees {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.value.fmt(f)?;
        write!(f, " deg")
    }
}

impl fmt::Display for Radians {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.value.fmt(f)?;
        write!(f, " rad")
    }
}

impl From<Radians> for Degrees {
    fn from(r: Radians) -> Self {
        r.to_degrees()
    }
}

impl From<Degrees> for Radians {
    fn from(d: Degrees) -> Self {
        d.to_radians()
    }
}

/// Extension trait for creating angles directly from `f64` literals,
/// e.g. `90.0.deg()` or `PI.rad()`.
pub trait AngleLiteral {
    /// Converts the literal to `Degrees` angle.
    fn deg(self) -> Degrees;
    /// Converts the literal to `Radians` angle.
    fn rad(self) -> Radians;
}

impl AngleLiteral for f64 {
    fn deg(self) -> Degrees {
        Degrees::new(self)
    }

    fn rad(self) -> Radians {
        Radians::new(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::universal_constants::*;

    /// Test angle addition.
    #[test]
    pub fn angle_addition_test() {
        // Degrees + Degrees
        let d1 = Degrees::new(10.0);
        let d2 = Degrees::new(20.0);
        let d3 = d1 + d2;
        assert_eq!(d3.value, 30.0);

        // Radians + Radians
        let r1 = Radians::new(PI);
        let r2 = Radians::new(PI);
        let r3 = r1 + r2;
        assert_eq!(r3.value, TWO_PI);

        // Degrees + Radians
        let d4 = Degrees::new(180.0);
        let r4 = Radians::new(PI);
        let d5 = d4 + r4;
        assert!(d5.is_nearly_equal(Degrees::new(FULL_ANGLE_DEG), SMALL_NUMBER));

        // Radians + Degrees
        let r5 = Radians::new(PI);
        let d6 = Degrees::new(180.0);
        let r6 = r5 + d6;
        assert!(r6.is_nearly_equal(Radians::new(TWO_PI), SMALL_NUMBER));
    }

    /// Test angle subtraction.
    #[test]
    pub fn angle_subtraction_test() {
        // Degrees - Degrees
        let d1 = Degrees::new(20.0);
        let d2 = Degrees::new(10.0);
        let d3 = d1 - d2;
        assert_eq!(d3.value, 10.0);

        // Radians - Radians
        let r1 = Radians::new(TWO_PI);
        let r2 = Radians::new(PI);
        let r3 = r1 - r2;
        assert_eq!(r3.value, PI);

        // Degrees - Radians
        let d4 = Degrees::new(180.0);
        let r4 = Radians::new(PI);
        let d5 = d4 - r4;
        assert!(d5.is_nearly_equal(Degrees::new(0.0), SMALL_NUMBER));

        // Radians - Degrees
        let r5 = Radians::new(TWO_PI);
        let d6 = Degrees::new(180.0);
        let r6 = r5 - d6;
        assert!(r6.is_nearly_equal(Radians::new(PI), SMALL_NUMBER));
    }

    /// Test angle literals
    #[test]
    pub fn angle_literals_test() {
        let d1 = 180.0.deg();
        let r1 = PI.rad();
        let d2 = d1 + r1;

        assert_eq!(d1.value, 180.0);
        assert_eq!(r1.value, PI);
        assert!(d2.is_nearly_equal(Degrees::new(FULL_ANGLE_DEG), SMALL_NUMBER));
    }

    /// Test assignment.
    #[test]
    pub fn angle_assignment_test() {
        let mut d1 = Degrees::new(10.0);
        assert_eq!(d1.value, 10.0);

        let r1 = Radians::new(PI);
        d1 = r1.into();
        assert!(d1.is_nearly_equal(Degrees::new(HALF_ANGLE_DEG), SMALL_NUMBER));

        let r2: Radians = (d1 * 2.0).into();
        assert!(r2.is_nearly_equal(Radians::new(TWO_PI), SMALL_NUMBER));
    }

    /// Test multiplication.
    #[test]
    pub fn angle_multiplication_test() {
        let d1 = Degrees::new(10.0);
        let d2 = d1 * 2.0;
        assert_eq!(d2.value, 20.0);

        let r1 = Radians::new(PI);
        let r2 = r1 * 2.0;
        assert_eq!(r2.value, TWO_PI);

        // f64 * angle
        let d3 = 2.0 * d1;
        assert_eq!(d3.value, 20.0);

        let r3 = 2.0 * r1;
        assert_eq!(r3.value, TWO_PI);
    }

    /// Test nearly-equal comparison.
    #[test]
    pub fn angle_is_nearly_equal_test() {
        // Within and outside tolerance.
        let d1 = Degrees::new(10.0);
        assert!(d1.is_nearly_equal(Degrees::new(10.0), SMALL_NUMBER));
        assert!(d1.is_nearly_equal(Degrees::new(10.0 + 1e-9), SMALL_NUMBER));
        assert!(!d1.is_nearly_equal(Degrees::new(10.1), SMALL_NUMBER));
        assert!(d1.is_nearly_equal(Degrees::new(10.1), 0.2));

        let r1 = Radians::new(PI);
        assert!(r1.is_nearly_equal(Radians::new(PI), SMALL_NUMBER));
        assert!(r1.is_nearly_equal(Radians::new(PI + 1e-9), SMALL_NUMBER));
        assert!(!r1.is_nearly_equal(Radians::new(PI + 0.1), SMALL_NUMBER));

        // A deg -> rad -> deg round trip is not exact, but is nearly equal.
        let d2 = Degrees::new(3.0);
        let d3 = d2.to_radians().to_degrees();
        assert_ne!(d2, d3);
        assert!(d2.is_nearly_equal(d3, SMALL_NUMBER));
    }

    /// Test display formatting.
    #[test]
    pub fn angle_display_test() {
        assert_eq!(format!("{}", Degrees::new(90.0)), "90 deg");
        assert_eq!(format!("{:.2}", Degrees::new(90.0)), "90.00 deg");
        assert_eq!(format!("{}", Radians::new(1.5)), "1.5 rad");
        assert_eq!(format!("{:.3}", Radians::new(PI)), "3.142 rad");
    }

    /// Test comparison operators.
    #[test]
    pub fn angle_comparison_test() {
        assert!(Degrees::new(10.0) < Degrees::new(20.0));
        assert!(Degrees::new(20.0) >= Degrees::new(20.0));
        assert!(Degrees::new(-10.0) > Degrees::new(-20.0));

        assert!(Radians::new(PI_OVER_TWO) < Radians::new(PI));
        assert!(Radians::new(PI) <= Radians::new(PI));
        assert!(Radians::new(-PI) < Radians::new(PI_OVER_TWO));
    }

    /// Test assignment operators.
    #[test]
    pub fn angle_assign_ops_test() {
        // Degrees += / -= Degrees
        let mut d1 = Degrees::new(10.0);
        d1 += Degrees::new(20.0);
        assert_eq!(d1.value, 30.0);
        d1 -= Degrees::new(5.0);
        assert_eq!(d1.value, 25.0);

        // Degrees += / -= Radians
        let mut d2 = Degrees::new(90.0);
        d2 += Radians::new(PI);
        assert!(d2.is_nearly_equal(Degrees::new(270.0), SMALL_NUMBER));
        d2 -= Radians::new(PI);
        assert!(d2.is_nearly_equal(Degrees::new(90.0), SMALL_NUMBER));

        // Radians += / -= Radians
        let mut r1 = Radians::new(PI);
        r1 += Radians::new(PI);
        assert_eq!(r1.value, TWO_PI);
        r1 -= Radians::new(PI);
        assert_eq!(r1.value, PI);

        // Radians += / -= Degrees
        let mut r2 = Radians::new(PI);
        r2 += Degrees::new(180.0);
        assert!(r2.is_nearly_equal(Radians::new(TWO_PI), SMALL_NUMBER));
        r2 -= Degrees::new(180.0);
        assert!(r2.is_nearly_equal(Radians::new(PI), SMALL_NUMBER));

        // Degrees *= / /= f64
        let mut d3 = Degrees::new(10.0);
        d3 *= 2.0;
        assert_eq!(d3.value, 20.0);
        d3 /= 4.0;
        assert_eq!(d3.value, 5.0);

        // Radians *= / /= f64
        let mut r3 = Radians::new(PI);
        r3 *= 2.0;
        assert_eq!(r3.value, TWO_PI);
        r3 /= 2.0;
        assert_eq!(r3.value, PI);
    }

    /// Test negation.
    #[test]
    pub fn angle_negation_test() {
        let d1 = Degrees::new(10.0);
        assert_eq!((-d1).value, -10.0);
        assert_eq!((-(-d1)).value, 10.0);

        let r1 = Radians::new(PI);
        assert_eq!((-r1).value, -PI);
        assert_eq!((-(-r1)).value, PI);
    }

    /// Test angle normalization.
    #[test]
    pub fn angle_normalization_test() {
        // Degrees, [0, 360) range.
        assert_eq!(Degrees::new(10.0).normalize_360().value, 10.0);
        assert_eq!(Degrees::new(370.0).normalize_360().value, 10.0);
        assert_eq!(Degrees::new(730.0).normalize_360().value, 10.0);
        assert_eq!(Degrees::new(-90.0).normalize_360().value, 270.0);
        assert_eq!(Degrees::new(360.0).normalize_360().value, 0.0);
        assert_eq!(Degrees::new(-720.0).normalize_360().value, 0.0);

        // Degrees, [-180, 180) range.
        assert_eq!(Degrees::new(10.0).normalize_180().value, 10.0);
        assert_eq!(Degrees::new(190.0).normalize_180().value, -170.0);
        assert_eq!(Degrees::new(-190.0).normalize_180().value, 170.0);
        assert_eq!(Degrees::new(180.0).normalize_180().value, -180.0);
        assert_eq!(Degrees::new(-180.0).normalize_180().value, -180.0);
        assert_eq!(Degrees::new(540.0).normalize_180().value, -180.0);

        // Radians, [0, 2*PI) range.
        assert_eq!(Radians::new(1.0).normalize_two_pi().value, 1.0);
        assert_eq!(Radians::new(TWO_PI).normalize_two_pi().value, 0.0);
        assert!(
            Radians::new(TWO_PI + 1.0)
                .normalize_two_pi()
                .is_nearly_equal(Radians::new(1.0), SMALL_NUMBER)
        );
        assert!(
            Radians::new(-PI_OVER_TWO)
                .normalize_two_pi()
                .is_nearly_equal(Radians::new(3.0 * PI_OVER_TWO), SMALL_NUMBER)
        );

        // Radians, [-PI, PI) range.
        assert_eq!(Radians::new(1.0).normalize_pi().value, 1.0);
        assert_eq!(Radians::new(PI).normalize_pi().value, -PI);
        assert_eq!(Radians::new(-PI).normalize_pi().value, -PI);
        assert!(
            Radians::new(3.0 * PI_OVER_TWO)
                .normalize_pi()
                .is_nearly_equal(Radians::new(-PI_OVER_TWO), SMALL_NUMBER)
        );
        assert!(
            Radians::new(-3.0 * PI_OVER_TWO)
                .normalize_pi()
                .is_nearly_equal(Radians::new(PI_OVER_TWO), SMALL_NUMBER)
        );
    }

    /// Test trigonometric functions.
    #[test]
    pub fn angle_trigonometry_test() {
        // Degrees
        let d1 = Degrees::new(30.0);
        assert!((d1.sin() - 0.5).abs() < SMALL_NUMBER);

        let d2 = Degrees::new(60.0);
        assert!((d2.cos() - 0.5).abs() < SMALL_NUMBER);

        let d3 = Degrees::new(45.0);
        assert!((d3.tan() - 1.0).abs() < SMALL_NUMBER);

        let (sin, cos) = Degrees::new(90.0).sin_cos();
        assert!((sin - 1.0).abs() < SMALL_NUMBER);
        assert!(cos.abs() < SMALL_NUMBER);

        // Radians
        let r1 = Radians::new(PI / 6.0);
        assert!((r1.sin() - 0.5).abs() < SMALL_NUMBER);

        let r2 = Radians::new(PI / 3.0);
        assert!((r2.cos() - 0.5).abs() < SMALL_NUMBER);

        let r3 = Radians::new(PI_OVER_FOUR);
        assert!((r3.tan() - 1.0).abs() < SMALL_NUMBER);

        let (sin, cos) = Radians::new(PI_OVER_TWO).sin_cos();
        assert!((sin - 1.0).abs() < SMALL_NUMBER);
        assert!(cos.abs() < SMALL_NUMBER);

        // Degrees and Radians give the same result for the same angle.
        assert_eq!(
            Degrees::new(30.0).sin(),
            Radians::new(30.0f64.to_radians()).sin()
        );
    }

    /// Test division.
    #[test]
    pub fn angle_division_test() {
        let d1 = Degrees::new(10.0);
        let d2 = d1 / 2.0;
        assert_eq!(d2.value, 5.0);

        let r1 = Radians::new(PI);
        let r2 = r1 / 2.0;
        assert_eq!(r2.value, PI_OVER_TWO);

        // angle / angle -> dimensionless ratio
        let ratio = Degrees::new(720.0) / Degrees::new(360.0);
        assert_eq!(ratio, 2.0);

        let ratio = Radians::new(TWO_PI) / Radians::new(PI);
        assert_eq!(ratio, 2.0);
    }
}
