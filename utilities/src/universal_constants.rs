//! Common mathematical, unit conversion, and physical constants.
//!
//! This module provides shared constants grouped around simulation-oriented
//! work, including mathematical constants, angle values, angle and length
//! conversion factors, time constants, SI reference values, and
//! floating-point tolerances.
//!
//! # Examples
//!
//! ```rust
//! use utilities::universal_constants::*;
//!
//! assert_eq!(FULL_ANGLE_DEG, 360.0);
//! assert_eq!(HALF_ANGLE_RAD, PI);
//! assert_eq!(M_TO_CM, 100.0);
//! assert!(KINDA_SMALL_NUMBER > SMALL_NUMBER);
//! ```

/// Re-export of `std::f64::consts::PI`.
pub use std::f64::consts::PI;

/// Mathematical constants.
/// Two times PI.
pub const TWO_PI: f64 = 2.0 * PI;
/// PI divided by two.
pub const PI_OVER_TWO: f64 = PI / 2.0;
/// PI divided by four.
pub const PI_OVER_FOUR: f64 = PI / 4.0;

/// Angle constants.
/// Full angle in degrees, 360 deg.
pub const FULL_ANGLE_DEG: f64 = 360.0;
/// Half angle in degrees, 180 deg.
pub const HALF_ANGLE_DEG: f64 = 180.0;
/// Quarter angle in degrees, 90 deg.
pub const QUARTER_ANGLE_DEG: f64 = 90.0;
/// Full angle in radians, 2 * PI.
pub const FULL_ANGLE_RAD: f64 = TWO_PI;
/// Half angle in radians, PI.
pub const HALF_ANGLE_RAD: f64 = PI;
/// Quarter angle in radians, PI / 2.
pub const QUARTER_ANGLE_RAD: f64 = PI_OVER_TWO;

/// Angle conversion factors.
/// Degrees to radians conversion factor.
pub const DEG_TO_RAD: f64 = PI / 180.0;
/// Radians to degrees conversion factor.
pub const RAD_TO_DEG: f64 = 180.0 / PI;

/// Length conversion factors.
/// Kilometers to meters conversion factor.
pub const KM_TO_M: f64 = 1_000.0;
/// Meters to kilometers conversion factor.
pub const M_TO_KM: f64 = 0.001;
/// Meters to centimeters conversion factor.
pub const M_TO_CM: f64 = 100.0;
/// Centimeters to meters conversion factor.
pub const CM_TO_M: f64 = 0.01;
/// Meters to millimeters conversion factor.
pub const M_TO_MM: f64 = 1_000.0;
/// Millimeters to meters conversion factor.
pub const MM_TO_M: f64 = 0.001;

/// Time constants.
/// Number of seconds in one minute.
pub const SECONDS_PER_MINUTE: f64 = 60.0;
/// Number of seconds in one hour.
pub const SECONDS_PER_HOUR: f64 = 3_600.0;
/// Number of milliseconds in one second.
pub const MILLIS_PER_SECOND: f64 = 1_000.0;
/// Number of microseconds in one second.
pub const MICROS_PER_SECOND: f64 = 1_000_000.0;
/// Number of nanoseconds in one second.
pub const NANOS_PER_SECOND: f64 = 1_000_000_000.0;

/// SI reference constants.
/// Standard gravitational acceleration in meters per second squared.
pub const STANDARD_GRAVITY: f64 = 9.80665;
/// Standard atmospheric pressure in pascals.
pub const STANDARD_ATMOSPHERE_PA: f64 = 101_325.0;
/// Absolute temperature of zero degrees Celsius in kelvin.
pub const ZERO_CELSIUS_KELVIN: f64 = 273.15;

/// Floating-point tolerances.
/// Small floating-point tolerance value.
pub const SMALL_NUMBER: f64 = 1e-8;
/// Larger floating-point tolerance value for approximate comparisons.
pub const KINDA_SMALL_NUMBER: f64 = 1e-4;
