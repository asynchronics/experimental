//! PID controller building blocks.
//!
//! `PController` is a standalone proportional controller (e.g. an outer
//! loop in a cascade); `PidController` is the full PID. In both, the error
//! (setpoint minus measurement) is expected to be computed by the caller,
//! in the same spirit as a Simulink sum block feeding a PID block.
//!
//! # Examples
//!
//! ```rust
//! use utilities::pid::{PController, PidController};
//!
//! let mut controller = PController::new(2.0);
//! let output = controller.update(1.5);
//! assert_eq!(output, 3.0);
//!
//! // Clamp the output to a normalized range, e.g. for a normalized actuator command.
//! let mut clamped = PController::with_limits(2.0, Some(-1.0), Some(1.0));
//! assert_eq!(clamped.update(1.5), 1.0);
//!
//! let mut pid = PidController::new(2.0, 0.0, 0.0);
//! let output = pid.update(1.5, 0.1); // error, dt
//! assert_eq!(output, 3.0);
//! ```

use crate::lowpass::LowPassFilter;
use serde::{Deserialize, Serialize};

/// Proportional controller.
#[derive(Debug, Serialize, Deserialize)]
pub struct PController {
    /// Proportional gain.
    gain: f64,
    /// Lower bound on the controller output, if any.
    min: Option<f64>,
    /// Upper bound on the controller output, if any.
    max: Option<f64>,
}

impl PController {
    /// Create a proportional controller from a proportional gain, with an
    /// unbounded output.
    #[inline]
    pub fn new(gain: f64) -> Self {
        Self {
            gain,
            min: None,
            max: None,
        }
    }

    /// Create a proportional controller from a proportional gain, clamping
    /// the output to `[min, max]`. Either bound can be `None` to leave that
    /// side unbounded, e.g. `(Some(-1.0), Some(1.0))` or `(Some(0.0), None)`.
    #[inline]
    pub fn with_limits(gain: f64, min: Option<f64>, max: Option<f64>) -> Self {
        if let (Some(min), Some(max)) = (min, max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        Self { gain, min, max }
    }

    /// Compute the controller output for a given error.
    #[inline]
    pub fn update(&self, error: f64) -> f64 {
        let mut output = self.gain * error;
        if let Some(min) = self.min {
            output = output.max(min);
        }
        if let Some(max) = self.max {
            output = output.min(max);
        }
        output
    }

    /// Get the proportional gain.
    #[inline]
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Set the proportional gain.
    #[inline]
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain;
    }

    /// Get the lower bound on the controller output, if any.
    #[inline]
    pub fn min(&self) -> Option<f64> {
        self.min
    }

    /// Get the upper bound on the controller output, if any.
    #[inline]
    pub fn max(&self) -> Option<f64> {
        self.max
    }

    /// Set the lower bound on the controller output.
    /// Pass `None` to leave that side unbounded.
    #[inline]
    pub fn set_min(&mut self, min: Option<f64>) {
        if let (Some(min), Some(max)) = (min, self.max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        self.min = min;
    }

    /// Set the upper bound on the controller output.
    /// Pass `None` to leave that side unbounded.
    #[inline]
    pub fn set_max(&mut self, max: Option<f64>) {
        if let (Some(min), Some(max)) = (self.min, max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        self.max = max;
    }

    /// Clear both output bounds, leaving the output unbounded.
    #[inline]
    pub fn clear_limits(&mut self) {
        self.min = None;
        self.max = None;
    }
}

/// Full PID controller with optional output limits and derivative filter.
/// `master_gain` scales the combined P+I+D output. The integral term can be
/// independently clamped to `[-integral_limit, integral_limit]` via
/// [`Self::set_integral_limit`] as a basic anti-windup safeguard.
#[derive(Debug, Serialize, Deserialize)]
pub struct PidController {
    /// Proportional gain.
    proportional_gain: f64,
    /// Integral gain.
    integral_gain: f64,
    /// Derivative gain.
    derivative_gain: f64,
    /// Overall gain multiplying the combined P+I+D output.
    master_gain: f64,
    /// Accumulated integral of the error.
    integral: f64,
    /// Symmetric bound on the accumulated integral, if any.
    integral_limit: Option<f64>,
    /// Error from the previous `update` call, used for the derivative term.
    previous_error: f64,
    /// Lowpass filter applied to the raw derivative term, if any, to reduce
    /// noise amplification.
    derivative_filter: Option<LowPassFilter>,
    /// Lower bound on the controller output, if any.
    min: Option<f64>,
    /// Upper bound on the controller output, if any.
    max: Option<f64>,
}

impl PidController {
    /// Create a PID controller from proportional, integral, and derivative
    /// gains, with an unbounded output.
    #[inline]
    pub fn new(proportional_gain: f64, integral_gain: f64, derivative_gain: f64) -> Self {
        Self {
            proportional_gain,
            integral_gain,
            derivative_gain,
            master_gain: 1.0,
            integral: 0.0,
            integral_limit: None,
            previous_error: 0.0,
            derivative_filter: None,
            min: None,
            max: None,
        }
    }

    /// Create a PID controller from proportional, integral, and derivative
    /// gains, clamping the output to `[min, max]`. Either bound can be
    /// `None` to leave that side unbounded.
    #[inline]
    pub fn with_limits(
        proportional_gain: f64,
        integral_gain: f64,
        derivative_gain: f64,
        min: Option<f64>,
        max: Option<f64>,
    ) -> Self {
        if let (Some(min), Some(max)) = (min, max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        Self {
            proportional_gain,
            integral_gain,
            derivative_gain,
            master_gain: 1.0,
            integral: 0.0,
            integral_limit: None,
            previous_error: 0.0,
            derivative_filter: None,
            min,
            max,
        }
    }

    /// Compute the controller output for a given error and time step `dt`
    /// (in seconds), updating the integral and derivative state. `dt` must
    /// be strictly positive.
    #[inline]
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        assert!(dt > 0.0, "PID delta time must be positive");

        // Update the integral and derivative state.
        let mut derivative = (error - self.previous_error) / dt;
        self.integral += error * dt;
        self.previous_error = error;

        // Clamp the integral to [-integral_limit, integral_limit], if
        // configured, as a basic anti-windup safeguard, independent of the
        // output bounds.
        if let Some(limit) = self.integral_limit {
            self.integral = self.integral.clamp(-limit, limit);
        }

        // Filter the derivative, if a filter is configured, to reduce
        // noise amplification.
        if let Some(filter) = self.derivative_filter.as_mut() {
            derivative = filter.update(derivative);
        }

        // Compute the PID terms.
        let proportional_term = self.proportional_gain * error;
        let integral_term = self.integral_gain * self.integral;
        let derivative_term = self.derivative_gain * derivative;

        // Compute the output.
        let mut output = self.master_gain * (proportional_term + integral_term + derivative_term);

        // Clamp the output to the configured bounds, if any.
        if let Some(min) = self.min {
            output = output.max(min);
        }
        if let Some(max) = self.max {
            output = output.min(max);
        }
        output
    }

    /// Reset the accumulated integral and derivative state, including the
    /// derivative filter's state, if any.
    #[inline]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        if let Some(filter) = &mut self.derivative_filter {
            filter.reset();
        }
    }

    /// Set the lowpass filter applied to the raw derivative term, to reduce
    /// noise amplification. Pass `None` to disable filtering.
    #[inline]
    pub fn set_derivative_filter(&mut self, filter: Option<LowPassFilter>) {
        self.derivative_filter = filter;
    }

    /// Get the lowpass filter applied to the derivative term, if any.
    #[inline]
    pub fn derivative_filter(&self) -> Option<&LowPassFilter> {
        self.derivative_filter.as_ref()
    }

    /// Get the proportional gain.
    #[inline]
    pub fn proportional_gain(&self) -> f64 {
        self.proportional_gain
    }

    /// Get the integral gain.
    #[inline]
    pub fn integral_gain(&self) -> f64 {
        self.integral_gain
    }

    /// Get the derivative gain.
    #[inline]
    pub fn derivative_gain(&self) -> f64 {
        self.derivative_gain
    }

    /// Set the proportional gain.
    #[inline]
    pub fn set_proportional_gain(&mut self, proportional_gain: f64) {
        self.proportional_gain = proportional_gain;
    }

    /// Set the integral gain.
    #[inline]
    pub fn set_integral_gain(&mut self, integral_gain: f64) {
        self.integral_gain = integral_gain;
    }

    /// Set the derivative gain.
    #[inline]
    pub fn set_derivative_gain(&mut self, derivative_gain: f64) {
        self.derivative_gain = derivative_gain;
    }

    /// Get the master gain.
    #[inline]
    pub fn master_gain(&self) -> f64 {
        self.master_gain
    }

    /// Set the master gain, an overall multiplier on the combined P+I+D
    /// output, applied before the output is clamped to its limits.
    #[inline]
    pub fn set_master_gain(&mut self, master_gain: f64) {
        self.master_gain = master_gain;
    }

    /// Get the accumulated integral of the error.
    #[inline]
    pub fn integral(&self) -> f64 {
        self.integral
    }

    /// Get the symmetric bound on the accumulated integral, if any.
    #[inline]
    pub fn integral_limit(&self) -> Option<f64> {
        self.integral_limit
    }

    /// Set the symmetric bound on the accumulated integral, clamping it to
    /// `[-limit, limit]` as a basic anti-windup safeguard. Pass `None` to
    /// leave the integral unbounded. `limit` must be non-negative.
    #[inline]
    pub fn set_integral_limit(&mut self, limit: Option<f64>) {
        if let Some(limit) = limit {
            debug_assert!(limit >= 0.0, "integral limit must be non-negative");
        }
        self.integral_limit = limit;
    }

    /// Get the lower bound on the controller output, if any.
    #[inline]
    pub fn min(&self) -> Option<f64> {
        self.min
    }

    /// Get the upper bound on the controller output, if any.
    #[inline]
    pub fn max(&self) -> Option<f64> {
        self.max
    }

    /// Set the lower bound on the controller output. Pass `None` to leave
    /// that side unbounded.
    #[inline]
    pub fn set_min(&mut self, min: Option<f64>) {
        if let (Some(min), Some(max)) = (min, self.max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        self.min = min;
    }

    /// Set the upper bound on the controller output. Pass `None` to leave
    /// that side unbounded.
    #[inline]
    pub fn set_max(&mut self, max: Option<f64>) {
        if let (Some(min), Some(max)) = (self.min, max) {
            debug_assert!(min <= max, "min must be less than or equal to max");
        }
        self.max = max;
    }

    /// Clear both output bounds, leaving the output unbounded.
    #[inline]
    pub fn clear_limits(&mut self) {
        self.min = None;
        self.max = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::universal_constants::*;

    #[test]
    fn zero_error_gives_zero_output() {
        let controller = PController::new(5.0);
        assert_eq!(controller.update(0.0), 0.0);
    }

    #[test]
    fn output_scales_linearly_with_error() {
        let controller = PController::new(2.0);
        assert_eq!(controller.update(1.0), 2.0);
        assert_eq!(controller.update(3.0), 6.0);
    }

    #[test]
    fn negative_error_gives_negative_output() {
        let controller = PController::new(2.0);
        assert_eq!(controller.update(-1.5), -3.0);
    }

    #[test]
    fn zero_gain_gives_zero_output() {
        let controller = PController::new(0.0);
        assert_eq!(controller.update(10.0), 0.0);
    }

    #[test]
    fn gain_returns_configured_gain() {
        let controller = PController::new(3.5);
        assert!((controller.gain() - 3.5).abs() < KINDA_SMALL_NUMBER);
    }

    #[test]
    fn unlimited_controller_does_not_clamp() {
        let controller = PController::new(1e6);
        assert_eq!(controller.update(1e6), 1e12);
        assert_eq!(controller.update(-1e6), -1e12);
    }

    #[test]
    fn output_clamps_to_symmetric_limits() {
        let controller = PController::with_limits(2.0, Some(-1.0), Some(1.0));
        assert_eq!(controller.update(1.5), 1.0);
        assert_eq!(controller.update(-1.5), -1.0);
        assert_eq!(controller.update(0.25), 0.5);
    }

    #[test]
    fn output_clamps_to_asymmetric_limits() {
        let controller = PController::with_limits(2.0, Some(0.0), Some(1.0));
        assert_eq!(controller.update(-1.0), 0.0);
        assert_eq!(controller.update(2.0), 1.0);
        assert_eq!(controller.update(0.25), 0.5);
    }

    #[test]
    fn output_clamps_only_lower_bound_when_max_is_none() {
        let controller = PController::with_limits(2.0, Some(0.0), None);
        assert_eq!(controller.update(-1.0), 0.0);
        assert_eq!(controller.update(10.0), 20.0);
    }

    #[test]
    fn output_clamps_only_upper_bound_when_min_is_none() {
        let controller = PController::with_limits(2.0, None, Some(1.0));
        assert_eq!(controller.update(10.0), 1.0);
        assert_eq!(controller.update(-10.0), -20.0);
    }

    #[test]
    fn with_limits_exposes_bounds() {
        let controller = PController::with_limits(2.0, Some(-1.0), Some(1.0));
        assert_eq!(controller.min(), Some(-1.0));
        assert_eq!(controller.max(), Some(1.0));
    }

    #[test]
    fn new_controller_has_no_bounds() {
        let controller = PController::new(2.0);
        assert_eq!(controller.min(), None);
        assert_eq!(controller.max(), None);
    }

    #[test]
    fn set_min_and_set_max_update_bounds() {
        let mut controller = PController::new(2.0);
        controller.set_min(Some(0.0));
        controller.set_max(Some(1.0));
        assert_eq!(controller.min(), Some(0.0));
        assert_eq!(controller.max(), Some(1.0));
        assert_eq!(controller.update(10.0), 1.0);
        assert_eq!(controller.update(-10.0), 0.0);
    }

    #[test]
    fn set_min_and_set_max_can_clear_a_bound() {
        let mut controller = PController::with_limits(2.0, Some(0.0), Some(1.0));
        controller.set_min(None);
        controller.set_max(None);
        assert_eq!(controller.min(), None);
        assert_eq!(controller.max(), None);
        assert_eq!(controller.update(10.0), 20.0);
    }

    #[test]
    fn clear_limits_removes_both_bounds() {
        let mut controller = PController::with_limits(2.0, Some(0.0), Some(1.0));
        controller.clear_limits();
        assert_eq!(controller.min(), None);
        assert_eq!(controller.max(), None);
        assert_eq!(controller.update(10.0), 20.0);
    }

    #[test]
    fn set_gain_updates_gain() {
        let mut controller = PController::new(2.0);
        controller.set_gain(5.0);
        assert_eq!(controller.gain(), 5.0);
        assert_eq!(controller.update(2.0), 10.0);
    }

    #[test]
    fn pid_proportional_only_matches_p_controller() {
        let mut pid = PidController::new(2.0, 0.0, 0.0);
        assert_eq!(pid.update(1.5, 0.1), 3.0);
        assert_eq!(pid.update(-1.5, 0.1), -3.0);
    }

    #[test]
    fn pid_integral_accumulates_over_time() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        assert_eq!(pid.update(2.0, 0.5), 1.0); // integral = 1.0
        assert_eq!(pid.update(2.0, 0.5), 2.0); // integral = 2.0
        assert_eq!(pid.integral(), 2.0);
    }

    #[test]
    fn pid_derivative_reacts_to_error_change() {
        let mut pid = PidController::new(0.0, 0.0, 1.0);
        assert_eq!(pid.update(1.0, 0.5), 2.0); // (1.0 - 0.0) / 0.5
        assert_eq!(pid.update(3.0, 0.5), 4.0); // (3.0 - 1.0) / 0.5
    }

    #[test]
    #[should_panic]
    fn pid_zero_dt_panics_in_debug() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.update(2.0, 0.0);
    }

    #[test]
    #[should_panic]
    fn pid_negative_dt_panics_in_debug() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.update(2.0, -1.0);
    }

    #[test]
    fn pid_combines_all_three_terms() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        // error = 2.0, dt = 1.0: integral = 2.0, derivative = (2.0 - 0.0) / 1.0 = 2.0
        // output = 1.0*2.0 + 1.0*2.0 + 1.0*2.0 = 6.0
        assert_eq!(pid.update(2.0, 1.0), 6.0);
    }

    #[test]
    fn pid_output_clamps_to_limits() {
        let mut pid = PidController::with_limits(10.0, 0.0, 0.0, Some(-1.0), Some(1.0));
        assert_eq!(pid.update(5.0, 0.1), 1.0);
        assert_eq!(pid.update(-5.0, 0.1), -1.0);
    }

    #[test]
    fn pid_reset_clears_integral_and_derivative_state() {
        let mut pid = PidController::new(0.0, 1.0, 1.0);
        pid.update(2.0, 1.0);
        assert_ne!(pid.integral(), 0.0);

        pid.reset();
        assert_eq!(pid.integral(), 0.0);
        // Derivative term should react as if there were no prior error.
        assert_eq!(pid.update(2.0, 1.0), 2.0 + 2.0); // integral (2.0) + derivative (2.0)
    }

    #[test]
    fn pid_gains_are_exposed() {
        let pid = PidController::new(1.0, 2.0, 3.0);
        assert_eq!(pid.proportional_gain(), 1.0);
        assert_eq!(pid.integral_gain(), 2.0);
        assert_eq!(pid.derivative_gain(), 3.0);
    }

    #[test]
    fn pid_set_min_and_set_max_update_bounds() {
        let mut pid = PidController::new(2.0, 0.0, 0.0);
        pid.set_min(Some(0.0));
        pid.set_max(Some(1.0));
        assert_eq!(pid.min(), Some(0.0));
        assert_eq!(pid.max(), Some(1.0));
        assert_eq!(pid.update(10.0, 0.1), 1.0);
    }

    #[test]
    fn pid_clear_limits_removes_both_bounds() {
        let mut pid = PidController::with_limits(2.0, 0.0, 0.0, Some(0.0), Some(1.0));
        pid.clear_limits();
        assert_eq!(pid.min(), None);
        assert_eq!(pid.max(), None);
        assert_eq!(pid.update(10.0, 0.1), 20.0);
    }

    #[test]
    fn pid_has_no_derivative_filter_by_default() {
        let pid = PidController::new(0.0, 0.0, 1.0);
        assert!(pid.derivative_filter().is_none());
    }

    #[test]
    fn pid_set_derivative_filter_smooths_noisy_derivative() {
        let dt = 0.1;
        let mut unfiltered = PidController::new(0.0, 0.0, 1.0);
        let mut filtered = PidController::new(0.0, 0.0, 1.0);
        filtered.set_derivative_filter(Some(LowPassFilter::from_alpha(0.1)));
        assert!(filtered.derivative_filter().is_some());

        // Alternating error, e.g. sensor noise, produces a large raw
        // derivative every call; the filtered controller should react much
        // less strongly to it.
        let mut unfiltered_peak = 0.0f64;
        let mut filtered_peak = 0.0f64;
        for i in 0..20 {
            let error = if i % 2 == 0 { 1.0 } else { -1.0 };
            unfiltered_peak = unfiltered_peak.max(unfiltered.update(error, dt).abs());
            filtered_peak = filtered_peak.max(filtered.update(error, dt).abs());
        }

        assert!(filtered_peak < unfiltered_peak);
    }

    #[test]
    fn pid_reset_clears_derivative_filter_state() {
        let mut pid = PidController::new(0.0, 0.0, 1.0);
        pid.set_derivative_filter(Some(LowPassFilter::from_alpha(0.5)));

        pid.update(10.0, 1.0);
        assert_ne!(pid.derivative_filter().unwrap().value(), 0.0);

        pid.reset();
        assert_eq!(pid.derivative_filter().unwrap().value(), 0.0);
    }

    #[test]
    fn pid_integral_unclamped_by_default() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        for _ in 0..100 {
            pid.update(10.0, 1.0);
        }
        assert_eq!(pid.integral(), 100.0 * 10.0);
    }

    #[test]
    fn pid_integral_clamps_symmetrically_to_configured_limit() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.set_integral_limit(Some(5.0));

        for _ in 0..100 {
            pid.update(10.0, 1.0);
        }
        assert_eq!(pid.integral(), 5.0);

        for _ in 0..100 {
            pid.update(-10.0, 1.0);
        }
        assert_eq!(pid.integral(), -5.0);
    }

    #[test]
    fn pid_integral_limit_works_without_output_limits() {
        // No output limits are configured at all, only the integral limit.
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        assert_eq!(pid.min(), None);
        assert_eq!(pid.max(), None);

        pid.set_integral_limit(Some(2.0));
        for _ in 0..100 {
            pid.update(10.0, 1.0);
        }
        assert_eq!(pid.integral(), 2.0);
    }

    #[test]
    fn pid_integral_limit_unwinds_quickly_after_clamping() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.set_integral_limit(Some(5.0));
        for _ in 0..100 {
            pid.update(10.0, 1.0);
        }
        assert_eq!(pid.integral(), 5.0);

        // A single call with a large opposite error should be enough to
        // move off the clamp, unlike an unwound integral which would need
        // many calls to unwind the accumulated windup first.
        pid.update(-10.0, 1.0);
        assert!(pid.integral() < 5.0);
    }

    #[test]
    fn pid_integral_limit_getter_and_setter() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        assert_eq!(pid.integral_limit(), None);

        pid.set_integral_limit(Some(3.0));
        assert_eq!(pid.integral_limit(), Some(3.0));

        pid.set_integral_limit(None);
        assert_eq!(pid.integral_limit(), None);
    }

    #[test]
    fn pid_set_gains_updates_each_gain_independently() {
        let mut pid = PidController::new(1.0, 2.0, 3.0);
        pid.set_proportional_gain(4.0);
        pid.set_integral_gain(5.0);
        pid.set_derivative_gain(6.0);
        assert_eq!(pid.proportional_gain(), 4.0);
        assert_eq!(pid.integral_gain(), 5.0);
        assert_eq!(pid.derivative_gain(), 6.0);
    }

    #[test]
    fn pid_master_gain_defaults_to_one_and_does_not_change_output() {
        let pid = PidController::new(1.0, 0.0, 0.0);
        assert_eq!(pid.master_gain(), 1.0);
    }

    #[test]
    fn pid_set_master_gain_scales_combined_output() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.set_master_gain(2.0);
        assert_eq!(pid.master_gain(), 2.0);
        // error = 2.0, dt = 1.0: integral = 2.0, derivative = (2.0 - 0.0) / 1.0 = 2.0
        // combined = 1.0*2.0 + 1.0*2.0 + 1.0*2.0 = 6.0, scaled by master_gain = 2.0 -> 12.0
        assert_eq!(pid.update(2.0, 1.0), 12.0);
    }

    #[test]
    fn pid_master_gain_scales_before_output_clamp() {
        let mut pid = PidController::with_limits(1.0, 0.0, 0.0, Some(-1.0), Some(1.0));
        pid.set_master_gain(10.0);
        assert_eq!(pid.update(0.5, 0.1), 1.0); // 10.0 * 0.5 = 5.0, clamped to 1.0
    }
}
