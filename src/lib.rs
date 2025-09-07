//! A platform agnostic Rust driver for XPT2046 touch controller based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!
//! In addition, it provides functions for calculating the calibration data
//! needed to align the touch panel position measurements with the display panel
//! pixel locations as well as a function for performing this calibration.

// #![doc(html_root_url = "https://")]
// #![doc(issue_tracker_base_url = "https://github.com/pebender/xpt2046/issues/")]
#![deny(
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    unused_variables,
    unreachable_code,
    unused_comparisons,
    unused_must_use
)]
#![no_std]

pub mod driver;

#[cfg(feature = "calibration")]
pub mod calibration;

#[cfg(feature = "calibration_run")]
pub mod calibration_run;
