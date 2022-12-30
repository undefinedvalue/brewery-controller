#![cfg_attr(not(test), no_std)]

pub mod common;
pub mod max31865;
pub mod mcp23008;
pub mod rotary_encoder;
pub mod seven_segment_display;

pub(crate) mod ht16k33;

#[cfg(test)]
pub(crate) mod mocks;