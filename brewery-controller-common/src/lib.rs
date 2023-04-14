#![cfg_attr(not(test), no_std)]
#![feature(type_alias_impl_trait)]

pub mod circular_buffer;
pub mod common;
pub mod max31865;
pub mod pid;
pub mod rotary_encoder;
pub mod seven_segment_display;
pub mod solid_state_relay;
pub mod temperature_stats;

pub(crate) mod ht16k33;

#[cfg(test)]
pub(crate) mod mocks;