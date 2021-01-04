# ambientsensor-rs

This is a rust re-implementation of an embedded C project I did a while ago: [AmbientSensor].

Schematics, photos, ect. will be in that repository, this repository is just for the rust firmware.

## Status

This is still a work in progress.

1. DHCP rebinding is not handled.
2. The OPT3002 luminosity sensor is not implemented.
3. Logging needs to be hidden behind a feature flag for release builds.
4. The panic handler needs to get hooked up to the EEPROM to store error logs then reset.

## Purpose

Co-workers kept hyping up this new rust language, and I wanted to see if the hype was warranted.

On paper rust seems amazing, but the only way to really evaluate a language is to use it.  I specifically wanted to compare embedded rust to embedded C, and [AmbientSensor] was simply the last embedded project I completed in C.

Turns out, the hype was warranted.

There is not a lot of commentary I can offer about embedded rust that has not already been said.  The concepts rust introduces (safety, ownership, lifetimes) make developing robust code simple.  The language is new and still has a small (but rapidly growing) ecosystem for embedded development.  From my experience a lot of embedded development is still C simply because nothing else (except C++ depending on who you ask) offers a compelling reason to switch.  The features rust brings to the table are incredibly compelling for embedded development, and I think rust has a bright future in this industry.

## Notable crates used

* [rtic](https://rtic.rs/0.5/book/en/) - RTOS
* [rtt_target](https://docs.rs/rtt-target/0.3.0/rtt_target/) - Probe based logging 
* [cargo-embed](https://crates.io/crates/cargo-embed) - build and run your code with one command
* [embedded-hal](https://github.com/rust-embedded/embedded-hal) - abstractions for embedded hardware

## Crates Developed

* [eeprom25aa02e48](https://github.com/newAM/eeprom25aa02e48-rs) - EEPROM driver, my first rust code
* [w5500-ll](https://github.com/newAM/w5500-ll-rs) - W5500 Ethernet chip driver

## Other Mentions

The [ferrous systems blog] is a great read, and they keep me excited about the future of embedded development.

[ferrous systems blog]: https://ferrous-systems.com/blog/
[AmbientSensor]: https://github.com/newam/ambientsensor
