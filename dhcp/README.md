# DHCP

This is broken up into its own crate to enable unit testing.

I implemented my own DHCP client for reasons that are not applicable to a production product:

1. Fun
2. Learning

This is pretty low-effort code here.
Minimum viable product sort of thing.

## Running Tests

```
cargo test -p dhcp --target x86_64-unknown-linux-gnu
```
