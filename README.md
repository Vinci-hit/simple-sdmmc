# Simple SD/MMC Driver

This crate is a simple SD/MMC driver based on SDIO. Pure Rust, `#![no_std]` and no `alloc`.

*Experimental*

## Optional features

- `gpt`: implement [gpt_disk_io::BlockIo](https://docs.rs/gpt_disk_io/0.16.2/gpt_disk_io/trait.BlockIo.html)
