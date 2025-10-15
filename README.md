# Simple SD/MMC Driver

This crate is a simple SD/MMC driver based on SDIO. Pure Rust, `#![no_std]` and no `alloc`.

*Experimental*

## Features

- **SD Card Support**: Full support for SD cards including SDHC/SDXC
- **eMMC Support**: Native support for embedded MultiMediaCard (eMMC) devices
- **Automatic Detection**: Automatically detects whether the connected device is an SD card or eMMC
- **Pure Rust**: Written entirely in Rust with no external dependencies
- **no_std Compatible**: Works in embedded environments without the standard library
- **Block-level Operations**: Read and write operations at the block level (512 bytes per block)

## Usage

### Automatic Card Detection

```rust
use simple_sdmmc::SdMmc;

// The driver will automatically detect SD or eMMC
let mut driver = unsafe { SdMmc::new(0x1000_0000) };

// Check what type of card was detected
match driver.card_type() {
    "eMMC" => println!("eMMC detected"),
    "SD" => println!("SD card detected"),
    _ => unreachable!(),
}

// Read/write operations work the same for both types
let mut buffer = [0u8; 512];
driver.read_block(0, &mut buffer);
driver.write_block(1, &buffer);
```

### Explicit eMMC Initialization

```rust
use simple_sdmmc::SdMmc;

// Force eMMC initialization (skips SD detection)
let mut emmc = unsafe { SdMmc::new_emmc(0x1000_0000) };
```

## Optional features

- `gpt`: implement [gpt_disk_io::BlockIo](https://docs.rs/gpt_disk_io/0.16.2/gpt_disk_io/trait.BlockIo.html)
