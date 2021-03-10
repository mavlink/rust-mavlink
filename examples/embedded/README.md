# rust-MAVLink Embedded example
### How to run:
- Install cargo flash:
  - cargo install cargo-flash
- Install toolchain
  - rustup target add thumbv7em-none-eabihf  --toolchain nightly
- Check if we can build the project
  - cargo +nightly build
- Connect your STM32f303Xe board
- Flash it!
  - cargo +nightly flash --chip stm32f303RETx --release --log info
