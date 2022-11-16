# rust-MAVLink Embedded example
### How to run:
- Install cargo flash:
  - cargo install cargo-flash
- Install target
  - rustup target add thumbv7em-none-eabihf
- Check if we can build the project
  - cargo build
- Connect your STM32f303Xe board
- Flash it!
  - cargo flash --chip stm32f303RETx --release --log info
