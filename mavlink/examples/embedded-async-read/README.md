# rust-MAVLink Embedded async example (with reading loop)
### How to run:
- Install cargo flash:
  - cargo install cargo-flash
- Install target
  - rustup target add thumbv7em-none-eabihf
- Check if we can build the project
  - cargo build
- Connect your STM32f446re board
- Flash it!
  - cargo flash --chip stm32f446RETx --release --log info
