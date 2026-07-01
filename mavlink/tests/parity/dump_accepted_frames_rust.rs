use std::fs::File;
use std::io;
use std::path::PathBuf;
use std::process::ExitCode;

use mavlink::dialects::all::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::peek_reader::PeekReader;
use mavlink::{MAVLinkMessageRaw, read_any_raw_message};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("{err}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .ok_or_else(|| "usage: dump-accepted-frames-rust <mavlink-stream>".to_owned())?;

    let file =
        File::open(&path).map_err(|err| format!("failed to open {}: {err}", path.display()))?;
    let mut reader = PeekReader::new(file);

    loop {
        match read_any_raw_message::<MavMessage, _>(&mut reader) {
            Ok(raw_message) => println!("{}", encode_raw_message(&raw_message)),
            Err(MessageReadError::Io(err)) if err.kind() == io::ErrorKind::UnexpectedEof => break,
            Err(err) => return Err(format!("failed to parse {}: {err}", path.display())),
        }
    }

    Ok(())
}

fn encode_raw_message(raw_message: &MAVLinkMessageRaw) -> String {
    let bytes = match raw_message {
        MAVLinkMessageRaw::V1(message) => message.raw_bytes(),
        MAVLinkMessageRaw::V2(message) => message.raw_bytes(),
    };

    encode_hex(bytes)
}

fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";

    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}
