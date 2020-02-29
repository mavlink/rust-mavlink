#![recursion_limit="256"]
#[macro_use]
extern crate quote;

extern crate crc16;
extern crate xml;

mod parser;

use std::env;
use std::fs::{File, read_dir};
use std::path::{Path, PathBuf};
use std::process::Command;

pub fn main() {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR"));

    // Update and init submodule
    match Command::new("git")
        .arg("submodule")
        .arg("update")
        .arg("--init")
        .current_dir(&src_dir)
        .status()
    {
        Ok(content) => println!("{}", content),
        Err(error) => eprintln!("{}", error),
    }

    let mut definitions_dir = src_dir.to_path_buf();
    definitions_dir.push("mavlink/message_definitions/v1.0");

    for entry in read_dir(&definitions_dir).expect("could not read definitions directory") {
        let entry = entry.expect("could not read directory entry");

        let definition_file = PathBuf::from(entry.file_name());
        let mut definition_rs = definition_file.clone();
        definition_rs.set_extension("rs");

        let in_path = Path::new(&definitions_dir).join(&definition_file);
        let mut inf = File::open(&in_path).unwrap();

        let out_dir = env::var("OUT_DIR").unwrap();
        let dest_path = Path::new(&out_dir).join(definition_rs);
        let mut outf = File::create(&dest_path).unwrap();

        parser::generate(&mut inf, &mut outf);

        // Re-run build if common.xml changes
        println!(
            "cargo:rerun-if-changed={}",
            definition_file.to_string_lossy()
        );
    }
}
