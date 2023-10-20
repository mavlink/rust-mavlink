#![recursion_limit = "256"]
#[macro_use]
extern crate quote;

extern crate xml;

mod binder;
mod parser;
mod util;

use crate::util::to_module_name;
use std::env;
use std::fs::{read_dir, File};
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
        Ok(_) => {}
        Err(error) => eprintln!("{}", error),
    }

    // find & apply patches to XML definitions to avoid crashes
    let mut patch_dir = src_dir.to_path_buf();
    patch_dir.push("build/patches");
    let mut mavlink_dir = src_dir.to_path_buf();
    mavlink_dir.push("mavlink");

    if let Ok(dir) = read_dir(patch_dir) {
        for entry in dir {
            if let Ok(entry) = entry {
                println!("cargo:info=Add patch: {:?}", &entry.path());
                match Command::new("git")
                    .arg("apply")
                    .arg(entry.path().as_os_str())
                    .current_dir(&mavlink_dir)
                    .status()
                {
                    Ok(_) => (),
                    Err(error) => eprintln!("{}", error),
                }
            }
        }
    }

    let mut definitions_dir = src_dir.to_path_buf();
    definitions_dir.push("mavlink/message_definitions/v1.0");

    let out_dir = env::var("OUT_DIR").unwrap();

    let mut modules = vec![];

    for entry in read_dir(&definitions_dir).expect("could not read definitions directory") {
        let entry = entry.expect("could not read directory entry");

        let definition_file = entry.file_name();
        let module_name = to_module_name(&definition_file);

        let mut definition_rs = PathBuf::from(&module_name);
        definition_rs.set_extension("rs");

        modules.push(module_name);

        let in_path = Path::new(&definitions_dir).join(&definition_file);
        let mut inf = File::open(&in_path).unwrap();

        let dest_path = Path::new(&out_dir).join(definition_rs);
        let mut outf = File::create(&dest_path).unwrap();

        // generate code
        parser::generate(&mut inf, &mut outf);

        // format code
        match Command::new("rustfmt")
            .arg(dest_path.as_os_str())
            .current_dir(&out_dir)
            .status()
        {
            Ok(_) => (),
            Err(error) => eprintln!("{}", error),
        }

        // Re-run build if definition file changes
        println!("cargo:rerun-if-changed={}", entry.path().to_string_lossy());
    }

    // output mod.rs
    {
        let dest_path = Path::new(&out_dir).join("mod.rs");
        let mut outf = File::create(&dest_path).unwrap();

        // generate code
        binder::generate(modules, &mut outf);

        // format code
        match Command::new("rustfmt")
            .arg(dest_path.as_os_str())
            .current_dir(&out_dir)
            .status()
        {
            Ok(_) => (),
            Err(error) => eprintln!("{}", error),
        }
    }
}
