use std::fs;
use std::path::PathBuf;

use insta::{self, assert_snapshot, glob};
use mavlink_bindgen::{format_generated_code, generate, XmlDefinitions};
use tempfile::TempDir;

fn definitions_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("definitions")
}

fn run_snapshot(def_file: &str) {
    let defs = definitions_dir();
    let tmp = TempDir::new().expect("tmp dir");
    let out_dir = tmp.path();

    let xml = defs.join(def_file);
    let result = generate(XmlDefinitions::Files(vec![xml]), out_dir).expect("generate ok");

    format_generated_code(&result);

    glob!(out_dir, "**/*.rs", |path| {
        let contents = fs::read_to_string(path).expect("read generated file");
        assert_snapshot!(def_file, contents);
    });
}

#[test]
fn snapshot_heartbeat() {
    run_snapshot("heartbeat.xml");
}

#[test]
fn snapshot_parameters() {
    run_snapshot("parameters.xml");
}

#[test]
fn snapshot_deprecated() {
    run_snapshot("deprecated.xml");
}

#[test]
fn snapshot_superseded() {
    run_snapshot("superseded.xml");
}

#[test]
fn snapshot_no_field_description() {
    run_snapshot("no_field_description.xml");
}

#[test]
fn snapshot_mav_bool() {
    run_snapshot("mav_bool.xml");
}

#[test]
fn snapshot_mav_cmd() {
    run_snapshot("mav_cmd.xml");
}
