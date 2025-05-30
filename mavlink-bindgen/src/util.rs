use std::path::PathBuf;

pub fn to_module_name<P: Into<PathBuf>>(file_name: P) -> String {
    file_name
        .into()
        .file_stem() // remove extension
        .unwrap()
        .to_string_lossy() // convert to string
        .to_lowercase() // all lowercase
        .replace(|c: char| !c.is_alphanumeric(), "_") // remove non alphanum
}

pub fn to_dialect_name<P: Into<PathBuf>>(file_name: P) -> String {
    file_name
        .into()
        .file_stem() // remove extension
        .unwrap()
        .to_string_lossy()
        .to_string()
}
