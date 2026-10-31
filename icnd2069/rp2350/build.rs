use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-env-changed=ICND2069_PATTERN");
    println!("cargo:rerun-if-env-changed=ICND2069_GRAY");
    let pattern = env::var("ICND2069_PATTERN").unwrap_or_else(|_| "reference".into());
    let variant = match pattern.as_str() {
        "reference" => "Reference",
        "white" => "White",
        "black" => "Black",
        "out0" => "Out0",
        "out1" => "Out1",
        "alternating_rows" => "AlternatingRows",
        "checker" => "Checker",
        "split" => "Split",
        _ => panic!("ICND2069_PATTERN must be reference/white/black/out0/out1/alternating_rows/checker/split"),
    };
    let gray: u16 = env::var("ICND2069_GRAY")
        .unwrap_or_else(|_| "8192".into())
        .parse()
        .expect("ICND2069_GRAY must be an integer in 0..=8192");
    assert!(gray <= 8192, "ICND2069_GRAY must be in 0..=8192");
    fs::write(
        out.join("pattern_config.rs"),
        format!(
            "const PATTERN: protocol::Pattern = protocol::Pattern::{variant};\n\
             const PATTERN_NAME: &str = \"{pattern}\";\n\
             const GRAY: u16 = {gray};\n"
        ),
    ).unwrap();
}
