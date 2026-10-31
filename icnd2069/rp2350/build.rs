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
    println!("cargo:rerun-if-env-changed=ICND2069_GRAY_MODE");
    println!("cargo:rerun-if-env-changed=ICND2069_CHAIN_COPIES");
    println!("cargo:rerun-if-env-changed=ICND2069_SCAN_REPEATS");
    println!("cargo:rerun-if-env-changed=ICND2069_SHORT_DATA");
    println!("cargo:rerun-if-env-changed=ICND2069_REPEAT_LATCH_GAP");
    println!("cargo:rerun-if-env-changed=ICND2069_TARGET_CHANNEL");
    let target: usize = env::var("ICND2069_TARGET_CHANNEL")
        .unwrap_or_else(|_| "0".into())
        .parse()
        .expect("ICND2069_TARGET_CHANNEL must be an integer in 0..=15");
    assert!(target < 16, "ICND2069_TARGET_CHANNEL must be in 0..=15");
    println!("cargo:rustc-env=ICND2069_TARGET_CHANNEL={target}");
    let short_data = env::var("ICND2069_SHORT_DATA").unwrap_or_else(|_| "1".into());
    assert!(short_data == "0" || short_data == "1", "ICND2069_SHORT_DATA must be 0 or 1");
    println!("cargo:rustc-env=ICND2069_SHORT_DATA={short_data}");
    let latch_gap: usize = env::var("ICND2069_REPEAT_LATCH_GAP")
        .unwrap_or_else(|_| "1".into())
        .parse()
        .expect("ICND2069_REPEAT_LATCH_GAP must be an integer in 1..=64");
    assert!((1..=64).contains(&latch_gap), "ICND2069_REPEAT_LATCH_GAP must be in 1..=64");
    println!("cargo:rustc-env=ICND2069_REPEAT_LATCH_GAP={latch_gap}");
    let repeats: usize = env::var("ICND2069_SCAN_REPEATS")
        .unwrap_or_else(|_| "1".into())
        .parse()
        .expect("ICND2069_SCAN_REPEATS must be an integer in 1..=64");
    assert!((1..=64).contains(&repeats), "ICND2069_SCAN_REPEATS must be in 1..=64");
    println!("cargo:rustc-env=ICND2069_SCAN_REPEATS={repeats}");
    let chain: usize = env::var("ICND2069_CHAIN_COPIES")
        .unwrap_or_else(|_| "1".into())
        .parse()
        .expect("ICND2069_CHAIN_COPIES must be an integer in 1..=8");
    assert!((1..=8).contains(&chain), "ICND2069_CHAIN_COPIES must be in 1..=8");
    println!("cargo:rustc-env=ICND2069_CHAIN_COPIES={chain}");
    let mode = env::var("ICND2069_GRAY_MODE").unwrap_or_else(|_| "sweep".into());
    assert!(mode == "sweep" || mode == "static", "ICND2069_GRAY_MODE must be sweep/static");
    let pattern = env::var("ICND2069_PATTERN").unwrap_or_else(|_| "white".into());
    assert!(mode != "sweep" || pattern == "white", "sweep mode requires white pattern");
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
             const GRAY: u16 = {gray};\n\
             const GRAY_SWEEP: bool = {};\n", mode == "sweep"
        ),
    ).unwrap();
}
