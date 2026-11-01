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
    println!("cargo:rerun-if-env-changed=ICND2069_CONTINUOUS_LATCH");
    println!("cargo:rerun-if-env-changed=ICND2069_REPEAT_LATCH_GAP");
    println!("cargo:rerun-if-env-changed=ICND2069_TARGET_CHANNEL");
    println!("cargo:rerun-if-env-changed=ICND2069_CONFIG_INTERVAL");
    println!("cargo:rerun-if-env-changed=ICND2069_DCLK_MHZ");
    let dclk = env::var("ICND2069_DCLK_MHZ").unwrap_or_else(|_| "12.5".into());
    assert!(
        matches!(dclk.as_str(), "12.5" | "25" | "35" | "40" | "45"),
        "ICND2069_DCLK_MHZ must be 12.5, 25, 35, 40 or 45 (40/45 experimental)"
    );
    println!("cargo:rustc-env=ICND2069_DCLK_MHZ={dclk}");
    let interval: u32 = env::var("ICND2069_CONFIG_INTERVAL")
        .unwrap_or_else(|_| "10000".into())
        .parse()
        .expect("ICND2069_CONFIG_INTERVAL must be an integer in 1..=4294967295");
    assert!(interval > 0, "ICND2069_CONFIG_INTERVAL must be at least 1");
    println!("cargo:rustc-env=ICND2069_CONFIG_INTERVAL={interval}");
    let target: usize = env::var("ICND2069_TARGET_CHANNEL")
        .unwrap_or_else(|_| "0".into())
        .parse()
        .expect("ICND2069_TARGET_CHANNEL must be an integer in 0..=15");
    assert!(target < 16, "ICND2069_TARGET_CHANNEL must be in 0..=15");
    println!("cargo:rustc-env=ICND2069_TARGET_CHANNEL={target}");
    let continuous_latch = env::var("ICND2069_CONTINUOUS_LATCH")
        .or_else(|_| env::var("ICND2069_SHORT_DATA"))
        .unwrap_or_else(|_| "1".into());
    assert!(
        continuous_latch == "0" || continuous_latch == "1",
        "ICND2069_CONTINUOUS_LATCH (legacy ICND2069_SHORT_DATA) must be 0 or 1"
    );
    println!("cargo:rustc-env=ICND2069_CONTINUOUS_LATCH={continuous_latch}");
    println!("cargo:rustc-env=ICND2069_SHORT_DATA={continuous_latch}");
    let latch_gap: usize = env::var("ICND2069_REPEAT_LATCH_GAP")
        .unwrap_or_else(|_| "1".into())
        .parse()
        .expect("ICND2069_REPEAT_LATCH_GAP must be an integer in 1..=64");
    assert!((1..=64).contains(&latch_gap), "ICND2069_REPEAT_LATCH_GAP must be in 1..=64");
    println!("cargo:rustc-env=ICND2069_REPEAT_LATCH_GAP={latch_gap}");
    // Keep the default in protocol.rs only. Without an explicit override,
    // option_env! must remain None so editing SCAN_REPEATS changes the build.
    if let Ok(value) = env::var("ICND2069_SCAN_REPEATS") {
        let repeats: usize = value.parse()
            .expect("ICND2069_SCAN_REPEATS must be an integer in 1..=64");
        assert!((1..=64).contains(&repeats), "ICND2069_SCAN_REPEATS must be in 1..=64");
        println!("cargo:rustc-env=ICND2069_SCAN_REPEATS={repeats}");
    }
    let chain: usize = env::var("ICND2069_CHAIN_COPIES")
        .unwrap_or_else(|_| "1".into())
        .parse()
        .expect("ICND2069_CHAIN_COPIES must be an integer in 1..=8");
    assert!((1..=8).contains(&chain), "ICND2069_CHAIN_COPIES must be in 1..=8");
    println!("cargo:rustc-env=ICND2069_CHAIN_COPIES={chain}");
    let mode = env::var("ICND2069_GRAY_MODE").unwrap_or_else(|_| "static".into());
    assert!(mode == "sweep" || mode == "static", "ICND2069_GRAY_MODE must be sweep/static");
    let pattern = env::var("ICND2069_PATTERN")
        .unwrap_or_else(|_| if mode == "sweep" { "white".into() } else { "diagonal".into() });
    assert!(mode != "sweep" || pattern == "white", "sweep mode requires white pattern");
    assert!(
        pattern != "gradient" || continuous_latch == "0",
        "gradient requires ICND2069_CONTINUOUS_LATCH=0 to write all 256 pixels"
    );
    println!("cargo:rustc-env=ICND2069_PATTERN={pattern}");
    let variant = match pattern.as_str() {
        "diagonal" => "Diagonal",
        "gradient" => "Gradient",
        "reference" => "Reference",
        "white" => "White",
        "black" => "Black",
        "out0" => "Out0",
        "out1" => "Out1",
        "alternating_rows" => "AlternatingRows",
        "checker" => "Checker",
        "split" => "Split",
        _ => panic!("ICND2069_PATTERN must be diagonal/gradient/reference/white/black/out0/out1/alternating_rows/checker/split"),
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
