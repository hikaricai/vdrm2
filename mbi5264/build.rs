//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn gen_pyramid_surface() -> vdrm_alg::PixelSurface {
    let mut pixel_surface = vdrm_alg::PixelSurface::new();
    let gray: u8 = 0xff;
    let r = vdrm_alg::W_PIXELS as i32 / 2;
    for x in 0..vdrm_alg::W_PIXELS as u32 {
        for y in 0..vdrm_alg::W_PIXELS as u32 {
            let x_i32 = x as i32 - r;
            let y_i32 = y as i32 - r;
            let h = x_i32.abs() + y_i32.abs();
            if h >= vdrm_alg::H_PIXELS as i32 {
                continue;
            }
            // let z = vdrm_alg::H_PIXELS as u32 - 1 - h as u32;
            let z = h as u32;
            let color = match (x_i32 >= 0, y_i32 >= 0) {
                (true, true) => u32::from_ne_bytes([gray, gray, gray, 0]),
                (false, true) => u32::from_ne_bytes([gray, 0, 0, 0]),
                (false, false) => u32::from_ne_bytes([0, gray, 0, 0]),
                (true, false) => u32::from_ne_bytes([gray, 0, gray, 0]),
            };
            pixel_surface.push((x, y, (z, color)));
        }
    }
    pixel_surface
}

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let const_path = std::path::Path::new(crate_dir.as_str()).join("src/consts.rs");

    std::fs::write(
        &const_path,
        format!(
            "pub const TOTAL_ANGLES: usize = {};",
            vdrm_alg::TOTAL_ANGLES
        ),
    )
    .unwrap();

    let image_path = std::path::Path::new(crate_dir.as_str()).join("img.bin");
    if !image_path.exists() {
        let codec = vdrm_alg::Codec::new(0..400);
        let pyramid = gen_pyramid_surface();
        let map = codec.encode(&pyramid, 0, true);
        let mut angle_list = vec![];
        for (angle, screen_lines) in map {
            let mut img = mbi5264_common::AngleImage::new(angle);
            let mut pixels: [Option<[u8; 4]>; mbi5264_common::IMG_HEIGHT] =
                [None; mbi5264_common::IMG_HEIGHT];
            let [line, ..] = screen_lines;
            if line.is_empty() {
                continue;
            }
            for p in line {
                let mut addr = p.addr;
                if addr >= 144 {
                    continue;
                }
                addr = 143 - addr;
                for (color, pixel) in p.pixels.iter().zip(&mut pixels) {
                    let Some(color) = color else {
                        continue;
                    };
                    let [r, g, b, _a] = color.to_ne_bytes();
                    match pixel {
                        Some(rgbh) => {
                            let h = rgbh[3];
                            if addr < h as u32 {
                                *rgbh = [r, g, b, addr as u8];
                            }
                        }
                        None => {
                            *pixel = Some([r, g, b, addr as u8]);
                        }
                    }
                }
            }
            for (c, p) in img.coloum.iter_mut().rev().zip(pixels) {
                if let Some(p) = p {
                    *c = p;
                }
            }
            // optimize fps
            for i in 0..64usize {
                let region0 = i;
                let region1 = i + 64;
                let region2 = i + 128;
                let regions = [region0, region1, region2];
                let mut non_empty_h = 0u8;
                for region in regions {
                    let p = img.coloum[region];
                    let h = p[3];
                    if p[..3] != [0, 0, 0] {
                        non_empty_h = h;
                    }
                }
                for region in regions {
                    let p = &mut img.coloum[region];
                    if p[..3] == [0, 0, 0] {
                        p[3] = non_empty_h;
                    }
                }
            }
            angle_list.push(img);
        }
        let buf = unsafe {
            std::slice::from_raw_parts(
                angle_list.as_ptr() as *const u8,
                angle_list.len() * std::mem::size_of::<mbi5264_common::AngleImage>(),
            )
        };
        std::fs::write(image_path, buf).unwrap();
    }
    // skip build
    // std::process::exit(0);

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=img.bin");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
