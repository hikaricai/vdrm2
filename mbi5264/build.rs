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
            let z = h.abs() as u32;
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
    let image_path = std::path::Path::new(crate_dir.as_str()).join("img.bin");
    if !image_path.exists() {
        let codec = vdrm_alg::Codec::new(0..400);
        let pyramid = gen_pyramid_surface();
        let map = codec.encode(&pyramid, 0);
        let mut angle_list = vec![];
        for (angle, line) in map {
            let mut img = mbi5264_common::AngleImage::new(angle);
            let mut pixels: [Option<[u8; 4]>; mbi5264_common::IMG_HEIGHT] =
                [None; mbi5264_common::IMG_HEIGHT];
            for p in line {
                for (idx, color) in p.pixels.iter().enumerate() {
                    let Some(color) = color else {
                        continue;
                    };
                    let [r, g, b, _a] = color.to_ne_bytes();
                    match pixels[idx].as_mut() {
                        Some(rgbh) => {
                            let h = rgbh[3];
                            if p.addr < h as u32 {
                                *rgbh = [r, g, b, p.addr as u8];
                            }
                        }
                        None => {
                            pixels[idx] = Some([r, g, b, p.addr as u8]);
                        }
                    }
                }
            }
            for (c, p) in img.coloum.iter_mut().zip(pixels) {
                if let Some(p) = p {
                    *c = p;
                }
            }
            angle_list.push(img);
        }
    }
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

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
