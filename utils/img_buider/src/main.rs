use clap::Parser;

#[derive(Debug, Parser)]
#[command(name = "img_buider")]
struct Args {
    #[arg(short, long, default_value = "./imgs")]
    out_dir: String,
    #[arg(
        short,
        long,
        default_value = "/Users/hikari/rust/vdrmtd/output_rgbh_1784963625housecar.png"
    )]
    input: String,
    #[arg(long, default_value_t = 0.5f32)]
    gamma: f32,
}

fn brighten_gamma(v: u8, gamma: f32) -> u8 {
    let normalized = v as f32 / 255.0;
    let corrected = normalized.powf(gamma);
    (corrected * 255.0).round().clamp(0.0, 255.0) as u8
}

fn parse_angle_line(
    angle: u32,
    line: Vec<vdrm_alg::ScreenLine>,
) -> Option<mbi5264_common::AngleImage> {
    let mut img = mbi5264_common::AngleImage::new(angle);
    let mut pixels: [Option<[u8; 4]>; mbi5264_common::IMG_HEIGHT] =
        [None; mbi5264_common::IMG_HEIGHT];
    if line.is_empty() {
        return None;
    }
    for p in line {
        let addr = p.addr;
        if addr >= 144 {
            continue;
        }
        // reverse
        // addr = 143 - addr;
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
    for (c, p) in img.coloum.iter_mut().zip(pixels) {
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
    Some(img)
}

fn gen_threed_surface(input: &str, gamma: f32) -> vdrm_alg::PixelSurface {
    let img = image::open(input).unwrap();
    let rgb_img = img.as_rgb8().unwrap();
    let width = img.width();
    let height = img.height();
    assert_eq!(width, 384 * 2);
    assert_eq!(height, 384);
    let mut surface = vdrm_alg::PixelSurface::default();
    for y in 0..192u32 {
        for x in 0..192u32 {
            let px = x * 2;
            let px2 = px + 384;
            let py = y * 2;
            let rgb = rgb_img.get_pixel(px, py).0;
            if rgb == [0; 3] {
                continue;
            }
            let [r, g, b] = rgb;
            let r = brighten_gamma(r, gamma);
            let g = brighten_gamma(g, gamma);
            let b = brighten_gamma(b, gamma);
            let h = rgb_img.get_pixel(px2, py).0[0];
            let h = (255 - h) / 2;
            let rgb = u32::from_ne_bytes([r, g, b, 0]);
            let x = 191 - x;
            surface.push((x, y, (h as u32, rgb)));
        }
    }
    surface
}

fn main() {
    let args = Args::parse();
    let image_dir = args.out_dir;

    std::fs::create_dir_all(&image_dir).unwrap();
    let codec = vdrm_alg::Codec::new();
    // let surface = gen_rrds_surface();
    // let surface = gen_pyramid_surface();
    let surface = gen_threed_surface(&args.input, args.gamma);
    let map = codec.encode(&surface, 0, true);
    let mut angle_lists = [0; vdrm_alg::NUM_SCREENS].map(|_| vec![]);
    for (angle, screen_lines) in map {
        for (angle_list, line) in angle_lists.iter_mut().zip(screen_lines) {
            let Some(mut img) = parse_angle_line(angle, line) else {
                continue;
            };
            for rgbh in img.coloum.iter_mut() {
                // fix hight for 5x circuit
                rgbh[3] = rgbh[3] + 16;
            }
            angle_list.push(img);
        }
    }

    for (idx, angle_list) in angle_lists.into_iter().enumerate() {
        let mut init_angle = (vdrm_alg::TOTAL_ANGLES / 4) - vdrm_alg::W_PIXELS / 2;
        let offset = vdrm_alg::W_PIXELS / 4;
        init_angle -= offset;
        init_angle += offset * idx;
        let len = angle_list.len();
        let image_path = format!("{image_dir}/img{idx}_{len}.bin");
        let img_size = len * std::mem::size_of::<mbi5264_common::AngleImage>();
        let mut buf: Vec<u8> = vec![];
        buf.extend_from_slice(&(init_angle as u32).to_le_bytes());
        buf.extend_from_slice(&(len as u32).to_le_bytes());
        let img_buf =
            unsafe { std::slice::from_raw_parts(angle_list.as_ptr() as *const u8, img_size) };
        buf.extend_from_slice(img_buf);
        std::fs::write(image_path, buf).unwrap();

        let dbg_path = format!("{image_dir}/img{idx}_{len}.png");
        let mut img = [[false; 160]; 192];

        for angle_img in angle_list {
            for (line, p) in angle_img.coloum.iter().enumerate() {
                if p[0..3] == [0; 3] {
                    continue;
                }
                let col = p[3];
                img[line][col as usize] = true;
            }
        }

        let mut dbg_buf = image::RgbImage::new(160, 192);
        for (y, row) in img.iter().rev().enumerate() {
            for (x, occupied) in row.iter().enumerate() {
                let pixel = if *occupied {
                    image::Rgb([255, 255, 255])
                } else {
                    image::Rgb([0, 0, 0])
                };
                dbg_buf.put_pixel(x as u32, y as u32, pixel);
            }
        }

        dbg_buf.save(dbg_path).unwrap();
    }
}
