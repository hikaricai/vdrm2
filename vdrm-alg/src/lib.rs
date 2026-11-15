use geo::{ClosestPoint, EuclideanDistance, EuclideanLength, LineInterpolatePoint};
use std::collections::BTreeMap;

// pub const W_PIXELS: usize = 64;
// pub const H_PIXELS: usize = 40;
pub const W_PIXELS: usize = 192;
pub const H_PIXELS: usize = 130;
const CIRCLE_R: f32 = 1.;
pub const SCREEN_HEIGHT: f32 = SCREEN_ZOOM * CIRCLE_R * 2.;
const POINT_SIZE: f32 = SCREEN_ZOOM * 2. * CIRCLE_R / W_PIXELS as f32;
// screem 位置和大小
const SCREEN_ZOOM: f32 = 1.;
// 修改这个值 越大成像越远 屏幕间距越大
// pub const SCREEN_OFFSET: f32 = 0.25;
pub const SCREEN_OFFSET: f32 = 0.05;
pub const SCREEN_Z_OFFSET: f32 = -1.0 + SCREEN_OFFSET;
pub const SCREEN_Y_OFFSET: f32 = -1.0 + SCREEN_OFFSET;
// 八边形就x8 越大越清晰
pub const TOTAL_ANGLES: usize = W_PIXELS * 8;

// 点顺时针
// 坐标系逆时针
const MAT_ROTATE_X: glam::Mat2 = glam::Mat2::from_cols(
    glam::Vec2::new(
        std::f32::consts::FRAC_1_SQRT_2,
        -std::f32::consts::FRAC_1_SQRT_2,
    ),
    glam::Vec2::new(
        std::f32::consts::FRAC_1_SQRT_2,
        std::f32::consts::FRAC_1_SQRT_2,
    ),
);

const MAT_ROTATE_X_REV: glam::Mat2 = glam::Mat2::from_cols(
    glam::Vec2::new(
        std::f32::consts::FRAC_1_SQRT_2,
        std::f32::consts::FRAC_1_SQRT_2,
    ),
    glam::Vec2::new(
        -std::f32::consts::FRAC_1_SQRT_2,
        std::f32::consts::FRAC_1_SQRT_2,
    ),
);

pub fn rotate_x(v: (f32, f32)) -> (f32, f32) {
    let v = glam::Vec2::new(v.0, v.1);
    let v = MAT_ROTATE_X * v;
    (v.x, v.y)
}

pub fn rotate_x_rev(v: (f32, f32)) -> (f32, f32) {
    let v = glam::Vec2::new(v.0, v.1);
    let v = MAT_ROTATE_X_REV * v;
    (v.x, v.y)
}

// pub const TOTAL_ANGLES: usize = 360;

type PixelColor = u32;
type PixelXY = (u32, u32);
pub type PixelSurface = Vec<(u32, u32, (u32, PixelColor))>;
pub type FloatSurface = Vec<(f32, f32, f32)>;
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
struct ScreenLineAddr {
    screen_idx: usize,
    addr: u32,
}
#[derive(Debug, Copy, Clone)]
struct ScreenLinePixels {
    pixels: [Option<PixelColor>; W_PIXELS],
}

impl Default for ScreenLinePixels {
    fn default() -> Self {
        Self {
            pixels: [None; W_PIXELS],
        }
    }
}

pub const MIRROR_OFFSET: f32 = std::f32::consts::SQRT_2;
// const MIRROR_OFFSET2: f32 = -std::f32::consts::SQRT_2;
// const SCREEN_OFFSET: f32 = std::f32::consts::SQRT_2;
// const V_IMG_Y_TOP: f32 = (MIRROR_OFFSET2 * 2. - SCREEN_OFFSET) / std::f32::consts::SQRT_2;
// const V_IMG_Y_TOP: f32 = -2.;
// const V_IMG_Z_TOP: f32 = -3.;

// const VIRTUAL_IMG_CENTER: f32 = -4.;

fn cacul_v_img_cord() -> glam::Vec4 {
    let angle = (90f32).to_radians();
    let mat = mirror_mat4(angle);
    let p = glam::Vec4::new(0.0, SCREEN_Y_OFFSET, SCREEN_Z_OFFSET, 1.0);
    mat * p
}

pub const NUM_SCREENS: usize = 3;

lazy_static::lazy_static! {
    pub static ref V_IMG_CORD: glam::Vec4 = {
        cacul_v_img_cord()
    };
    pub static ref V_IMG_CENTER_CORD: glam::Vec4 = {
        let center_y = V_IMG_CORD.y - SCREEN_ZOOM;
        let center_z = V_IMG_CORD.z - 0.5 * SCREEN_ZOOM;
        glam::Vec4::new(0.0, center_y, center_z, 1.0)
    };
    static ref SCREENS:[Screen; NUM_SCREENS]  = {
        // screens_with_rotate(std::f32::consts::PI / 8., Some(std::f32::consts::PI / 8.))
        screens_with_rotate(0., None)
    };
}

pub fn screens_with_rotate(
    rad_rotate: f32,
    offset_middle_screen: Option<f32>,
) -> [Screen; NUM_SCREENS] {
    let z = SCREEN_Z_OFFSET;

    let depth = 2f32 * SCREEN_ZOOM;
    let a: (f32, f32) = (-0.0, SCREEN_Y_OFFSET);

    let b: (f32, f32) = (
        a.0 + depth * rad_rotate.sin(),
        a.1 + depth * rad_rotate.cos(),
    );

    let screen = Screen::new([a, b], z);
    let angle_off = 360.0f32 / 8.0 / 2.0 / 2.0;
    let angle_l = 90f32 - angle_off;
    let angle_r = 90f32 + angle_off;
    let v_screen = mirror_points_f(90f32.to_radians(), &screen.points);
    let screen_l = mirror_points_f(angle_l.to_radians(), &v_screen);
    let screen_r = mirror_points_f(angle_r.to_radians(), &v_screen);

    let screen_l = Screen {
        points: screen_l.try_into().unwrap(),
    };
    let screen_r = Screen {
        points: screen_r.try_into().unwrap(),
    };
    let screen = if let Some(angle) = offset_middle_screen {
        let screen_real_depth = 0.75;
        let offset = screen_real_depth * angle.sin() / 2.;
        let a = (a.0 - offset, a.1);
        let b = (b.0 - offset, b.1);
        Screen::new([a, b], z)
    } else {
        screen
    };
    [screen_l, screen, screen_r]
}

pub fn screens() -> &'static [Screen] {
    &*SCREENS
}
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
struct ScreenPixel {
    idx: usize,
    addr: u32,
    pixel: u32,
}

#[derive(Clone, Copy, Debug)]
struct PixelZInfo {
    angle: u32,
    pixel: u32,
    is_borrowed: bool,
    screen_pixel: ScreenPixel,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ScreenLine {
    pub screen_idx: usize,
    pub addr: u32,
    pub pixels: [Option<PixelColor>; W_PIXELS],
}

pub type AngleMap = BTreeMap<u32, [Vec<ScreenLine>; NUM_SCREENS]>;

type PixelZInfoList = [Option<PixelZInfo>; H_PIXELS];
type PixelXYArr = Vec<Vec<PixelZInfoList>>;

#[derive(Copy, Clone)]
pub struct Screen {
    pub points: [(f32, f32, f32); 4],
}

impl Screen {
    fn new(xy: [(f32, f32); 2], z: f32) -> Self {
        let xy_line = geo::Line::new(xy[0], xy[1]);
        let (a, b) = xy_line.points();
        let screen_top = z + SCREEN_HEIGHT;
        let points = [
            (a.x(), a.y(), z),
            (a.x(), a.y(), screen_top),
            (b.x(), b.y(), screen_top),
            (b.x(), b.y(), z),
        ];
        Self { points }
    }
}

pub fn pixel_surface_to_float(pixel_surface: &PixelSurface) -> FloatSurface {
    pixel_surface
        .into_iter()
        .map(|&(pixel_x, pixel_y, (pixel_z, _color))| {
            let x = pixel_to_v(pixel_x);
            let y = pixel_to_v(pixel_y);
            let z = pixel_to_h(pixel_z);
            (x, y, z)
        })
        .collect()
}

fn pixel_to_v(p: u32) -> f32 {
    let point_size: f32 = POINT_SIZE;
    p as f32 * point_size + 0.5 * point_size - CIRCLE_R * SCREEN_ZOOM
}

fn v_to_pixel(v: f32) -> Option<u32> {
    let point_size: f32 = POINT_SIZE;
    let v = (v + CIRCLE_R * SCREEN_ZOOM) / point_size - 0.5;
    if v < -POINT_SIZE {
        return None;
    }
    let mut v = v as u32;
    if v > W_PIXELS as u32 {
        return None;
    }
    if v == W_PIXELS as u32 {
        v -= 1;
    }
    Some(v)
}

fn pixel_to_h(p: u32) -> f32 {
    (p as f32) * POINT_SIZE
}

fn h_to_pixel(mut h: f32) -> Option<u32> {
    let point_size: f32 = POINT_SIZE;
    // fix z offset
    h += POINT_SIZE;
    if h < -POINT_SIZE {
        return None;
    }
    let mut p = (h / point_size) as u32;
    if p > H_PIXELS as u32 {
        return None;
    }
    if p == H_PIXELS as u32 {
        p -= 1;
    }
    Some(p)
}

fn v3_2_pixel(x: f32, y: f32, z: f32) -> Option<(u32, u32, u32)> {
    let x = v_to_pixel(x)?;
    let y = v_to_pixel(y)?;
    let z = h_to_pixel(z)?;
    // fix offset
    // if z + 1 < H_PIXELS as u32 {
    //     z += 1;
    // }
    Some((x, y, z))
}

pub fn angle_to_v(p: u32) -> f32 {
    (p as f32 * 360. / TOTAL_ANGLES as f32).to_radians()
}

fn cacl_view_point(
    mat: glam::Mat4,
    screen_idx: usize,
    addr: u32,
    pixel_z: u32,
) -> ((f32, f32, f32), (f32, f32, f32)) {
    let screen = &SCREENS[screen_idx];
    let fraction = addr as f32 / W_PIXELS as f32;
    let fraction_z = pixel_z as f32 / W_PIXELS as f32;
    let p_o = glam::Vec3::from(screen.points[0]);
    let p_z = glam::Vec3::from(screen.points[1]);
    let p_y = glam::Vec3::from(screen.points[3]);
    let v_oz = p_z - p_o;
    let v_oy = p_y - p_o;
    let p_xy = p_o + v_oy * fraction + v_oz * fraction_z;

    let v = glam::Vec4::new(p_xy.x, p_xy.y, p_xy.z, 1.0);
    let led = v;
    let p_view = mat * v;
    ((p_view.x, p_view.y, p_view.z), (led.x, led.y, led.z))
}

fn closest_len(line: &geo::Line<f32>, p: &geo::Point<f32>) -> f32 {
    let close_p = match line.closest_point(p) {
        geo::Closest::Intersection(point) => point,
        geo::Closest::SinglePoint(point) => point,
        geo::Closest::Indeterminate => unreachable!(),
    };
    close_p.euclidean_distance(p)
}

fn mirror_mat4(angle_f: f32) -> glam::Mat4 {
    let sin = -angle_f.sin();
    let cos = -angle_f.cos();
    let sin2 = sin * sin;
    let cos2 = cos * cos;
    let sin_cos = sin * cos;
    let mat_mir = glam::Mat4::from_cols(
        glam::Vec4::new(sin2 - cos2, -2.0 * sin_cos, 0., 0.),
        glam::Vec4::new(-2.0 * sin_cos, cos2 - sin2, 0., 0.),
        glam::Vec4::new(0., 0., 1., 0.),
        glam::Vec4::new(2.0 * MIRROR_OFFSET * cos, 2.0 * MIRROR_OFFSET * sin, 0., 1.),
    );
    let mat_ratate_x = glam::Mat4::from_cols(
        glam::Vec4::new(1.0, 0.0, 0.0, 0.),
        glam::Vec4::new(0.0, MAT_ROTATE_X.x_axis.x, MAT_ROTATE_X.x_axis.y, 0.),
        glam::Vec4::new(0.0, MAT_ROTATE_X.y_axis.x, MAT_ROTATE_X.y_axis.y, 0.),
        glam::Vec4::new(0.0, 0., 0., 1.),
    );
    let ro_rev = MAT_ROTATE_X_REV;
    let mat_ratate_x_rev = glam::Mat4::from_cols(
        glam::Vec4::new(1.0, 0.0, 0.0, 0.),
        glam::Vec4::new(0.0, ro_rev.x_axis.x, ro_rev.x_axis.y, 0.),
        glam::Vec4::new(0.0, ro_rev.y_axis.x, ro_rev.y_axis.y, 0.),
        glam::Vec4::new(0.0, 0., 0., 1.),
    );
    let mat = mat_ratate_x_rev * mat_mir * mat_ratate_x;
    mat
}

pub fn mirror_points_f(angle_f: f32, points: &[(f32, f32, f32)]) -> Vec<(f32, f32, f32)> {
    let mat = mirror_mat4(angle_f);
    points
        .iter()
        .map(|v| {
            let v = glam::Vec4::new(v.0, v.1, v.2, 1.0);
            let v = mat * v;
            (v.x, v.y, v.z)
        })
        .collect()
}

pub fn mirror_points(angle: u32, points: &[(f32, f32, f32)]) -> Vec<(f32, f32, f32)> {
    let angle_f = angle_to_v(angle);
    mirror_points_f(angle_f, points)
}

pub struct Codec {
    xy_arrs: [PixelXYArr; NUM_SCREENS],
    mat_map: BTreeMap<u32, glam::Mat4>,
}

impl Codec {
    // TODO map screens to image and fill tthe xy_arr
    pub fn new() -> Self {
        // 初始化坐标map key是xyz虚像自己的相对坐标
        let mut xy_arrs = [PixelXYArr::new(), PixelXYArr::new(), PixelXYArr::new()];
        for _x in 0..W_PIXELS {
            let mut line = vec![];
            line.extend(
                std::iter::repeat(())
                    .take(W_PIXELS)
                    .map(|_| [None; H_PIXELS]),
            );
            xy_arrs[0].push(line.clone());
            xy_arrs[1].push(line.clone());
            xy_arrs[2].push(line);
        }
        let mut mat_map = BTreeMap::new();
        let screen_metas: Vec<_> = SCREENS
            .iter()
            .map(|screen| {
                let fraction = 1f32 / W_PIXELS as f32;
                let fraction_z = 1f32 / W_PIXELS as f32;
                let p_o = glam::Vec3::from(screen.points[0]);
                let p_z = glam::Vec3::from(screen.points[1]);
                let p_y = glam::Vec3::from(screen.points[3]);
                let v_oz = p_z - p_o;
                let v_oy = p_y - p_o;
                let v_oa = v_oz * fraction_z;
                let v_ob = v_oy * fraction;

                let p_o = glam::Vec4::new(p_o.x, p_o.y, p_o.z, 1.0);
                let v_oa = glam::Vec4::new(v_oa.x, v_oa.y, v_oa.z, 1.0);
                let v_ob = glam::Vec4::new(v_ob.x, v_ob.y, v_ob.z, 1.0);
                log::info!("p_o {p_o:?} v_oa {v_oa:?} v_ob {v_ob:?}");
                (screen, p_o, v_oa, v_ob)
            })
            .collect();
        for angle in 0..TOTAL_ANGLES {
            let angle = angle as u32;
            let angle_f = angle_to_v(angle);
            let mat = mirror_mat4(angle_f);
            mat_map.insert(angle, mat);

            // 虚像的中心
            let center = mat * *V_IMG_CENTER_CORD;
            // 虚像对应实际的和屏幕接触的中心
            let center_xy = geo::Point::new(center.x, center.y);

            let dbg = angle == (TOTAL_ANGLES as u32 / 4);
            // Vec4(0.0, 1.0, 1.0, 1.0) v_oa Vec4(0.0, 0.0, 0.03125, 1.0) v_ob Vec4(0.0, 0.03125, 0.0, 1.0)

            for (screen_idx, (screen, p_o, v_oa, v_ob)) in
                screen_metas.clone().into_iter().enumerate()
            {
                // 过滤掉太远的角度 减小计算量
                let xy_line = geo::Line::new(
                    (screen.points[0].0, screen.points[0].1),
                    (screen.points[3].0, screen.points[3].1),
                );
                let closest_len = closest_len(&xy_line, &center_xy);
                // log::info!("angle {angle} closest_len {closest_len}");
                if closest_len > (2f32 * SCREEN_ZOOM).sqrt() {
                    continue;
                }

                let v_o = mat * glam::Vec4::new(0.0, 0.0, 0.0, 1.0);
                if dbg {
                    log::info!("angle_f {angle_f}");
                    log::info!("mat {mat}");
                }

                // 把斜着放的屏幕的向量映射到虚像空间 实际上是二维坐标变换
                let p_o = mat * p_o;
                let v_oa = mat * v_oa;
                let v_ob = mat * v_ob;
                if dbg {
                    log::info!("p_o {p_o:?} v_oa {v_oa:?} v_ob {v_ob:?}");
                }
                let v_oa = v_oa - v_o;
                let v_ob = v_ob - v_o;
                if dbg {
                    log::info!("new v_o {v_o} v_oa {v_oa:?} v_ob {v_ob:?}");
                }
                // 计算屏幕上每一个点对应虚像自己坐标的位置
                for i in 0..W_PIXELS {
                    for j in 0..W_PIXELS {
                        let p = p_o + v_oa * (i as f32) + v_ob * (j as f32);
                        let dbg = dbg && (i < 10 && j < 10);
                        if dbg {
                            log::info!("i {i} j {j} p {p}");
                        }
                        let pz = V_IMG_CORD.z - p.z;
                        let px = p.x;
                        let py = p.y - V_IMG_CORD.y + 1. * SCREEN_ZOOM;
                        let Some((x, y, z)) = v3_2_pixel(px, py, pz) else {
                            continue;
                        };
                        if dbg {
                            log::info!("x {x} y {y} z {z}");
                        }
                        let z_point = PixelZInfo {
                            angle,
                            pixel: z,
                            is_borrowed: false,
                            screen_pixel: ScreenPixel {
                                idx: screen_idx,
                                addr: j as u32,
                                pixel: i as u32,
                            },
                        };
                        xy_arrs[screen_idx][x as usize][y as usize][z as usize] = Some(z_point);
                    }
                }
            }
        }
        // TODO borrow value from other coloums
        // not good

        // TODO no clone
        // let xy_arr_cl = xy_arr.clone();
        // let borrow = |x: usize, y: usize, z: usize, z_info: &mut Option<PixelZInfo>| match xy_arr_cl
        //     .get(x)
        //     .and_then(|v| v.get(y))
        //     .and_then(|v| v[z])
        //     .as_ref()
        // {
        //     Some(v) => {
        //         if v.is_borrowed {
        //             return false;
        //         }
        //         let mut v = v.clone();
        //         v.is_borrowed = true;
        //         *z_info = Some(v);
        //         return true;
        //     }
        //     None => return false,
        // };

        // for (x, y_arr) in xy_arr.iter_mut().enumerate() {
        //     for (y, z_arr) in y_arr.iter_mut().enumerate() {
        //         for (z, z_info) in z_arr.iter_mut().enumerate() {
        //             if z_info.is_some() {
        //                 continue;
        //             }

        //             if borrow(x + 1, y, z, z_info) {
        //                 continue;
        //             }
        //             if borrow(x, y + 1, z, z_info) {
        //                 continue;
        //             }
        //             if x > 1 {
        //                 if borrow(x - 1, y, z, z_info) {
        //                     continue;
        //                 }
        //             }
        //             if y > 1 {
        //                 if borrow(x, y - 1, z, z_info) {
        //                     continue;
        //                 }
        //             }
        //         }
        //     }
        // }
        Self { xy_arrs, mat_map }
    }

    pub fn encode(
        &self,
        pixel_surface: &PixelSurface,
        pixel_offset: i32,
        optimze_speed_for_mbi5264: bool,
    ) -> AngleMap {
        let mut angle_map: BTreeMap<
            u32,
            [BTreeMap<ScreenLineAddr, ScreenLinePixels>; NUM_SCREENS],
        > = BTreeMap::new();
        for &(x, y, (z, color)) in pixel_surface {
            for screen_idx in 0..NUM_SCREENS {
                let z_info_list = &self.xy_arrs[screen_idx][x as usize][y as usize];
                // fix z offset
                // TODO find the reason for offset
                // let z = if z > 1 { z - 1 } else { z };
                let Some(z_info) = z_info_list.get(z as usize).and_then(|v| *v) else {
                    continue;
                };
                let entry = angle_map.entry(z_info.angle).or_default();
                let addr = ScreenLineAddr {
                    screen_idx: z_info.screen_pixel.idx,
                    addr: z_info.screen_pixel.addr,
                };
                let line_pixels = entry[screen_idx].entry(addr).or_default();
                let pixel_idx = z_info.screen_pixel.pixel as usize;
                if let Some(c) = line_pixels.pixels.get_mut(pixel_idx) {
                    *c = Some(color);
                } else {
                    panic!("{x}, {y}, {z}, pixel_idx {pixel_idx}");
                }
            }
        }
        angle_map
            .into_iter()
            .map(|(k, addr_maps)| {
                (
                    k,
                    addr_maps.map(|addr_map| {
                        parse_addr_map(addr_map, pixel_offset, optimze_speed_for_mbi5264)
                    }),
                )
            })
            .collect()
    }
    pub fn decode(&self, angle: u32, lines: &[ScreenLine]) -> (FloatSurface, FloatSurface) {
        let mut view_surface = FloatSurface::default();
        let mut led_surface = FloatSurface::default();
        for ScreenLine {
            screen_idx,
            addr,
            pixels,
        } in lines
        {
            for (idx, pixel) in pixels.into_iter().enumerate() {
                let Some(_pixel) = pixel else { continue };
                let pixel_z = idx as u32;
                let mat = self.mat_map.get(&angle).unwrap();
                let (view, led) = cacl_view_point(*mat, *screen_idx, *addr, pixel_z);
                view_surface.push(view);
                led_surface.push(led);
            }
        }
        (view_surface, led_surface)
    }
    pub fn decode_all(&self, angle_map: AngleMap) -> (FloatSurface, FloatSurface) {
        let mut view_surface = FloatSurface::default();
        let mut led_surface = FloatSurface::default();
        for (angle, lines_arr) in angle_map {
            for lines in lines_arr {
                let (view, led) = self.decode(angle, lines.as_slice());
                view_surface.extend(view);
                led_surface.extend(led);
            }
        }
        (view_surface, led_surface)
    }
}

fn parse_addr_map(
    mut addr_map: BTreeMap<ScreenLineAddr, ScreenLinePixels>,
    pixel_offset: i32,
    optimze_speed_for_mbi5264: bool,
) -> Vec<ScreenLine> {
    let mut pixels_info: [Option<([u8; 4], ScreenLineAddr)>; W_PIXELS] = [None; W_PIXELS];
    for (addr, line) in addr_map.iter_mut() {
        for (color, pixel_info) in line.pixels.iter_mut().zip(&mut pixels_info) {
            let Some(rgbh) = color.as_ref() else {
                continue;
            };
            let [r, g, b, _a] = rgbh.to_ne_bytes();
            match pixel_info {
                Some(_rgbh) => {
                    *color = None;
                }
                None => {
                    *pixel_info = Some(([r, g, b, addr.addr as u8], *addr));
                }
            }
        }
    }
    if optimze_speed_for_mbi5264 {
        for i in 0..64usize {
            let region0 = i;
            let region1 = i + 64;
            let region2 = i + 128;
            let regions = [region1, region0, region2];
            let mut non_empty_h: Option<u8> = None;
            for region in regions {
                let Some(pixel_info) = pixels_info[region] else {
                    continue;
                };
                if pixel_info.0[..3] == [0; 3] {
                    continue;
                }
                non_empty_h = Some(pixel_info.0[3]);
                break;
            }
            let Some(non_empty_h) = non_empty_h else {
                continue;
            };
            let non_empty_h_mod = non_empty_h % 16;
            for region in regions {
                let Some((rgbh, line_addr)) = &pixels_info[region] else {
                    continue;
                };
                let h = rgbh[3];
                let h_mod = h % 16;
                if h_mod == non_empty_h_mod {
                    continue;
                }
                let mut deta = non_empty_h_mod as i16 - h_mod as i16;
                if deta.abs() > 8 {
                    let try_deta = if deta > 0 { deta - 16 } else { deta + 16 };
                    let try_h = h as i16 + try_deta as i16;
                    if try_h >= 0 && try_h <= 143 {
                        deta = try_deta;
                    }
                }
                let new_h = (h as i16 + deta) as u8;
                let line_pixels = addr_map.get_mut(&line_addr).unwrap();
                let pixel = line_pixels.pixels[region].take().unwrap();
                let mut line_addr = *line_addr;
                line_addr.addr = new_h as u32;
                let entry = addr_map.entry(line_addr).or_default();
                entry.pixels[region] = Some(pixel);
            }
        }
    }

    addr_map
        .into_iter()
        .map(|(k, mut v)| {
            if pixel_offset > 0 {
                let offset = pixel_offset as usize;
                v.pixels.rotate_right(offset);
                v.pixels.iter_mut().take(offset).for_each(|v| {
                    v.take();
                });
            } else if pixel_offset < 0 {
                let offset = (-pixel_offset) as usize;
                v.pixels.rotate_left(offset);
                v.pixels.iter_mut().rev().take(offset).for_each(|v| {
                    v.take();
                });
            }
            ScreenLine {
                screen_idx: k.screen_idx,
                addr: k.addr,
                pixels: v.pixels,
            }
        })
        .collect::<Vec<_>>()
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_mirror_mat() {
        let angle = (90f32).to_radians();
        let mat = mirror_mat4(angle);
        let p = glam::Vec4::new(0.0, 1.0, 1.0, 1.0);
        let p = mat * p;
        // p [-0.00000024726896, -2.9999998, -2.9999998, 1]
        println!("p {p}");
    }
}
