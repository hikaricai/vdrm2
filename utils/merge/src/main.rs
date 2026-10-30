fn main() {
    let mut imgs = vec![];
    for path in std::env::args().skip(1) {
        println!("load {path}");
        let img = std::fs::read(&path).unwrap();
        assert!(img.len() >= 8, "{path} is missing its RGBH image header");
        let angle_count = u32::from_le_bytes(img[4..8].try_into().unwrap()) as usize;
        assert!(angle_count > 0, "{path} contains no angles");
        let payload_len = img.len() - 8;
        let angle_size = std::mem::size_of::<mbi5264_common::AngleImage>();
        assert!(
            payload_len % angle_size == 0 && payload_len / angle_size == angle_count,
            "{path} is not a compatible RGBH image"
        );
        imgs.push(img);
    }
    assert!(!imgs.is_empty(), "at least one RGBH image is required");
    let mut buf: Vec<u8> = vec![];

    let len = imgs.len() as u32;
    buf.extend_from_slice(&len.to_le_bytes());
    let mut offset: u32 = 4 + 4 * len;
    for img in &imgs {
        buf.extend_from_slice(&offset.to_le_bytes());
        offset += img.len() as u32;
    }
    for img in &imgs {
        buf.extend_from_slice(img);
    }
    std::fs::write("./batchimgs.bin", buf).unwrap();
}
