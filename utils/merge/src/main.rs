fn main() {
    let mut imgs = vec![];
    for path in std::env::args().skip(1) {
        println!("load {path}");
        let img = std::fs::read(&path).unwrap();
        let header = mbi5264_common::preencoded::Header::parse(&img)
            .unwrap_or_else(|| panic!("{path} is not a PIO2 image"));
        assert!(header.frame_count > 0, "{path} contains no frames");
        imgs.push(img);
    }
    assert!(!imgs.is_empty(), "at least one PIO2 image is required");
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
