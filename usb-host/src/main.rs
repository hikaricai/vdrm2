use rusb::UsbContext;
use std::{io::Cursor, time::Duration};
const BULK_OUT_EP: u8 = 0x01;
const BULK_IN_EP: u8 = 0x81;

fn usb_data_to_buf(usb_data: mbi5264_common::UsbData) -> Vec<u8> {
    let mut buf = vec![];
    let hdr = usb_data.hdr.to_buf();
    buf.extend_from_slice(hdr);
    buf.extend_from_slice(usb_data.payload);
    buf
}
fn send_bulk(usb_data: mbi5264_common::UsbData) {
    let buf = usb_data_to_buf(usb_data);
    println!("buf len {}", buf.len());
    // list_devices().unwrap();
    let mut context = rusb::Context::new().unwrap();
    let mut handle = context.open_device_with_vid_pid(0xc0de, 0xcafe).unwrap();
    let mut dev = handle.device();
    // handle.set_active_configuration();
    handle.claim_interface(0).unwrap();
    // handle.set_alternate_setting();

    let begin = std::time::Instant::now();
    let mut total_send = 0usize;
    let mut cnt = 0;
    loop {
        let size = handle
            .write_bulk(
                BULK_OUT_EP,
                &buf[total_send..],
                Duration::from_millis(1_000),
            )
            .unwrap();
        total_send += size;
        if size != buf.len() {
            println!("real sent {size}");
        }

        let time = std::time::Instant::now().duration_since(begin).as_millis();
        if total_send >= buf.len() {
            total_send = 0;
            cnt += 1;
        }
        if time >= 1000 {
            println!("{}ms total {} bytes", time, cnt * buf.len());
            break;
        }
    }
}

fn main() {
    println!("Hello, world!");
    let image = image::open("asserts/testcard_rgba.png").unwrap();
    let crop_img = image.crop_imm(0, 0, 200, 50);
    let crop_rgba = crop_img.as_rgba8().unwrap();
    let mut qoi_buf: Vec<u8> = vec![];
    crop_rgba
        .write_to(&mut Cursor::new(&mut qoi_buf), image::ImageFormat::Qoi)
        .unwrap();
    let hdr = mbi5264_common::UsbDataHead {
        cmd: mbi5264_common::UsbCmd::QOI,
        payload_len: qoi_buf.len() as u32,
    };
    let usb_data = mbi5264_common::UsbData {
        hdr: &hdr,
        payload: qoi_buf.as_slice(),
    };
    send_bulk(usb_data);
}
