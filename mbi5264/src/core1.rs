#![allow(unused)]
use embassy_executor::Executor;
use embassy_executor::InterruptExecutor;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt;
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::CORE1;
use embassy_rp::peripherals::USB;
use embassy_rp::usb;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_usb::driver::{Endpoint, EndpointOut};
use embassy_usb::{Builder, Config};
use static_cell::ConstStaticCell;
use static_cell::StaticCell;
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    // USBCTRL_IRQ => UsbInterruptHandler;
});

static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_MED.on_interrupt()
}

// const QOI_BUF_SIZE: usize = crate::IMG_SIZE * 4;
// at least compress to 1/4
const QOI_BUF_SIZE: usize = mbi5264_common::QOI_BUF_SIZE;
type UsbDataBuf = [u8; QOI_BUF_SIZE];
static QOI_BUF: ConstStaticCell<[UsbDataBuf; 2]> = ConstStaticCell::new([[0; QOI_BUF_SIZE]; 2]);
static QOI_CHANNEL: StaticCell<zerocopy_channel::Channel<'_, CriticalSectionRawMutex, UsbDataBuf>> =
    StaticCell::new();

// usb interrupt is running on core1
// struct UsbInterruptHandler {}
// impl embassy_rp::interrupt::typelevel::Handler<<USB as embassy_rp::usb::Instance>::Interrupt>
//     for UsbInterruptHandler
// {
//     unsafe fn on_interrupt() {
//         let core_num = embassy_rp::pac::SIO.cpuid().read();
//         rtt_target::rprintln!("usb interrupt core {}", core_num);
//         InterruptHandler::<USB>::on_interrupt();
//     }
// }

pub static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

pub fn spawn_usb_core1(cpu1: CORE1, usb: USB, safe_sender: crate::SafeSender<crate::ImageBuffer>) {
    spawn_core1(
        cpu1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            run(usb, safe_sender);
        },
    );
}

pub fn run(usb: USB, safe_sender: crate::SafeSender<crate::ImageBuffer>) -> ! {
    let qoi_buf = QOI_BUF.take();
    let channel = QOI_CHANNEL.init(zerocopy_channel::Channel::new(qoi_buf));
    let (qoi_tx, qoi_rx) = channel.split();
    let qoi_tx = crate::SafeSender { sender: qoi_tx };
    interrupt::SWI_IRQ_0.set_priority(interrupt::Priority::P3);
    let spawner1 = EXECUTOR_MED.start(interrupt::SWI_IRQ_0);
    spawner1.spawn(core1_usb_task(usb, qoi_tx)).unwrap();
    let executor1 = EXECUTOR.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(decode_qoi(qoi_rx, safe_sender)).unwrap();
    });
}

#[embassy_executor::task]
async fn decode_qoi(
    mut usb_data_rx: zerocopy_channel::Receiver<'static, CriticalSectionRawMutex, UsbDataBuf>,
    mut safe_sender: crate::SafeSender<crate::ImageBuffer>,
) {
    let mut cnt = 0_u8;
    loop {
        // rtt_target::rprintln!("decode qoi");
        let send_buf = safe_sender.sender.send().await;
        // rtt_target::rprintln!("acquire qoi send buffer");
        // let raw_buf: &mut [u8; crate::IMG_SIZE * 4] =
        //     unsafe { core::mem::transmute(send_buf.as_ptr()) };
        // loop {
        //     let usb_data_buf = usb_data_rx.receive().await;
        //     let Some(usb_data) = mbi5264_common::UsbData::ref_from_buf(usb_data_buf) else {
        //         usb_data_rx.receive_done();
        //         continue;
        //     };
        //     let payload_len = usb_data.hdr.payload_len;
        //     rtt_target::rprintln!("usb data {}", payload_len);
        //     let ret = qoi::decode_to_buf(raw_buf.as_mut(), usb_data.payload);
        //     usb_data_rx.receive_done();
        //     rtt_target::rprintln!("decode qoi ret {}", ret.is_ok());
        //     if ret.is_ok() {
        //         break;
        //     }
        // }
        for p in send_buf.iter_mut() {
            // FIXME
            // *p = [255, 255, 255, cnt];
            // p[3] = core::cmp::min(p[3], 143);
        }
        safe_sender.sender.send_done();
        cnt += 1;
        cnt %= 144;
    }
}

#[embassy_executor::task]
async fn core1_usb_task(usb: USB, mut qoi_tx: crate::SafeSender<UsbDataBuf>) {
    let core_num = embassy_rp::pac::SIO.cpuid().read();
    rtt_target::rprintln!("Hello from core {}", core_num);
    let driver = Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB raw example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // that uses our custom handler.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let mut read_ep = alt.endpoint_bulk_out(64);
    let mut write_ep = alt.endpoint_bulk_in(64);
    drop(function);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            read_ep.wait_enabled().await;
            // rtt_target::rprintln!("Connected");
            loop {
                if !handle_usb(&mut read_ep, &mut qoi_tx).await {
                    break;
                };
            }
            // rtt_target::rprintln!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

async fn handle_usb(
    read_ep: &mut usb::Endpoint<'_, USB, usb::Out>,
    qoi_tx: &mut crate::SafeSender<UsbDataBuf>,
) -> bool {
    let buf = qoi_tx.sender.send().await.as_mut();
    let mut total_len = 0;
    loop {
        match read_ep.read(&mut buf[total_len..]).await {
            Ok(n) => {
                // rtt_target::rprintln!("Got bulk {}", n);
                // Echo back to the host:
                // write_ep.write(&data[..n]).await.ok();
                total_len += n;
            }
            Err(e) => {
                // rtt_target::rprintln!("Got bulk err {}", e);
                match e {
                    embassy_usb::driver::EndpointError::BufferOverflow => {
                        return true;
                    }
                    embassy_usb::driver::EndpointError::Disabled => {
                        return false;
                    }
                };
            }
        }
        // if buf.len() > 8 {
        //     let hdr: &mbi5264_common::UsbDataHead = unsafe { core::mem::transmute(buf.as_ptr()) };
        //     // let cmd = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        //     // let len = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        //     // rtt_target::rprintln!("decode info {} {}", cmd, len);
        //     let cmd = hdr.cmd as u32;
        //     let len = hdr.payload_len;
        //     rtt_target::rprintln!("decode info {} {}", cmd, len);
        // }

        let Some(usb_data) = mbi5264_common::UsbData::ref_from_buf(&buf[0..total_len]) else {
            continue;
        };
        // rtt_target::rprintln!("recv usb_data succ");
        match usb_data.hdr.cmd {
            mbi5264_common::UsbCmd::QOI => {}
        }
        break;
    }
    qoi_tx.sender.send_done();
    true
}
