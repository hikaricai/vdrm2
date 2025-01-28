use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::Timer;
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    // USBCTRL_IRQ => UsbInterruptHandler;
});

// usb interrupt is running on core1
struct UsbInterruptHandler {}
impl embassy_rp::interrupt::typelevel::Handler<<USB as embassy_rp::usb::Instance>::Interrupt>
    for UsbInterruptHandler
{
    unsafe fn on_interrupt() {
        let core_num = embassy_rp::pac::SIO.cpuid().read();
        defmt::info!("usb interrupt core {}", core_num);
        InterruptHandler::<USB>::on_interrupt();
    }
}

pub static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();
pub fn run(usb: USB, safe_sender: crate::SafeSender) -> ! {
    let executor1 = EXECUTOR.init(Executor::new());
    executor1.run(move |spawner| {
        spawner.spawn(core1_task(usb, safe_sender)).unwrap();
    });
}

#[embassy_executor::task]
async fn core1_task(usb: USB, mut safe_sender: crate::SafeSender) {
    let core_num = embassy_rp::pac::SIO.cpuid().read();
    defmt::info!("Hello from core {}", core_num);
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
            defmt::info!("Connected");
            let buf = safe_sender.sender.send().await;
            buf[0] = [233; 4];
            safe_sender.sender.send_done();
            loop {
                let mut data = [0; 64];
                match read_ep.read(&mut data).await {
                    Ok(n) => {
                        defmt::info!("Got bulk: {:a}", data[..n]);
                        // Echo back to the host:
                        write_ep.write(&data[..n]).await.ok();
                    }
                    Err(_) => break,
                }
            }
            defmt::info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}
