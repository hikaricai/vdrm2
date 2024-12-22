mod mbi5264;
use flexi_logger::LogSpecBuilder;
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{DownChannel, Rtt, UpChannel};
use probe_rs::{Permissions, Session};
use std::time::Duration;

struct RttHost {
    session: Session,
    rtt: Rtt,
    up: UpChannel,
    down: DownChannel,
}

impl RttHost {
    fn open(elf_path: &str) -> Self {
        let lister = Lister::new();

        let probes = lister.list_all();
        log::info!("probes {probes:?}");
        let probe = probes[0].open().unwrap();
        let mut session = probe.attach("RP2040", Permissions::default()).unwrap();
        // session.resume_all_cores()?;
        let memory_map = session.target().memory_map.clone();
        // Select a core.
        let mut core = session.core(0).unwrap();

        // Attach to RTT
        log::info!("[begin] attach");
        let rtt_address = get_rtt_symbol_from_bytes(&elf_path).unwrap();
        log::info!("rtt_address {rtt_address:X?}");
        let mut rtt = Rtt::attach_region(
            &mut core,
            &memory_map,
            &probe_rs::rtt::ScanRegion::Exact(rtt_address),
        )
        .unwrap();
        log::info!("[end] attach");
        std::mem::drop(core);

        let up = rtt.up_channels().take(0).unwrap();
        let down = rtt.down_channels().take(0).unwrap();
        Self {
            session,
            rtt,
            up,
            down,
        }
    }
}

impl RttHost {
    fn try_read(&mut self) -> anyhow::Result<usize> {
        let mut core = self.session.core(0).unwrap();
        let mut buf = [0; 1024];
        let size = self.up.read(&mut core, &mut buf[..])?;
        Ok(size)
    }

    fn block_read(&mut self) -> anyhow::Result<usize> {
        loop {
            let size = self.try_read()?;
            if size != 0 {
                return Ok(size);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn try_write(&mut self, buf: &[u8]) -> anyhow::Result<usize> {
        let mut core = self.session.core(0).unwrap();
        let size = self.down.write(&mut core, buf)?;
        Ok(size)
    }

    fn block_write(&mut self, buf: &[u8]) -> anyhow::Result<usize> {
        loop {
            let size = self.try_write(buf)?;
            if size == buf.len() {
                return Ok(size);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

fn main() {
    flexi_logger::Logger::with(
        LogSpecBuilder::new()
            .default(log::LevelFilter::Error)
            .module("mbi5264_host", log::LevelFilter::Info)
            .build(),
    )
    .format(flexi_logger::colored_detailed_format)
    .start()
    .unwrap();
    log::info!("Hello, world!");
    let elf_path = std::env::args().nth(1).unwrap();
    let mut rtt_host = RttHost::open(&elf_path);

    let cmd = mbi5264::Transaction::new_cmd(1, 0x2333);
    let mut has_write = false;
    loop {
        std::thread::sleep(Duration::from_secs(1));
        // Read from a channel
        match rtt_host.try_read() {
            Ok(count) => {
                if count != 0 {
                    println!("Read {count}",);
                }
            }
            Err(e) => {
                println!("read e {e}");
            }
        };
        if has_write {
            continue;
        }

        // Write to a channel
        let w_buf = cmd.to_buf();
        match rtt_host.block_write(w_buf) {
            Ok(len) => {
                println!("write {len}");
            }
            Err(e) => {
                panic!("write e {e}");
            }
        }
        has_write = true;
    }
}

fn get_rtt_symbol_from_bytes(path: &str) -> Option<u64> {
    let bytes = std::fs::read(path).unwrap();
    if let Ok(binary) = goblin::elf::Elf::parse(&bytes) {
        for sym in &binary.syms {
            if binary.strtab.get_at(sym.st_name) == Some("_SEGGER_RTT") {
                return Some(sym.st_value);
            }
        }
    }
    None
}
