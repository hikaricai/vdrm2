mod mbi5264;
use flexi_logger::LogSpecBuilder;
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{DownChannel, Rtt, UpChannel};
use probe_rs::{Permissions, Session};
use std::time::Duration;

struct RttHost {
    session: Session,
    rtt: Rtt,
    terminal_up: UpChannel,
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

        let terminal_up = rtt.up_channels().take(0).unwrap();
        let up = rtt.up_channels().take(1).unwrap();
        let down = rtt.down_channels().take(0).unwrap();

        Self {
            session,
            rtt,
            terminal_up,
            up,
            down,
        }
    }
}

impl RttHost {
    fn try_read_log(&mut self) {
        let mut core = self.session.core(0).unwrap();
        let mut buf = [0; 1024];
        let size = self.terminal_up.read(&mut core, &mut buf[..]).unwrap();
        if size == 0 {
            return;
        }
        log::info!("log: {}", String::from_utf8_lossy(&buf[0..size]));
    }

    fn try_read(&mut self) -> anyhow::Result<usize> {
        self.try_read_log();
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
        self.try_read_log();
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

    fn block_write_cmd(&mut self, cmd: mbi5264::Command) {
        let buf = cmd.to_buf();
        // log::info!("write cmd {}, buf[0] {}", cmd.cmd, buf[0]);
        self.block_write(buf).unwrap();
        let size = self.block_read().unwrap();
        // log::info!("block read {}", size);
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
    if let Ok(size) = rtt_host.try_read() {
        log::info!("drain up {}", size);
    };
    let mut cmd = 0u8;
    loop {
        cmd += 1;
        rtt_host.block_write_cmd(mbi5264::Command::new_cmd(cmd, cmd as u16));
        if cmd == 143 - 8 {
            cmd = 0
        }
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
