mod decode;
mod dsl;

use anyhow::{ensure, Context, Result};
use clap::{Parser, ValueEnum};
use decode::{Decoder, Event, Lane};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Clone, Copy, ValueEnum)]
enum Format {
    Text,
    Jsonl,
}

#[derive(Parser)]
#[command(
    version,
    about = "Decode ICND2069 CLK/LAT/RGB commands from a DSView v3 .dsl capture"
)]
struct Args {
    /// DSView .dsl input file
    input: PathBuf,
    #[arg(long, default_value_t = 0)]
    clk: usize,
    #[arg(long, default_value_t = 1)]
    lat: usize,
    #[arg(long, alias = "r", default_value_t = 2)]
    red: usize,
    #[arg(long, alias = "g", default_value_t = 5)]
    green: usize,
    #[arg(long, alias = "b", default_value_t = 6)]
    blue: usize,
    /// Known cascade length; select final N words for DATA_LATCH/WR_CFG/RD_CFG.
    /// Without it, retain all right-aligned candidate words.
    #[arg(long)]
    chips: Option<usize>,
    #[arg(long, value_enum, default_value = "text")]
    format: Format,
    /// Output file (default: stdout). Existing files are not overwritten.
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Also save the machine-readable summary to a new JSON file
    #[arg(long)]
    summary_json: Option<PathBuf>,
}

#[derive(Default, Serialize)]
struct Summary {
    commands: usize,
    incomplete_pulses: usize,
    warning_events: usize,
    dclk_rising_edges: usize,
    trailing_unlatched_clocks: usize,
    command_counts: BTreeMap<String, usize>,
    payload_window_clocks: BTreeMap<usize, usize>,
    /// Key is lane:address=value, count is individual serial words, including cascaded copies.
    register_write_words: BTreeMap<String, usize>,
    vsync_samples: Vec<usize>,
    vsync_period_us: Vec<f64>,
    /// First/last frame may be partial; this is an observation, not a completeness claim.
    data_latches_by_frame: BTreeMap<usize, usize>,
}

impl Summary {
    fn observe(&mut self, event: &Event, rate: f64) {
        if event.complete {
            self.commands += 1;
        } else {
            self.incomplete_pulses += 1;
        }
        self.warning_events += usize::from(!event.warnings.is_empty());
        *self.command_counts.entry(event.command.into()).or_default() += 1;
        if matches!(event.command, "DATA_LATCH" | "WR_CFG" | "RD_CFG") {
            *self
                .payload_window_clocks
                .entry(event.shift_clocks)
                .or_default() += 1;
        }
        if event.command == "DATA_LATCH" {
            *self.data_latches_by_frame.entry(event.frame).or_default() += 1;
        }
        if event.command == "VSYNC" {
            if let Some(last) = self.vsync_samples.last() {
                self.vsync_period_us
                    .push((event.end_sample - last) as f64 * 1e6 / rate);
            }
            self.vsync_samples.push(event.end_sample);
        }
        if event.command == "WR_CFG" {
            for (name, lane) in [("R", &event.red), ("G", &event.green), ("B", &event.blue)] {
                for reg in &lane.registers {
                    let key = format!(
                        "{name}:{}={}",
                        reg.address_hex,
                        reg.value_hex.as_deref().unwrap_or("?")
                    );
                    *self.register_write_words.entry(key).or_default() += 1;
                }
            }
        }
    }
}

fn runs(words: &[String]) -> String {
    let mut parts = Vec::new();
    let mut start = 0;
    while start < words.len() {
        let mut end = start + 1;
        while end < words.len() && words[end] == words[start] {
            end += 1;
        }
        parts.push(if end - start > 1 {
            format!("{}*{}", words[start], end - start)
        } else {
            words[start].clone()
        });
        start = end;
    }
    parts.join(" ")
}

fn write_lane(out: &mut dyn Write, name: &str, lane: &Lane) -> Result<()> {
    writeln!(
        out,
        "  {name}: [{}]  prefix={}  tail16={}",
        runs(&lane.words_hex),
        if lane.prefix_bits.is_empty() {
            "-"
        } else {
            &lane.prefix_bits
        },
        lane.tail16_hex.as_deref().unwrap_or("incomplete")
    )?;
    if !lane.registers.is_empty() {
        let regs: Vec<_> = lane
            .registers
            .iter()
            .map(|r| match &r.value_hex {
                Some(v) => format!("{}={v}", r.address_hex),
                None => format!("read({})", r.address_hex),
            })
            .collect();
        writeln!(out, "     registers: [{}]", runs(&regs))?;
    }
    Ok(())
}

fn write_event(out: &mut dyn Write, format: Format, event: &Event) -> Result<()> {
    match format {
        Format::Jsonl => {
            serde_json::to_writer(
                &mut *out,
                &serde_json::json!({"type": "command", "event": event}),
            )?;
            writeln!(out)?;
        }
        Format::Text => {
            write!(
                out,
                "#{:04} {:12.3} us  {:11} LAT={} CLK={} samples={}..{} shift_start={} frame={}",
                event.index,
                event.time_us,
                event.command,
                event.lat_clocks,
                event.shift_clocks,
                event.lat_start_sample,
                event.end_sample,
                event.shift_start_sample,
                event.frame
            )?;
            if let (Some(index), Some(line), Some(out_channel)) =
                (event.data_index, event.line_1based, event.out)
            {
                write!(out, " data={index} line={line} OUT{out_channel}")?;
            }
            writeln!(out, " ({})", event.data_kind)?;
            write_lane(out, "R", &event.red)?;
            write_lane(out, "G", &event.green)?;
            write_lane(out, "B", &event.blue)?;
            for warning in &event.warnings {
                writeln!(out, "  WARNING: {warning}")?;
            }
        }
    }
    Ok(())
}

fn new_output(path: &Path) -> Result<File> {
    File::options()
        .write(true)
        .create_new(true)
        .open(path)
        .with_context(|| {
            format!(
                "cannot create {} (choose a new path if it exists)",
                path.display()
            )
        })
}

fn run(args: Args) -> Result<()> {
    ensure!(args.chips != Some(0), "--chips must be greater than zero");
    let channels = [args.clk, args.lat, args.red, args.green, args.blue];
    let capture = dsl::open(&args.input, channels)?;
    let meta = &capture.metadata;
    // Check all destinations before creating either file.
    for path in [&args.output, &args.summary_json].into_iter().flatten() {
        ensure!(
            !path.try_exists()?,
            "output already exists: {}",
            path.display()
        );
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        ensure!(
            parent.is_dir(),
            "output directory does not exist: {}",
            parent.display()
        );
    }
    if let (Some(a), Some(b)) = (&args.output, &args.summary_json) {
        let absolute = |p: &Path| -> Result<PathBuf> {
            let parent = p
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(Path::new("."));
            Ok(fs::canonicalize(parent)?.join(p.file_name().context("output needs a filename")?))
        };
        ensure!(
            absolute(a)? != absolute(b)?,
            "output and summary must be different files"
        );
    }
    let mut out: Box<dyn Write> = match &args.output {
        Some(path) => Box::new(BufWriter::new(new_output(path)?)),
        None => Box::new(BufWriter::new(io::stdout().lock())),
    };
    match args.format {
        Format::Jsonl => {
            serde_json::to_writer(
                &mut out,
                &serde_json::json!({"type": "metadata", "capture": meta, "chips": args.chips}),
            )?;
            writeln!(out)?;
        }
        Format::Text => {
            writeln!(
                out,
                "ICND2069 | {} samples @ {} Hz | {:.3} us | CLK={} LAT={} R={} G={} B={}",
                meta.samples,
                meta.samplerate_hz,
                meta.samples as f64 * 1e6 / meta.samplerate_hz,
                args.clk,
                args.lat,
                args.red,
                args.green,
                args.blue
            )?;
            writeln!(
                out,
                "HEX words are MSB-first, in transmit order; WORD*N repeats a word N times."
            )?;
            writeln!(out, "Words are aligned to the latch from the right; prefix contains ALL remaining leading bits.")?;
            writeln!(out, "Control commands show raw shift history, not semantic payload. Frame 0 has no observed VSYNC.")?;
            writeln!(
                out,
                "Cascade length: {}\n",
                args.chips
                    .map_or("unknown (candidate words)".into(), |n| n.to_string())
            )?;
        }
    }
    let mut decoder = Decoder::new(capture.sample(0), meta.samplerate_hz, args.chips);
    let mut summary = Summary::default();
    for sample in 1..meta.samples {
        if let Some(event) = decoder.push(sample, capture.sample(sample)) {
            summary.observe(&event, meta.samplerate_hz);
            write_event(&mut out, args.format, &event)?;
        }
    }
    if let Some(event) = decoder.finish(meta.samples) {
        summary.observe(&event, meta.samplerate_hz);
        write_event(&mut out, args.format, &event)?;
    }
    summary.dclk_rising_edges = decoder.total_clocks();
    summary.trailing_unlatched_clocks = decoder.trailing_clocks();
    match args.format {
        Format::Jsonl => {
            serde_json::to_writer(
                &mut out,
                &serde_json::json!({"type": "summary", "summary": summary}),
            )?;
            writeln!(out)?;
        }
        Format::Text => {
            writeln!(out, "\nSUMMARY")?;
            serde_json::to_writer_pretty(&mut out, &summary)?;
            writeln!(out)?;
        }
    }
    out.flush()?;
    if let Some(path) = &args.summary_json {
        let mut file = BufWriter::new(new_output(path)?);
        serde_json::to_writer_pretty(
            &mut file,
            &serde_json::json!({
                "capture": meta, "chips": args.chips, "summary": summary
            }),
        )?;
        writeln!(file)?;
        file.flush()?;
    }
    eprintln!(
        "Decoded {} commands, {} incomplete pulses, {} events with warnings; {} trailing unlatched clocks.",
        summary.commands, summary.incomplete_pulses, summary.warning_events, summary.trailing_unlatched_clocks
    );
    eprintln!("Counts: {:?}", summary.command_counts);
    Ok(())
}

fn main() -> std::process::ExitCode {
    match run(Args::parse()) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error)
            if error
                .downcast_ref::<io::Error>()
                .is_some_and(|e| e.kind() == io::ErrorKind::BrokenPipe) =>
        {
            std::process::ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: {error:#}");
            std::process::ExitCode::FAILURE
        }
    }
}
