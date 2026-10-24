use anyhow::{bail, ensure, Context, Result};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::Read,
    path::Path,
};
use zip::ZipArchive;

#[derive(Debug, Serialize)]
pub struct Metadata {
    pub version: u32,
    pub samples: usize,
    pub samplerate_hz: f64,
    pub blocks: usize,
    pub channels: [usize; 5],
}

pub struct Capture {
    pub metadata: Metadata,
    // CLK, LAT, R, G, B. DSView stores time in ascending bit positions,
    // independently of the ICND serial word's MSB-first transmission order.
    planes: [Vec<u8>; 5],
}

impl Capture {
    pub fn sample(&self, index: usize) -> [u8; 5] {
        self.planes
            .each_ref()
            .map(|plane| (plane[index / 8] >> (index % 8)) & 1)
    }
}

fn rate(text: &str) -> Result<f64> {
    let split = text
        .find(|c: char| c.is_ascii_alphabetic())
        .unwrap_or(text.len());
    let value: f64 = text[..split].trim().parse().context("invalid samplerate")?;
    let factor = match text[split..].trim().to_ascii_lowercase().as_str() {
        "" | "hz" => 1.0,
        "khz" => 1e3,
        "mhz" => 1e6,
        "ghz" => 1e9,
        other => bail!("unsupported samplerate unit: {other}"),
    };
    let hz = value * factor;
    ensure!(hz.is_finite() && hz > 0.0, "samplerate must be positive");
    Ok(hz)
}

pub fn open(path: &Path, channels: [usize; 5]) -> Result<Capture> {
    ensure!(
        channels.iter().collect::<BTreeSet<_>>().len() == 5,
        "CLK/LAT/R/G/B must use five distinct channels"
    );
    let mut zip = ZipArchive::new(File::open(path)?).context("DSL must be a ZIP archive")?;
    let mut names = BTreeSet::new();
    for name in zip.file_names() {
        ensure!(names.insert(name.to_owned()), "duplicate ZIP entry: {name}");
    }
    let mut header = String::new();
    zip.by_name("header")
        .context("missing DSView header")?
        .take(65537)
        .read_to_string(&mut header)?;
    ensure!(header.len() <= 65536, "DSView header is too large");
    let mut section = "";
    let mut fields = BTreeMap::new();
    for line in header.lines().map(str::trim) {
        if line.starts_with('[') && line.ends_with(']') {
            section = &line[1..line.len() - 1];
        } else if let Some((key, value)) = line.split_once('=') {
            fields.insert((section, key.trim()), value.trim());
        }
    }
    let get = |section, key| {
        fields
            .get(&(section, key))
            .copied()
            .with_context(|| format!("missing header field [{section}] {key}"))
    };
    let version = get("version", "version")?.parse()?;
    // Do not silently reinterpret older, interleaved capture formats.
    ensure!(
        version == 3,
        "only DSView DSL version 3 is supported (got {version})"
    );
    ensure!(
        get("header", "device mode")? == "0",
        "capture is not logic mode"
    );
    let samples: usize = get("header", "total samples")?.parse()?;
    let blocks: usize = get("header", "total blocks")?.parse()?;
    ensure!(samples > 0 && blocks > 0, "empty capture");
    ensure!(
        blocks <= zip.len(),
        "block count exceeds archive entry count"
    );
    let samplerate_hz = rate(get("header", "samplerate")?)?;
    let expected = samples.div_ceil(8);
    let mut planes: [Vec<u8>; 5] = std::array::from_fn(|_| Vec::new());
    let mut block_lengths = Vec::new();
    for (slot, channel) in channels.iter().enumerate() {
        ensure!(
            fields.contains_key(&("header", format!("probe{channel}").as_str())),
            "channel {channel} is not enabled in the header"
        );
        planes[slot].try_reserve_exact(expected)?;
        for block in 0..blocks {
            let name = format!("L-{channel}/{block}");
            let mut entry = zip
                .by_name(&name)
                .with_context(|| format!("missing {name}"))?;
            let size = usize::try_from(entry.size()).context("block too large")?;
            ensure!(size > 0, "empty block {name}");
            if slot == 0 {
                block_lengths.push(size);
            } else {
                ensure!(
                    size == block_lengths[block],
                    "channel block sizes differ at {name}"
                );
            }
            let total = planes[slot]
                .len()
                .checked_add(size)
                .context("capture size overflow")?;
            // DSView may pad the last block to a 64-bit boundary.
            ensure!(
                total <= expected.saturating_add(7),
                "data exceeds sample count in {name}"
            );
            entry.read_to_end(&mut planes[slot])?;
            ensure!(planes[slot].len() == total, "truncated block {name}");
            if block + 1 < blocks {
                ensure!(total < expected, "sample count ends before final block");
            }
        }
        ensure!(
            planes[slot].len() >= expected,
            "channel {channel} truncated: need {expected} bytes, got {}",
            planes[slot].len()
        );
    }
    Ok(Capture {
        metadata: Metadata {
            version,
            samples,
            samplerate_hz,
            blocks,
            channels,
        },
        planes,
    })
}
