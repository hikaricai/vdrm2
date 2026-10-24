use serde::Serialize;

pub fn command_name(clocks: usize) -> &'static str {
    match clocks {
        1 => "DATA_LATCH",
        3 => "VSYNC",
        5 => "WR_CFG",
        7 => "RD_CFG",
        9 => "SOFT_RST",
        11 => "EN_OP",
        12 => "DIS_OP",
        14 => "PRE_ACT",
        _ => "UNKNOWN",
    }
}

#[derive(Debug, Serialize)]
pub struct Register {
    pub address_hex: String,
    // RD_CFG sends an address on SIN; its result is on SDO, which is not captured here.
    pub value_hex: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct Lane {
    pub prefix_bits: String,
    /// Whole 16-bit words, aligned backwards from the latch, in transmission order.
    /// Without an explicit chain length, these are candidates, not a topology inference.
    pub words_hex: Vec<String>,
    /// Actual continuous shift-register tail, including earlier commands if necessary.
    pub tail16_hex: Option<String>,
    pub registers: Vec<Register>,
}

#[derive(Debug, Serialize)]
pub struct Event {
    pub index: usize,
    pub shift_start_sample: usize,
    pub lat_start_sample: usize,
    /// Falling-edge sample, or exclusive capture end for an incomplete pulse.
    pub end_sample: usize,
    pub time_us: f64,
    pub lat_clocks: usize,
    pub shift_clocks: usize,
    pub complete: bool,
    pub command: &'static str,
    pub data_kind: &'static str,
    /// Zero means that no VSYNC has yet been observed.
    pub frame: usize,
    pub data_index: Option<usize>,
    pub line_1based: Option<usize>,
    pub out: Option<usize>,
    pub red: Lane,
    pub green: Lane,
    pub blue: Lane,
    pub warnings: Vec<String>,
}

pub struct Decoder {
    previous: [u8; 5],
    rate: f64,
    chips: Option<usize>,
    shift_start: usize,
    bits: Vec<[u8; 3]>,
    rolling: [u16; 3],
    total_clocks: usize,
    lat_start: Option<usize>,
    missing_start: bool,
    lat_clocks: usize,
    simultaneous: usize,
    index: usize,
    frame: usize,
    vsync_count: usize,
    data_index: usize,
}

impl Decoder {
    pub fn new(initial: [u8; 5], rate: f64, chips: Option<usize>) -> Self {
        Self {
            previous: initial,
            rate,
            chips,
            shift_start: 0,
            bits: Vec::new(),
            rolling: [0; 3],
            total_clocks: 0,
            lat_start: (initial[1] == 1).then_some(0),
            missing_start: initial[1] == 1,
            lat_clocks: 0,
            simultaneous: 0,
            index: 0,
            frame: 0,
            vsync_count: 0,
            data_index: 0,
        }
    }

    pub fn push(&mut self, sample: usize, pins: [u8; 5]) -> Option<Event> {
        let rising = self.previous[0] == 0 && pins[0] == 1;
        let lat_rise = self.previous[1] == 0 && pins[1] == 1;
        let lat_fall = self.previous[1] == 1 && pins[1] == 0;
        if rising && (lat_rise || lat_fall) {
            self.simultaneous += 1;
        }
        // At a simultaneous LE falling edge and DCLK rising edge, close the
        // old latch first. The new LE-low clock belongs to the next window.
        // The acquisition cannot establish sub-sample ordering; flag it.
        let event = lat_fall.then(|| self.emit(sample, true));
        if lat_rise {
            self.lat_start = Some(sample);
            self.lat_clocks = 0;
        }
        if rising {
            let rgb = [pins[2], pins[3], pins[4]];
            self.bits.push(rgb);
            for (tail, bit) in self.rolling.iter_mut().zip(rgb) {
                *tail = (*tail << 1) | u16::from(bit);
            }
            self.total_clocks += 1;
            if pins[1] == 1 {
                self.lat_clocks += 1;
            }
        }
        self.previous = pins;
        event
    }

    pub fn finish(&mut self, end: usize) -> Option<Event> {
        self.lat_start.map(|_| self.emit(end, false))
    }

    pub fn total_clocks(&self) -> usize {
        self.total_clocks
    }

    pub fn trailing_clocks(&self) -> usize {
        self.bits.len()
    }

    fn emit(&mut self, end: usize, closed: bool) -> Event {
        let complete = closed && !self.missing_start;
        let command = if complete {
            command_name(self.lat_clocks)
        } else {
            "INCOMPLETE"
        };
        let payload = matches!(command, "DATA_LATCH" | "WR_CFG" | "RD_CFG");
        let mut warnings = Vec::new();
        if self.missing_start {
            warnings.push("capture starts inside LAT high; command length is unknown".into());
        }
        if !closed {
            warnings.push("capture ends inside LAT high; no latch falling edge observed".into());
        }
        if command == "UNKNOWN" {
            warnings.push(format!(
                "undocumented LAT length: {} rising edges",
                self.lat_clocks
            ));
        }
        if self.simultaneous > 0 {
            warnings.push(format!(
                "{} CLK rising edge(s) coincide with LAT transitions; ordering is ambiguous at this sample rate",
                self.simultaneous
            ));
        }
        let available = self.bits.len() / 16;
        let wanted = if payload {
            self.chips.unwrap_or(available)
        } else {
            available
        };
        let words = available.min(wanted);
        if payload && (available == 0 || available < wanted) {
            warnings.push(format!(
                "insufficient clocks in this window: {} bits for {} requested word(s)",
                self.bits.len(),
                wanted.max(1)
            ));
        }
        if self.total_clocks < 16 {
            warnings.push(
                "fewer than 16 observed DCLK edges; shift-register tail is incomplete".into(),
            );
        }
        let prefix = self.bits.len() - words * 16;
        let lanes: [Lane; 3] = std::array::from_fn(|lane| {
            let values: Vec<u16> = self.bits[prefix..]
                .as_chunks::<16>()
                .0
                .iter()
                .map(|bits| {
                    bits.iter()
                        .fold(0, |word, bit| (word << 1) | u16::from(bit[lane]))
                })
                .collect();
            Lane {
                prefix_bits: self.bits[..prefix]
                    .iter()
                    .map(|b| char::from(b'0' + b[lane]))
                    .collect(),
                words_hex: values.iter().map(|v| format!("{v:04X}")).collect(),
                tail16_hex: (self.total_clocks >= 16)
                    .then(|| format!("{:04X}", self.rolling[lane])),
                registers: values
                    .iter()
                    .filter_map(|v| match command {
                        "WR_CFG" => Some(Register {
                            address_hex: format!("{:02X}", v >> 8),
                            value_hex: Some(format!("{:02X}", v & 255)),
                        }),
                        "RD_CFG" => Some(Register {
                            address_hex: format!("{:02X}", v & 255),
                            value_hex: None,
                        }),
                        _ => None,
                    })
                    .collect(),
            }
        });
        if command == "VSYNC" {
            self.vsync_count += 1;
            self.frame = self.vsync_count;
            self.data_index = 0;
        }
        if command == "SOFT_RST" {
            // Reset makes a previously inferred SRAM position unreliable.
            self.frame = 0;
            self.data_index = 0;
        }
        let data_index = (command == "DATA_LATCH" && self.frame > 0).then_some(self.data_index);
        if command == "DATA_LATCH" {
            self.data_index += 1;
        }
        let [red, green, blue] = lanes;
        let event = Event {
            index: self.index,
            shift_start_sample: self.shift_start,
            lat_start_sample: self.lat_start.unwrap_or(self.shift_start),
            end_sample: end,
            time_us: end as f64 * 1e6 / self.rate,
            lat_clocks: self.lat_clocks,
            shift_clocks: self.bits.len(),
            complete,
            command,
            data_kind: match command {
                "DATA_LATCH" => "gray16",
                "WR_CFG" => "register_address8_value8",
                "RD_CFG" => "read_address_low8_no_sdo",
                _ => "raw_shift_history_no_defined_payload",
            },
            frame: self.frame,
            data_index,
            line_1based: data_index.map(|i| i / 16 + 1),
            out: data_index.map(|i| 15 - i % 16),
            red,
            green,
            blue,
            warnings,
        };
        self.index += 1;
        self.bits.clear();
        self.shift_start = end;
        self.lat_start = None;
        self.lat_clocks = 0;
        self.missing_start = false;
        self.simultaneous = 0;
        event
    }
}
