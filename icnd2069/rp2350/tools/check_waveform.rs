//! Host inspection of the emitted DMA stream and all 47 register slots.
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn sample(words: &[u32], t: usize) -> u16 {
    (words[t / 2] >> ((t & 1) * 16)) as u16
}
fn commands(words: &[u32]) -> Vec<(usize, usize, u16)> {
    let mut result = Vec::new();
    let (mut shift, mut width) = (0u16, 0);
    for t in 0..FRAME_DCLKS {
        let pins = sample(words, t);
        if pins & LE == 0 && width != 0 {
            result.push((t, width, shift));
            width = 0;
        }
        shift = (shift << 1) | u16::from(pins & SIN != 0);
        if pins & LE != 0 { width += 1; }
    }
    assert_eq!(width, 0);
    result
}
fn main() {
    assert!(DEMO_GRAY < GRAY_CAPACITY);
    assert_eq!(REGISTERS.len(), 47);
    assert_eq!(PARK_WORD, PARK as u32 * 0x10001);
    // Independent reference R-chain table, including the captured scan count.
    let reference = [
        0x0055,0x0155,0x0227,0x033f,0x0404,0x050a,0x0601,0x0720,
        0x0820,0x0908,0x0a08,0x0b00,0x0c01,0x0d01,0x0e07,0x0f01,
        0x10c2,0x1121,0x1201,0x1300,0x1400,0x1500,0x1600,0x17f0,
        0x181f,0x1920,0x1a1f,0x1b10,0x1c72,0x1d82,0x1e41,0x1f04,
        0x2048,0x2101,0x221c,0x7000,0x7100,0x7200,0x7300,0x7400,
        0xf000,0xf100,0xf200,0xf300,0xf400,0xf500,0x2301,
    ];
    let mut words = vec![0; FRAME_WORDS];
    encode_frame(&mut words, &demo_frame());
    let baseline = words.clone();
    for (i, expected) in reference.into_iter().enumerate() {
        update_register(&mut words, i);
        let c = commands(&words);
        assert_eq!(c.len(), 3 + 5 + 640);
        assert_eq!([c[0].1,c[1].1,c[2].1], [3,11,14]);
        assert_eq!(c[0].0, VSYNC_FALL);
        assert_eq!(c[1].0-c[0].0, 23);
        assert_eq!(c[2].0-c[0].0, 40);
        assert_eq!(c[3..8].iter().map(|c| (c.1,c.2)).collect::<Vec<_>>(),
                   vec![(5,0x00aa),(5,0x01aa),(5,expected),(5,0x0055),(5,0x0155)]);
        assert!(c[8..].iter().all(|c| (c.1,c.2)==(1,255)));
        assert!(c[3..].windows(2).all(|p| p[1].0-p[0].0==136));
        // Verify all eight transmitted copies, not just the final word.
        for &(fall, _, value) in &c[3..] {
            for copy in 0..8 {
                let mut got = 0u16;
                for t in fall-128+copy*16..fall-128+(copy+1)*16 {
                    got = (got << 1) | u16::from(sample(&words,t)&SIN!=0);
                }
                assert_eq!(got,value);
            }
        }
        // Rotating config may not alter ROW/DIN/LCK/BK.
        for t in 0..FRAME_DCLKS {
            assert_eq!(sample(&words,t)&!(SIN|LE), sample(&baseline,t)&!(SIN|LE));
        }
    }
    let mut row_edges = Vec::new();
    let mut last = PARK;
    for t in 0..FRAME_DCLKS {
        let pins = sample(&words,t);
        if pins&ROW!=0 && last&ROW==0 { row_edges.push(t); }
        last = pins;
    }
    assert_eq!(row_edges.len(),2560);
    assert_eq!(row_edges[0]-VSYNC_FALL,25);
    assert!(row_edges.windows(2).all(|p| p[1]-p[0]==81));
    for (i,&t) in row_edges.iter().enumerate() {
        let width = (t..FRAME_DCLKS).take_while(|&n| sample(&words,n)&ROW!=0).count();
        assert_eq!(width,if i==0 {12} else {4});
        assert_eq!(sample(&words,t-4)&LCK,LCK);
        assert_eq!(sample(&words,t-4)&BK,BK);
        assert_eq!(sample(&words,t)&BK,0);
        assert_eq!(sample(&words,t-4)&DIN!=0,i%40==0);
    }
    assert!((SCAN_START+2560*81..FRAME_DCLKS).all(|t| sample(&words,t)==PARK));
    println!("PASS: reference 47-register rotation, 5 writes/frame, 8 copies/word, 640 gray255 words, 2560 ROW 12/4 pulses, 81-clock cadence, blank tail; {} bytes",FRAME_WORDS*4);
}
