//! Host inspection of the 16-scan uniform diagnostic DMA stream.
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;

fn sample(words: &[u32], t: usize) -> u16 {
    (words[t / 2] >> ((t & 1) * 16)) as u16
}
fn commands(words: &[u32]) -> Vec<(usize, usize, u16)> {
    let mut result = Vec::new();
    let (mut shift, mut width) = (0u16, 0);
    for t in 0..words.len()*2 {
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
    assert_eq!((ROWS, CHANNELS, GROUPS, DEMO_GRAY), (16,16,64,255));
    assert_eq!(DCLK_HZ, 10_000_000);
    assert!(DEMO_GRAY < GRAY_CAPACITY);
    let mut init = vec![0; INIT_WORDS];
    let len = encode_init(&mut init);
    let c = commands(&init[..len]);
    assert_eq!(c[..3].iter().map(|c|c.1).collect::<Vec<_>>(), vec![9,12,14]);
    let writes: Vec<_> = c.iter().filter(|c|c.1==5).map(|c|c.2).collect();
    let mut expected = vec![0x00aa,0x01aa];
    expected.extend(REGISTERS.iter().map(|&(a,v)|(a as u16)<<8|v as u16));
    expected.extend([0x0055,0x0155]);
    assert_eq!(writes, expected);
    assert_eq!(c.iter().filter(|c|c.1==3).count(),1);
    let black: Vec<_> = c.iter().filter(|c|c.1==1).map(|c|c.2).collect();
    assert_eq!(black,vec![0;256]);

    let mut words = vec![0; FRAME_WORDS];
    assert_eq!(encode_frame(&mut words,&demo_frame()),FRAME_WORDS);
    let c = commands(&words);
    assert_eq!(c.len(),258);
    assert_eq!((c[0].1,c[1].1),(3,11));
    assert!(c[2..].iter().all(|c|(c.1,c.2)==(1,255)));
    assert!(c[2..].windows(2).all(|p|p[1].0-p[0].0==17));
    let mut rows = Vec::new();
    for t in 1..FRAME_DCLKS {
        if sample(&words,t)&ROW!=0 && sample(&words,t-1)&ROW==0 { rows.push(t); }
    }
    assert_eq!(rows.len(),1024);
    assert_eq!(rows[0]-c[0].0,38);
    assert!(rows.windows(2).all(|p|p[1]-p[0]==160));
    for (i,&t) in rows.iter().enumerate() {
        let width=(t..FRAME_DCLKS).take_while(|&n|sample(&words,n)&ROW!=0).count();
        assert_eq!(width,if i==0 {12} else {4});
        assert_eq!(sample(&words,t-4)&LCK,LCK);
        assert_eq!(sample(&words,t-4)&DIN!=0,i%16==0);
        assert_eq!(sample(&words,t-4)&BK!=0,BK_ACTIVE_HIGH);
        assert_eq!(sample(&words,t)&BK!=0,!BK_ACTIVE_HIGH);
    }
    assert_eq!(sample(&words,FRAME_DCLKS-1),PARK);
    println!("PASS: 16 scans, 256 gray255 words, 1024 ROW 12/4 pulses, 160-clock cadence; optional init validated (not sent when INITIALIZE_CHIP=false)");
}
