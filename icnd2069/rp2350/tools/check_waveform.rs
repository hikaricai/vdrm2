//! Inspect the exact DMA buffer on the host before programming hardware.
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;
fn sample(w: &[u32], t: usize) -> u16 {
    (w[t / 2] >> ((t & 1) * 16)) as u16
}
fn main() {
    assert_eq!((ROWS, CHANNELS, GROUPS, DEMO_GRAY), (16,16,64,8192));
    assert_eq!(DCLK_HZ,12_500_000);
    let mut words=vec![0;FRAME_WORDS];
    assert_eq!(encode_frame(&mut words,&demo_frame()),FRAME_WORDS);
    let mut cs=Vec::new();
    let (mut width,mut shift)=(0,0u16);
    for t in 0..FRAME_DCLKS {
        let p=sample(&words,t);
        if p&LE==0 && width!=0 {
            cs.push((t,width,shift));
            width=0;
        }
        shift=(shift<<1)|u16::from(p&SIN!=0);
        if p&LE!=0 {width+=1;}
    }
    assert_eq!(width,0);
    assert_eq!(cs[..3],[(8,3,0),(31,11,0),(48,14,0)]);
    let mut expected=vec![0x00aa,0x01aa];
    expected.extend(REGISTERS.iter().map(|&(a,v)|(a as u16)<<8|v as u16));
    expected.extend([0x0055,0x0155]);
    assert_eq!(CONFIG_WRITES,49);
    let writes:Vec<_>=cs.iter().filter(|c|c.1==5).map(|c|c.2).collect();
    assert_eq!(writes,expected);
    assert!(writes.contains(&0x020f)); // scan register = 16-1.
    assert_eq!(cs[3].0,185);
    let data:Vec<_>=cs.iter().filter(|c|c.1==1).collect();
    assert_eq!(data.len(),ROWS * CHANNELS);
    assert!(data.iter().enumerate().all(|(i,c)|c.2==if i%16==0 {0} else {8192}));
    for (i,p) in data.windows(2).enumerate() {
        assert_eq!(p[1].0-p[0].0,136+usize::from((i+1)%16==0));
    }
    for &(fall,w,value) in &cs {
        if w!=1 && w!=5 {continue;}
        for copy in 0..8 {
            let mut v=0u16;
            for t in fall-128+copy*16..fall-128+(copy+1)*16 {
                v=(v<<1)|u16::from(sample(&words,t)&SIN!=0);
            }
            assert_eq!(v,value);
        }
    }
    let mut runs=[0;ROW_RUN_COUNT];
    encode_row_runs(&mut runs);
    let mut tick=0;
    let mut rows=Vec::new();
    let mut widths=Vec::new();
    for &run in &runs {
        let count=(run>>1) as usize+1;
        if run&1!=0 { rows.push(tick-1); widths.push(count); }
        tick+=count;
    }
    assert_eq!(tick,FRAME_DCLKS);
    assert_eq!(rows.len(),GROUPS * ROWS);
    assert_eq!(rows[0]-cs[0].0,25);
    assert!(rows.windows(2).enumerate().all(|(i,p)|p[1]-p[0]==row_period(i)));
    let counts:Vec<_>=(80..=83).map(|d| rows.windows(2).filter(|p|p[1]-p[0]==d).count()).collect();
    assert_eq!(counts.iter().sum::<usize>(),GROUPS * ROWS - 1);
    for (i,&t) in rows.iter().enumerate() {
        let width=widths[i];
        assert_eq!(width,if i==0 {12} else {4});
        assert_eq!(sample(&words,t-4)&LCK,LCK);
        assert_eq!(sample(&words,t-4)&DIN!=0,i%ROWS==0);
        assert_eq!(sample(&words,t-4)&BK!=0,BK_ACTIVE_HIGH);
        assert_eq!(sample(&words,t)&BK!=0,!BK_ACTIVE_HIGH);
    }
    assert!((FRAME_DCLKS-18..FRAME_DCLKS).all(|t|sample(&words,t)&0xfff==PARK));
    let extra:usize=(0..FRAME_DCLKS).map(|t|(sample(&words,t)>>12) as usize).sum();
    assert_eq!(12*FRAME_DCLKS+extra,2500123);
    assert_eq!(runs.last().unwrap() & 1,0);
    assert!((rows.last().unwrap()+4..FRAME_DCLKS)
        .all(|t|sample(&words,t)&(LCK|DIN)==0));
    let baseline = words.clone();
    for pattern in [Pattern::Reference, Pattern::White, Pattern::Black,
                    Pattern::Out0, Pattern::Out1, Pattern::AlternatingRows,
                    Pattern::Checker, Pattern::Split] {
        for gray in [0, 255, 2048, 4096, 8192] {
            let image = pattern_frame(pattern, gray);
            encode_frame(&mut words, &image);
            // Compare every pin and delay bit except DATA to the saved baseline.
            let mask = !((SIN as u32) | ((SIN as u32) << 16));
            assert!(words.iter().zip(&baseline).all(|(a,b)|a & mask == b & mask));
            let mut decoded = Vec::new();
            let mut le_width = 0;
            for t in 0..FRAME_DCLKS {
                if sample(&words,t)&LE != 0 { le_width += 1; continue; }
                if le_width == 1 {
                    let mut value = 0;
                    for copy in 0..CHAIN_COPIES {
                        let mut v = 0;
                        for k in t-128+copy*16..t-128+(copy+1)*16 {
                            v = (v<<1) | u16::from(sample(&words,k)&SIN != 0);
                        }
                        if copy == 0 { value = v; } else { assert_eq!(v,value); }
                    }
                    decoded.push(value);
                }
                le_width = 0;
            }
            let expected: Vec<u16> = image.iter().flat_map(|r|r.iter().rev().copied()).collect();
            assert_eq!(decoded,expected,"{pattern:?} gray={gray}");
            for row in 0..ROWS {
                for out in 0..CHANNELS {
                    let expected_on = match pattern {
                        Pattern::Reference => out < 15,
                        Pattern::White => true,
                        Pattern::Black => false,
                        Pattern::Out0 => out == 0,
                        Pattern::Out1 => out == 1,
                        Pattern::AlternatingRows => row%2 == 0,
                        Pattern::Checker => row%2 == out%2,
                        Pattern::Split => row < ROWS/2,
                    };
                    assert_eq!(image[row][out],if expected_on {gray} else {0});
                }
            }
        }
    }
    println!("PASS: 16 scans x16 channels; full 49 configs with 020F; 1024 OE; 256 words x8; 8 patterns x5 gray levels; saved CLK delays unchanged");
}
