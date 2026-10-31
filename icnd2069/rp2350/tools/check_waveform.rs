//! Inspect the exact DMA buffer on the host before programming hardware.
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;
fn sample(w: &[u32], t: usize) -> u16 {
    (w[t / 2] >> ((t & 1) * 16)) as u16
}
fn main() {
    assert_eq!((ROWS, CHANNELS, GROUPS, DEMO_GRAY), (40,16,64,8192));
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
    assert_eq!(cs[3].0,185);
    let data:Vec<_>=cs.iter().filter(|c|c.1==1).collect();
    assert_eq!(data.len(),640);
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
    assert_eq!(rows.len(),2560);
    assert_eq!(rows[0]-cs[0].0,25);
    assert!(rows.windows(2).enumerate().all(|(i,p)|p[1]-p[0]==row_period(i)));
    let counts:Vec<_>=(80..=83).map(|d| rows.windows(2).filter(|p|p[1]-p[0]==d).count()).collect();
    assert_eq!(counts,[141,1982,392,44]);
    for (i,&t) in rows.iter().enumerate() {
        let width=widths[i];
        assert_eq!(width,if i==0 {12} else {4});
        assert_eq!(sample(&words,t-4)&LCK,LCK);
        assert_eq!(sample(&words,t-4)&DIN!=0,i%40==0);
        assert_eq!(sample(&words,t-4)&BK!=0,BK_ACTIVE_HIGH);
        assert_eq!(sample(&words,t)&BK!=0,!BK_ACTIVE_HIGH);
    }
    assert!((FRAME_DCLKS-18..FRAME_DCLKS).all(|t|sample(&words,t)&0xfff==PARK));
    let extra:usize=(0..FRAME_DCLKS).map(|t|(sample(&words,t)>>12) as usize).sum();
    assert_eq!(12*FRAME_DCLKS+extra,2500123);
    encode_with_config(&mut words,&demo_frame(),Some(0x0055));
    update_rotation(&mut words,0x1c72);
    for copy in 0..8 {
        let mut v=0u16;
        for t in 329+copy*16..329+(copy+1)*16 {
            v=(v<<1)|u16::from(sample(&words,t)&SIN!=0);
        }
        assert_eq!(v,0x1c72);
    }
    println!("PASS: NOVA 40 scans; full and rotating configs; 640 gray words x8; exact reference ROW schedule; 12/4 width; frame tail PARK");
}
