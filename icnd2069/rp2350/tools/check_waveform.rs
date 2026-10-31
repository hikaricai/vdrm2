//! Inspect the exact DMA buffer on the host before programming hardware.
#[path = "../src/protocol.rs"]
mod protocol;
use protocol::*;
fn sample(w: &[u32], t: usize) -> u16 {
    (w[t / 2] >> ((t & 1) * 16)) as u16
}
fn main() {
    assert_eq!((ROWS, CHANNELS, GROUPS, DEMO_GRAY), (16,16,64,255));
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
    assert_eq!(data.len(),256);
    assert!(data.iter().all(|c|c.2==255));
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
    let rows:Vec<_>=(1..FRAME_DCLKS).filter(|&t|
        sample(&words,t)&ROW!=0 && sample(&words,t-1)&ROW==0).collect();
    assert_eq!(rows.len(),1024);
    assert_eq!(rows[0]-cs[0].0,25);
    assert!(rows.windows(2).all(|p|p[1]-p[0]==81));
    for (i,&t) in rows.iter().enumerate() {
        let width=(t..FRAME_DCLKS).take_while(|&n|sample(&words,n)&ROW!=0).count();
        assert_eq!(width,if i==0 {12} else {4});
        assert_eq!(sample(&words,t-4)&LCK,LCK);
        assert_eq!(sample(&words,t-4)&DIN!=0,i%16==0);
        assert_eq!(sample(&words,t-4)&BK!=0,BK_ACTIVE_HIGH);
        assert_eq!(sample(&words,t)&BK!=0,!BK_ACTIVE_HIGH);
    }
    assert!((FRAME_DCLKS-18..FRAME_DCLKS).all(|t|sample(&words,t)==PARK));
    println!("PASS: 16 scans; 49 full WR_CFG; 256 gray255 x8 copies; VSYNC/EN_OP/PRE_ACT falls 8/31/48; 1024 ROW; 81 cadence, 12/4 width; frame tail PARK");
}
