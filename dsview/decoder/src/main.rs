fn main() {
    let path = std::env::args().nth(1).unwrap();
    let file = std::fs::read_to_string(path).unwrap();
    show_cmd_seq(file.as_str());
}

fn show_cmd_seq(file: &str) {
    for line in file.lines().skip(1) {
        let mut items = line.split(',');
        let cmd = items.nth(2).unwrap();
        let cmd: u32 = u32::from_str_radix(cmd, 16).unwrap();
        if cmd <= 2 {
            continue;
        }
        let value = items.next().unwrap();
        println!("cmd {cmd} value {value}");
    }
}

fn show_cmds(file: &str) {
    let mut map = std::collections::BTreeMap::<(u32, u32), usize>::new();
    for line in file.lines().skip(1) {
        let mut items = line.split(',');
        let cmd = items.nth(2).unwrap();
        let cmd: u32 = u32::from_str_radix(cmd, 16).unwrap();
        if cmd <= 2 {
            continue;
        }

        let value = items.next().unwrap();
        let value: u32 = u32::from_str_radix(value, 16).unwrap();
        let entry = map.entry((cmd, value)).or_default();
        *entry += 1;
    }
    for ((cmd, value), total) in map {
        println!("cmd {cmd:x} value {value:4x} {value:32b} total {total}");
    }
}
