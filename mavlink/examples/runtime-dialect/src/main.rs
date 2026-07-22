use std::{env, error::Error};

use mavlink::{Connectable, SerialConfig, dialects::dynamic::DynamicDialect};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    let program = args.next().unwrap_or_else(|| "runtime-dialect".to_owned());
    let usage_text = format!("usage: {program} <dialect.xml> <port> <baud>");
    let xml = args.next().unwrap_or_else(|| panic!("{usage_text}"));
    let port = args.next().unwrap_or_else(|| panic!("{usage_text}"));
    let baud = args
        .next()
        .unwrap_or_else(|| panic!("{usage_text}"))
        .parse()?;

    let dialect = DynamicDialect::from_xml_file(xml)?;
    let config = SerialConfig::new(port, baud);
    let mut connection = config.connect_with_dialect(dialect)?;
    connection.set_allow_recv_any_version(true);

    loop {
        let (_, message) = connection.recv()?;
        println!("{:?}", message);
    }
}
