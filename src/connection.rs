use common::MavMessage;
use crc16;

use byteorder::{LittleEndian, ReadBytesExt};

use mio;
use mio::{TryRead, TryWrite};
use mio::tcp::TcpStream;
use mio::udp::UdpSocket;
use std::io::Cursor;
use std::collections::VecDeque;
use chan::{Sender, Receiver};
use eventual::Complete;
use std::net::{ToSocketAddrs, SocketAddr};
use std::io;
use std::str::FromStr;
use std::sync::{Arc, Mutex, Condvar};
use std::collections::HashSet;

pub const CLIENT: mio::Token = mio::Token(0);

pub type UpdaterList = Vec<Box<FnMut(MavMessage) -> bool>>;

#[derive(Debug)]
struct MavPacket {
    seq: u8,
    system_id: u8,
    component_id: u8,
    message_id: u8,
    data: Vec<u8>,
    checksum: u16,
}

impl MavPacket {
    fn new(payload: &[u8]) -> MavPacket {
        let mut cur = Cursor::new(payload);
        cur.set_position(2);
        MavPacket {
            seq: cur.read_u8().unwrap(),
            system_id: cur.read_u8().unwrap(),
            component_id: cur.read_u8().unwrap(),
            message_id: cur.read_u8().unwrap(),
            data: payload[6..payload.len() - 2].to_vec(),
            checksum: {
                cur.set_position((payload.len() - 2) as u64);
                cur.read_u16::<LittleEndian>().unwrap()
            },
        }
    }

    fn parse(&self) -> Option<MavMessage> {
        MavMessage::parse(self.message_id, &self.data)
    }

    fn encode_nocrc(&self) -> Vec<u8> {
        let mut pkt: Vec<u8> = vec![
            0xfe, self.data.len() as u8, self.seq,
            self.system_id, self.component_id, self.message_id,
        ];
        pkt.extend(&self.data);
        pkt
    }

    fn encode(&self) -> Vec<u8> {
        let mut pkt = self.encode_nocrc();
        pkt.push((self.checksum & 0xff) as u8);
        pkt.push((self.checksum >> 8) as u8);
        pkt
    }

    fn calc_crc(&self) -> u16 {
        let mut crc = crc16::State::<crc16::MCRF4XX>::new();
        crc.update(&self.encode_nocrc()[1..]);
        crc.update(&[MavMessage::extra_crc(self.message_id)]);
        crc.get()
    }

    fn update_crc(&mut self) {
        self.checksum = self.calc_crc();
    }

    fn check_crc(&self) -> bool {
        self.calc_crc() == self.checksum
    }
}

pub fn parse_mavlink_string(buf: &[u8]) -> String {
    buf.iter()
       .take_while(|a| **a != 0)
       .map(|x| *x as char)
       .collect::<String>()
}

pub enum MavSocket {
    Tcp(TcpStream),
    UdpIn(HashSet<SocketAddr>, UdpSocket),
    UdpOut(SocketAddr, UdpSocket),
}

pub fn socket_tcp<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(TcpStream::connect(&addr));
    Ok(MavSocket::Tcp(socket))
}

pub fn socket_udpin<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(UdpSocket::bound(&addr));
    Ok(MavSocket::UdpIn(HashSet::new(), socket))
}

pub fn socket_udpout<T: ToSocketAddrs>(address: T) -> io::Result<MavSocket> {
    let addr = address.to_socket_addrs().unwrap().next().unwrap();
    let socket = try!(UdpSocket::bound(&SocketAddr::from_str("0.0.0.0:0").unwrap()));
    Ok(MavSocket::UdpOut(addr, socket))
}

pub struct DkHandler {
    pub socket: MavSocket,
    pub buf: Vec<u8>,
    pub buf_cache: Vec<u8>,
    pub vehicle_tx: Sender<DkHandlerRx>,
    pub watchers: UpdaterList,
    pub pair: Arc<(Mutex<u64>, Condvar)>,
}

pub enum DkHandlerRx {
    RxCork,
    RxMessage(MavMessage),
}

pub enum DkHandlerMessage {
    TxMessage(Vec<u8>),
    TxWatcher(Box<FnMut(MavMessage) -> bool + Send>),
    TxCork,
    TxUncork,
}

impl DkHandler {
    fn dispatch(&mut self, pkt: MavMessage) {
        self.vehicle_tx.send(DkHandlerRx::RxMessage(pkt.clone()));

        let ups = self.watchers.split_off(0);
        for mut x in ups.into_iter() {
            if !x(pkt.clone()) {
                self.watchers.push(x);
            }
        }

        let &(ref lock, ref cvar) = &*self.pair;
        let mut started = lock.lock().unwrap();
        *started += 1;
        cvar.notify_one();
    }

    pub fn register(&mut self, event_loop: &mut mio::EventLoop<DkHandler>) {
        match &self.socket {
            &MavSocket::Tcp(ref socket) => {
                let _ = event_loop.register(socket,
                    CLIENT,
                    mio::EventSet::readable(),
                    mio::PollOpt::edge());
            }
            &MavSocket::UdpIn(_, ref socket) => {
                let _ = event_loop.register(socket,
                    CLIENT,
                    mio::EventSet::readable(),
                    mio::PollOpt::edge());
            }
            &MavSocket::UdpOut(_, ref socket) => {
                let _ = event_loop.register(socket,
                    CLIENT,
                    mio::EventSet::readable(),
                    mio::PollOpt::edge());
            }
        }
    }

    pub fn deregister(&mut self, event_loop: &mut mio::EventLoop<DkHandler>) {
        match &self.socket {
            &MavSocket::Tcp(ref socket) => {
                let _ = event_loop.deregister(socket);
            }
            &MavSocket::UdpIn(_, ref socket) => {
                let _ = event_loop.deregister(socket);
            }
            &MavSocket::UdpOut(_, ref socket) => {
                let _ = event_loop.deregister(socket);
            }
        }
    }
}

impl mio::Handler for DkHandler {
    type Timeout = ();
    type Message = DkHandlerMessage;

    fn ready(&mut self,
             _: &mut mio::EventLoop<DkHandler>,
             token: mio::Token,
             events: mio::EventSet) {
        match token {
            CLIENT => {
                // Only receive readable events
                assert!(events.is_readable());

                let len = match &mut self.socket {
                    &mut MavSocket::Tcp(ref mut socket) => {
                        match socket.try_read_buf(&mut self.buf) {
                            Ok(Some(0)) | Ok(None) => {
                                return;
                            }
                            Ok(Some(len)) => {
                                len
                            }
                            Err(e) => {
                                panic!("got an error trying to read; err={:?}", e);
                            }
                        }
                    }
                    &mut MavSocket::UdpIn(ref mut clients, ref mut socket) => {
                        match socket.recv_from(&mut self.buf) {
                            Ok(None) => {
                                return;
                            }
                            Ok(Some((len, addr))) => {
                                clients.insert(addr);
                                len
                            }
                            Err(e) => {
                                panic!("got an error trying to read; err={:?}", e);
                            }
                        }
                    }
                    &mut MavSocket::UdpOut(_, ref mut socket) => {
                        match socket.recv_from(&mut self.buf) {
                            Ok(None) => {
                                return;
                            }
                            Ok(Some((len, _))) => {
                                len
                            }
                            Err(e) => {
                                panic!("got an error trying to read; err={:?}", e);
                            }
                        }
                    }
                };

                self.buf_cache.extend(&self.buf[0..len]);

                let mut start = 0;
                loop {
                    match self.buf[start..len].iter().position(|&x| x == 0xfe) {
                        Some(i) => {
                            if start + i + 8 > self.buf.len() {
                                break;
                            }

                            let len = self.buf[start + i + 1] as usize;

                            if start + i + 8 + len > self.buf.len() {
                                break;
                            }

                            let packet;
                            {
                                let pktbuf = &self.buf[(start + i)..(start + i + 8 + len)];
                                packet = MavPacket::new(pktbuf);

                                // println!("ok {:?}", pktbuf);

                                if !packet.check_crc() {
                                    // println!("failed CRC!");
                                    start += i + 1;
                                    continue;
                                }
                            }

                            // handle packet
                            if let Some(pkt) = packet.parse() {
                                self.dispatch(pkt);
                            }

                            // change this
                            start += i + 8 + len;
                        }
                        None => {
                            break;
                        }
                    }
                }

                self.buf_cache = self.buf_cache.split_off(start);

                // Re-register the socket with the event loop. The current
                // state is used to determine whether we are currently reading
                // or writing.
                // self.reregister(event_loop);
            }
            _ => panic!("Received unknown token"),
        }
    }

    fn notify(&mut self, event_loop: &mut mio::EventLoop<DkHandler>, message: DkHandlerMessage) {
        match message {
            DkHandlerMessage::TxMessage(msg) => {
                // println!("[txmessage] {:?}", msg);
                match &mut self.socket {
                    &mut MavSocket::Tcp(ref mut socket) => {
                        socket.try_write_buf(&mut Cursor::new(msg)).unwrap();
                    }
                    &mut MavSocket::UdpIn(ref clients, ref mut socket) => {
                        for client in clients {
                            socket.send_to(&msg, &client).unwrap();
                        }
                    }
                    &mut MavSocket::UdpOut(ref client, ref mut socket) => {
                        socket.send_to(&msg, &client).unwrap();
                    }
                }
            }
            DkHandlerMessage::TxWatcher(func) => {
                // println!("[txwatcher]");
                self.watchers.push(func);
            }
            DkHandlerMessage::TxCork => {
                // println!("[txcork]");
                self.deregister(event_loop);
                self.vehicle_tx.send(DkHandlerRx::RxCork);
            }
            DkHandlerMessage::TxUncork => {
                // println!("[txuncork]");
                self.register(event_loop);
            }
        }
    }
}

pub struct VehicleConnection {
    pub tx: mio::Sender<DkHandlerMessage>,
    pub rx: Receiver<DkHandlerRx>,
    pub msg_id: u8,
    pub started: bool,
    pub buffer: VecDeque<MavMessage>,
    pub pair: Arc<(Mutex<u64>, Condvar)>,
}

pub struct VehicleAwait {
    value: u64,
    pair: Arc<(Mutex<u64>, Condvar)>,
}

impl VehicleAwait {
    pub fn sleep(self) {
        let &(ref lock, ref cvar) = &*self.pair;
        let mut started = lock.lock().unwrap();
        while *started == self.value {
            started = cvar.wait(started).unwrap();
        }
    }
}

impl VehicleConnection {
    // fn tick(&mut self) {
    //     println!("tick. location: {:?}", self.vehicle.location_global);
    // }

    pub fn cork(&mut self) -> Vec<MavMessage> {
        self.tx.send(DkHandlerMessage::TxCork).unwrap();

        loop {
            match self.rx.recv() {
                Some(DkHandlerRx::RxCork) => {
                    break;
                }
                Some(DkHandlerRx::RxMessage(msg)) => {
                    self.buffer.push_back(msg);
                }
                _ => {}
            }
        }

        self.buffer.clone().into_iter().collect()
    }

    pub fn uncork(&mut self) {
        self.tx.send(DkHandlerMessage::TxUncork).unwrap();
    }

    pub fn wait_recv(&self) -> VehicleAwait {
        let count = {
            let &(ref lock, _) = &*self.pair;
            *lock.lock().unwrap()
        };
        VehicleAwait {
            value: count,
            pair: self.pair.clone()
        }
    }

    pub fn recv(&mut self) -> Result<MavMessage, ()> {
        loop {
            if let Some(msg) = self.buffer.pop_front() {
                return Ok(msg);
            } else {
                match self.rx.recv() {
                    Some(DkHandlerRx::RxCork) => {
                        continue;
                    }
                    Some(DkHandlerRx::RxMessage(msg)) => {
                        return Ok(msg);
                    }
                    None => {
                        return Err(());
                    }
                }
            }
        }
    }

    pub fn send(&mut self, data: MavMessage) {
        let mut pkt = MavPacket {
            seq: self.msg_id,
            system_id: 255,
            component_id: 0,
            message_id: data.message_id(),
            data: data.serialize(),
            checksum: 0,
        };
        pkt.update_crc();
        let out = pkt.encode();

        self.msg_id = self.msg_id.wrapping_add(1);

        self.tx.send(DkHandlerMessage::TxMessage(out)).unwrap();
    }

    pub fn complete(&mut self,
                    tx: Complete<(), ()>,
                    mut watch: Box<FnMut(MavMessage) -> bool + Send>) {
        let buffer = self.cork();

        if !buffer.into_iter().any(|x| watch(x)) {
            let mut txlock = Some(tx);
            self.tx
                .send(DkHandlerMessage::TxWatcher(Box::new(move |msg| {
                    if watch(msg) {
                        if let Some(tx) = txlock.take() {
                            tx.complete(());
                        }
                        true
                    } else {
                        false
                    }
                })))
                .unwrap();
        } else {
            tx.complete(());
        }

        self.uncork();
    }
}
