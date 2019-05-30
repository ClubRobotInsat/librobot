//! Module pour la communication ethernet

use embedded_hal::spi::FullDuplex;
use w5500::*;

/// La socket utilisee pour l'UDP
pub const SOCKET_UDP: Socket = Socket::Socket0;

fn get_subnet() -> u8 {
    if cfg!(feature = "primary") {
        1
    } else if cfg!(feature = "secondary") {
        2
    } else {
        unreachable!()
    }
}

/// Initialise la connexion ethernet pour permettre une communication
/// a l'aide de la librairie W5500. La socket a utiliser pour lire
/// les message est eth::SOCKET_UDP
pub fn init_eth<E: core::fmt::Debug>(
    eth: &mut W5500,
    spi: &mut FullDuplex<u8, Error = E>,
    mac: u8,
    ip: u8,
) {
    let ip = IpAddress::new(192, 168, get_subnet(), ip);
    let mac = MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04 + get_subnet(), mac);
    //eth.set_mode(spi,false, false, false, true).unwrap();
    // using a 'locally administered' MAC address
    eth.init(spi).expect("Failed to initialize w5500");
    eth.set_mode(spi, false, false, false, true).unwrap();
    eth.set_mac(spi, &mac).unwrap();
    eth.set_ip(spi, &ip).unwrap();
    eth.set_subnet(spi, &IpAddress::new(255, 255, 255, 0))
        .unwrap();
    eth.set_gateway(spi, &IpAddress::new(192, 168, get_subnet(), 254))
        .unwrap();
    //eth.reset_interrupt(spi, SOCKET_UDP, Interrupt::Received)
    //    .expect("Failed ot reset interrupts for W5500");
}

/// Ecoute sur un port avec un socket donné
pub fn listen_on<E: core::fmt::Debug>(
    eth: &mut W5500,
    spi: &mut FullDuplex<u8, Error = E>,
    port: u16,
    socket: Socket,
) {
    eth.listen_udp(spi, socket, port).expect("Failed to listen");
}
