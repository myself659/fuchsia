// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use crate::configuration::{ClientConfig, ServerConfig};
use crate::protocol::{self, ConfigOption, Message, MessageType, OpCode, OptionCode};
use failure::Fail;
use byteorder::{BigEndian, ByteOrder};
use fidl_fuchsia_hardware_ethernet_ext::MacAddress as MacAddr;
use std::cmp;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::net::Ipv4Addr;
use std::ops::Fn;


/// A minimal DHCP server.
///
/// This comment will be expanded upon in future CLs as the server design
/// is iterated upon.
pub struct Server<F>
where
    F: Fn() -> i64,
{
    cache: CachedClients,
    pool: AddressPool,
    config: ServerConfig,
    time_provider: F,
}

// A wrapper around the error types which can be returned
// by DHCP Server in response to client requests
#[derive(Debug, Fail, PartialEq)]
pub enum ServerError {
    #[fail(display = "Received Client Message Type [{:?}] is invalid.", _0)]
    InvalidClientMessage(MessageType),

    #[fail(display = "Requested Ipv4 Address by client could not be retrieved : {}", _0)]
    BadRequestedIpv4Addr(String),

    #[fail(display = "Error occurred in Server Address Pool : {}", _0)]
    ServerAddressPoolFailure(AddressPoolError),

    #[fail(display = "Unable to identify Client Request Type")]
    UnknownClientRequest,

    #[fail(display = "Unknown Client State")]
    UnknownClientState,

    #[fail(display = "Server is not the intended target for this client request.")]
    UnwantedDHCPServer,

    #[fail(display = "Unable to assign the requested ipv4 address to client : {}", _0)]
    RequestedAddrAllocationFailure(String),

    #[fail(display = "Requested Ipv4Addr not found")]
    RequestedIpv4AddrNotFound,

    #[fail(display = "Invalid Client Request : {}", _0)]
    BadClientRequest(String),

    #[fail(display = "Error processing Client Request : {}", _0)]
    DhcpError(String)
}

//TODO (sshrivy) - this should be implemented for ALL
// different kind of Error's that can be generated anywhere in the server.rs file
// for all functions whose return type is Result<T, E>
// Add a methos impl From<T> for ServerError
impl From<AddressPoolError> for ServerError {
    fn from(e: AddressPoolError) -> Self {
        ServerError::ServerAddressPoolFailure(e)
    }
}

// TODO(sshrivy) The below is to handle Option None return as Error for Return<T,E>
// in short to ACTUALLY MAKE THIS ENUM AN ERROR
/*
impl Error for ServerError {

}
^Apparently not needed as Fail crate already implements trait Error
*/


impl<F> Server<F>
where
    F: Fn() -> i64,
{
    /// Returns an initialized `Server` value.
    pub fn new(time_provider: F) -> Server<F> {
        Server {
            cache: HashMap::new(),
            pool: AddressPool::new(),
            config: ServerConfig::new(),
            time_provider: time_provider,
        }
    }

    /// Instantiates a `Server` value from the provided `ServerConfig`.
    pub fn from_config(config: ServerConfig, time_provider: F) -> Server<F> {
        let mut server = Server {
            cache: HashMap::new(),
            pool: AddressPool::new(),
            config: config,
            time_provider: time_provider,
        };
        server.pool.load_pool(&server.config.managed_addrs);
        server
    }

    /// Dispatches an incoming DHCP message to the appropriate handler for processing.
    ///
    /// If the incoming message is a valid client DHCP message, then the server will attempt to
    /// take appropriate action to serve the client's request, update the internal server state,
    /// and return a response message. If the incoming message is invalid, or the server is
    /// unable to serve the request, then `dispatch()` will return `None`.
    pub fn dispatch(&mut self, msg: Message) -> Result<Message, ServerError> {
      //  fx_syslog::init_with_tags(&["dhcpd"])?;
        match msg.get_dhcp_type() {
            Some(MessageType::DHCPDISCOVER) => self.handle_discover(msg),
            Some(MessageType::DHCPOFFER) => Err(ServerError::InvalidClientMessage(MessageType::DHCPOFFER)),
            Some(MessageType::DHCPREQUEST) => self.handle_request(msg),
            //Some(MessageType::DHCPDECLINE) => self.handle_decline(msg),
            Some(MessageType::DHCPACK) => Err(ServerError::InvalidClientMessage(MessageType::DHCPACK)),
            Some(MessageType::DHCPNAK) => Err(ServerError::InvalidClientMessage(MessageType::DHCPNAK)),
            //Some(MessageType::DHCPRELEASE) => self.handle_release(msg),
            //Some(MessageType::DHCPINFORM) => self.handle_inform(msg),
            //None => Err(ServerError::UnknownClientRequest),

            _ => Err(ServerError::UnknownClientRequest)
        }
    }

    fn handle_discover(&mut self, disc: Message) -> Result<Message, ServerError> {
        let client_config = self.client_config(&disc);
        let offered_ip = self.get_addr(&disc)?;
        let mut offer = build_offer(disc, &self.config, &client_config);
        offer.yiaddr = offered_ip;
        let () = self.update_server_cache(
            Ipv4Addr::from(offer.yiaddr),
            offer.chaddr,
            vec![],
            &client_config,
        )?;

        Ok(offer)
    }

    fn get_addr(&mut self, client: &Message) -> Result<Ipv4Addr, ServerError> {
        if let Some(config) = self.cache.get(&client.chaddr) {
            if !config.expired((self.time_provider)()) {
                println!("CONFIG NOT EXPIRED");
                // Free cached address so that it can be reallocated to same client.
                let () = self.pool.free_addr(config.client_addr)?;
                return Ok(config.client_addr);
            } else if self.pool.addr_is_available(config.client_addr) {
                return Ok(config.client_addr);
            }
        }
        if let Some(opt) = client.get_config_option(OptionCode::RequestedIpAddr) {
            println!("THIS SHOULDNT run as client isnt requesting an addr here");
            if opt.value.len() >= 4 {
                let requested_addr = protocol::ip_addr_from_buf_at(&opt.value, 0)
                    .ok_or_else(|| ServerError::BadRequestedIpv4Addr("out of range indexing on opt.value".to_owned())).unwrap();
                if self.pool.addr_is_available(requested_addr) {
                    return Ok(requested_addr);
                }
            }
        }
        let addr = self.pool.get_next_available_addr()?;
        Ok(addr)
    }

    fn update_server_cache(
        &mut self,
        client_addr: Ipv4Addr,
        client_mac: MacAddr,
        client_opts: Vec<ConfigOption>,
        client_config: &ClientConfig,
    ) -> Result<(), ServerError> {
        let config = CachedConfig {
            client_addr: client_addr,
            options: client_opts,
            expiration: (self.time_provider)() + client_config.lease_time_s as i64,
        };
        self.cache.insert(client_mac, config);
        let () = self.pool.allocate_addr(client_addr)?;
        Ok(())
    }

    fn handle_request(&mut self, req: Message) -> Result<Message, ServerError> {
        match get_client_state(&req) {
            ClientState::Selecting => self.handle_request_selecting(req),
            ClientState::InitReboot => self.handle_request_init_reboot(req),
           // ClientState::Renewing => self.handle_request_renewing(req),
            _ => Err(ServerError::UnknownClientState),


                        //ClientState::Unknown => None,*/
        }
    }

    fn handle_request_selecting(&mut self, req: Message) -> Result<Message, ServerError> {
        let requested_ip = req.ciaddr;
        if !is_recipient(self.config.server_ip, &req) {
            Err(ServerError::UnwantedDHCPServer)
        } else {
            let () = self.is_assigned(&req, requested_ip)?;
            Ok(build_ack(req, requested_ip, &self.config))
        }
        /*
          let requested_ip = req.ciaddr;
        if !is_recipient(self.config.server_ip, &req) || !self.is_assigned(&req, requested_ip) {
            return Err(ServerError::IncorrectDHCP);
        }
        Some(build_ack(req, requested_ip, &self.config))

        */
    }

    fn is_assigned(&self, req: &Message, requested_ip: Ipv4Addr) -> Result<(), ServerError> {
        if let Some(client_config) = self.cache.get(&req.chaddr) {
            if client_config.client_addr != requested_ip {
                Err(ServerError::RequestedAddrAllocationFailure("Requested Ipv4 \
                address does not match initial address requested in discovery".to_owned()))
            } else if client_config.expired((self.time_provider)()) {
                Err(ServerError::RequestedAddrAllocationFailure("Client Config has expired".to_owned()))
            } else if !self.pool.addr_is_allocated(requested_ip) {
                Err(ServerError::RequestedAddrAllocationFailure("Server failed to reserve requested Ipv4 adddress".to_owned()))
            } else {
                Ok(())
            }
        } else {
            Err(ServerError::RequestedAddrAllocationFailure("Could not retrieve client config".to_owned()))
        }
/*        if let Some(client_config) = self.cache.get(&req.chaddr) {
            client_config.client_addr == requested_ip
                && !client_config.expired((self.time_provider)())
                && self.pool.addr_is_allocated(requested_ip)
        } else {
            false
        }*/
    }

    fn handle_request_init_reboot(&mut self, req: Message) -> Result<Message, ServerError> {
        let requested_ip = get_requested_ip_addr(&req).ok_or_else(|| ServerError::RequestedIpv4AddrNotFound).unwrap();
        if !is_in_subnet(requested_ip, &self.config) {
            return Ok(build_nak(req, &self.config));
        }
        if !is_client_mac_known(req.chaddr, &self.cache) {
            return Err(ServerError::RequestedIpv4AddrNotFound);
        }
        if !self.is_assigned(&req, requested_ip) {
            return Ok(build_nak(req, &self.config));
        }
        Ok(build_ack(req, requested_ip, &self.config))
    }

/*    fn handle_request_renewing(&mut self, req: Message) -> Option<Message> {
        let client_ip = req.ciaddr;
        if !self.is_assigned(&req, client_ip) {
            return None;
        }
        Some(build_ack(req, client_ip, &self.config))
    }*/

/*    fn handle_decline(&mut self, dec: Message) -> Option<Message> {
        let declined_ip = get_requested_ip_addr(&dec)?;
        if is_recipient(self.config.server_ip, &dec) && !self.is_assigned(&dec, declined_ip) {
            self.pool.allocate_addr(declined_ip);
        }
        self.cache.remove(&dec.chaddr);
        None
    }*/

/*    fn handle_release(&mut self, rel: Message) -> Option<Message> {
        if self.cache.contains_key(&rel.chaddr) {
            self.pool.free_addr(rel.ciaddr);
        }
        None
    }*/

/*    fn handle_inform(&mut self, inf: Message) -> Option<Message> {
        // When responding to an INFORM, the server must leave yiaddr zeroed.
        let yiaddr = Ipv4Addr::new(0, 0, 0, 0);
        let mut ack = build_ack(inf, yiaddr, &self.config);
        ack.options.clear();
        add_inform_ack_options(&mut ack, &self.config);
        Some(ack)
    }*/

    /// Releases all allocated IP addresses whose leases have expired back to
    /// the pool of addresses available for allocation.
    pub fn release_expired_leases(&mut self) {
        let now = (self.time_provider)();
        let expired_clients: Vec<(MacAddr, Ipv4Addr)> = self
            .cache
            .iter()
            .filter(|(_mac, config)| config.expired(now))
            .map(|(mac, config)| (*mac, config.client_addr))
            .collect();
        // Expired client entries must be removed in a separate statement because otherwise we
        // would be attempting to change a cache as we iterate over it.
        expired_clients.iter().for_each(|(mac, ip)| {
            match self.pool.free_addr(*ip) {
                Ok(()) => println!("Done"),
                Err(_e) => println!("Failed to free")
            };
            self.cache.remove(mac);
        });
    }

    pub fn client_config(&self, client_message: &Message) -> ClientConfig {
        println!("->>> Inside Client config");
        let requested_config = client_message.parse_to_config();
        ClientConfig {
            lease_time_s: match requested_config.lease_time_s {
                None => self.config.default_lease_time,
                Some(t) => cmp::min(t, self.config.max_lease_time_s),
            },
        }
    }
}

/// A cache mapping clients to their configuration data.
///
/// The server should store configuration data for all clients
/// to which it has sent a DHCPOFFER message. Entries in the cache
/// will eventually timeout, although such functionality is currently
/// unimplemented.
type CachedClients = HashMap<MacAddr, CachedConfig>;

#[derive(Clone, Debug, PartialEq)]
struct CachedConfig {
    client_addr: Ipv4Addr,
    options: Vec<ConfigOption>,
    expiration: i64,
}

impl Default for CachedConfig {
    fn default() -> Self {
        CachedConfig {
            client_addr: Ipv4Addr::new(0, 0, 0, 0),
            options: vec![],
            expiration: std::i64::MAX,
        }
    }
}

impl CachedConfig {
    fn expired(&self, now: i64) -> bool {
        self.expiration <= now
    }
}

/// The pool of addresses managed by the server.
///
/// Any address managed by the server should be stored in only one
/// of the available/allocated sets at a time. In other words, an
/// address in `available_addrs` must not be in `allocated_addrs` and
/// vice-versa.
#[derive(Debug)]
struct AddressPool {
    // available_addrs uses a BTreeSet so that addresses are allocated
    // in a deterministic order.
    available_addrs: BTreeSet<Ipv4Addr>,
    allocated_addrs: HashSet<Ipv4Addr>,
}

//This is a wrapper around different errors that could be returned by
// the DHCP server address pool during address allocation/de-allocation
#[derive(Debug, Fail, PartialEq)]
pub enum AddressPoolError {

    #[fail(display = "Address Pool does not have any more available Ipv4 addresses")]
    NoMoreAvailableIpv4Addr,

    #[fail(display = "Invalid Server State: attempted to allocate unavailable address : {}", _0)]
    UnavailableIpv4AddrAllocation(Ipv4Addr),

    #[fail(display = "Invalid Server State: attempted to free unallocated address : {}", _0)]
    UnallocatedIpv4AddrDeallocation(Ipv4Addr),
}

impl AddressPool {
    fn new() -> Self {
        AddressPool { available_addrs: BTreeSet::new(), allocated_addrs: HashSet::new() }
    }

    fn load_pool(&mut self, addrs: &[Ipv4Addr]) {
        for addr in addrs {
            if !self.allocated_addrs.contains(&addr) {
                self.available_addrs.insert(*addr);
            }
        }
    }

    fn get_next_available_addr(&self) -> Result<Ipv4Addr, AddressPoolError> {
        let mut iter = self.available_addrs.iter();
        match iter.next() {
            Some(addr) => Ok(*addr),
            None => Err(AddressPoolError::NoMoreAvailableIpv4Addr),
        }
    }

    fn allocate_addr(&mut self, addr: Ipv4Addr) -> Result<(), AddressPoolError>  {
        if self.available_addrs.remove(&addr) {
            self.allocated_addrs.insert(addr);
            Ok(())
        } else {
            Err(AddressPoolError::UnavailableIpv4AddrAllocation(addr))
            //panic!("Invalid Server State: attempted to allocate unavailable address");
        }
    }

    fn free_addr(&mut self, addr: Ipv4Addr) -> Result<(), AddressPoolError> {
        if self.allocated_addrs.remove(&addr) {
            self.available_addrs.insert(addr);
            Ok(())
        } else {
            Err(AddressPoolError::UnallocatedIpv4AddrDeallocation(addr))
            //panic!("Invalid Server State: attempted to free unallocated address");
        }
    }

    fn addr_is_available(&self, addr: Ipv4Addr) -> bool {
        self.available_addrs.contains(&addr) && !self.allocated_addrs.contains(&addr)
    }

    fn addr_is_allocated(&self, addr: Ipv4Addr) -> bool {
        !self.available_addrs.contains(&addr) && self.allocated_addrs.contains(&addr)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ClientState {
    Unknown,
    Selecting,
    InitReboot,
    Renewing,
}

fn build_offer(client: Message, config: &ServerConfig, client_config: &ClientConfig) -> Message {
    let mut offer = client;
    offer.op = OpCode::BOOTREPLY;
    offer.secs = 0;
    offer.ciaddr = Ipv4Addr::new(0, 0, 0, 0);
    offer.siaddr = Ipv4Addr::new(0, 0, 0, 0);
    offer.sname = String::new();
    offer.file = String::new();
    add_required_options(&mut offer, config, client_config, MessageType::DHCPOFFER);
    add_recommended_options(&mut offer, config);

    offer
}

fn add_required_options(
    msg: &mut Message,
    config: &ServerConfig,
    client_config: &ClientConfig,
    msg_type: MessageType,
) {
    msg.options.clear();
    let mut lease = vec![0; 4];
    BigEndian::write_u32(&mut lease, client_config.lease_time_s);
    msg.options.push(ConfigOption { code: OptionCode::IpAddrLeaseTime, value: lease });
    msg.options.push(ConfigOption {
        code: OptionCode::SubnetMask,
        value: config.subnet_mask.octets().to_vec(),
    });
    msg.options
        .push(ConfigOption { code: OptionCode::DhcpMessageType, value: vec![msg_type.into()] });
    msg.options.push(ConfigOption {
        code: OptionCode::ServerId,
        value: config.server_ip.octets().to_vec(),
    });
}

fn add_recommended_options(msg: &mut Message, config: &ServerConfig) {
    msg.options
        .push(ConfigOption { code: OptionCode::Router, value: ip_vec_to_bytes(&config.routers) });
    msg.options.push(ConfigOption {
        code: OptionCode::NameServer,
        value: ip_vec_to_bytes(&config.name_servers),
    });
    let mut renewal_time = vec![0, 0, 0, 0];
    BigEndian::write_u32(&mut renewal_time, config.default_lease_time / 2);
    msg.options.push(ConfigOption { code: OptionCode::RenewalTime, value: renewal_time });
    let mut rebinding_time = vec![0, 0, 0, 0];
    BigEndian::write_u32(&mut rebinding_time, config.default_lease_time / 4);
    msg.options.push(ConfigOption { code: OptionCode::RebindingTime, value: rebinding_time });
}

/*fn add_inform_ack_options(msg: &mut Message, config: &ServerConfig) {
    msg.options.push(ConfigOption {
        code: OptionCode::DhcpMessageType,
        value: vec![MessageType::DHCPINFORM.into()],
    });
    msg.options.push(ConfigOption {
        code: OptionCode::ServerId,
        value: config.server_ip.octets().to_vec(),
    });
    msg.options
        .push(ConfigOption { code: OptionCode::Router, value: ip_vec_to_bytes(&config.routers) });
    msg.options.push(ConfigOption {
        code: OptionCode::NameServer,
        value: ip_vec_to_bytes(&config.name_servers),
    });
}*/

fn ip_vec_to_bytes<'a, T>(ips: T) -> Vec<u8>
where
    T: IntoIterator<Item = &'a Ipv4Addr>,
{
    ips.into_iter().flat_map(|ip| ip.octets().to_vec()).collect()
}

fn is_recipient(server_ip: Ipv4Addr, req: &Message) -> bool {
    if let Some(server_id) = get_server_id_from(&req) {
        return server_id == server_ip;
    }
    false
}

fn build_ack(req: Message, requested_ip: Ipv4Addr, config: &ServerConfig) -> Message {
    let mut ack = req;
    ack.op = OpCode::BOOTREPLY;
    ack.secs = 0;
    ack.yiaddr = requested_ip;
    ack.options.clear();
    add_required_options(
        &mut ack,
        config,
        &ClientConfig::new(config.default_lease_time),
        MessageType::DHCPACK,
    );
    add_recommended_options(&mut ack, config);

    ack
}

fn is_in_subnet(ip: Ipv4Addr, config: &ServerConfig) -> bool {
    config.subnet_mask.apply_to(ip) == config.subnet_mask.apply_to(config.server_ip)
}

fn is_client_mac_known(mac: MacAddr, cache: &CachedClients) -> bool {
    cache.get(&mac).is_some()
}

fn build_nak(req: Message, config: &ServerConfig) -> Message {
    let mut nak = req;
    nak.op = OpCode::BOOTREPLY;
    nak.secs = 0;
    nak.ciaddr = Ipv4Addr::new(0, 0, 0, 0);
    nak.yiaddr = Ipv4Addr::new(0, 0, 0, 0);
    nak.siaddr = Ipv4Addr::new(0, 0, 0, 0);
    nak.options.clear();
    let mut lease = vec![0; 4];
    BigEndian::write_u32(&mut lease, config.default_lease_time);
    nak.options.push(ConfigOption {
        code: OptionCode::DhcpMessageType,
        value: vec![MessageType::DHCPNAK.into()],
    });
    nak.options.push(ConfigOption {
        code: OptionCode::ServerId,
        value: config.server_ip.octets().to_vec(),
    });

    nak
}

fn get_client_state(msg: &Message) -> ClientState {
    let maybe_server_id = get_server_id_from(&msg);
    let maybe_requested_ip = get_requested_ip_addr(&msg);
    let zero_ciaddr = Ipv4Addr::new(0, 0, 0, 0);

    if maybe_server_id.is_some() && maybe_requested_ip.is_none() && msg.ciaddr != zero_ciaddr {
        return ClientState::Selecting;
    } else if maybe_requested_ip.is_some() && msg.ciaddr == zero_ciaddr {
        return ClientState::InitReboot;
    } else if msg.ciaddr != zero_ciaddr {
        return ClientState::Renewing;
    } else {
        return ClientState::Unknown;
    }
}

fn get_requested_ip_addr(req: &Message) -> Option<Ipv4Addr> {
    let req_ip_opt = req.options.iter().find(|opt| opt.code == OptionCode::RequestedIpAddr)?;
    let raw_ip = BigEndian::read_u32(&req_ip_opt.value);
    Some(Ipv4Addr::from(raw_ip))
}

fn get_server_id_from(req: &Message) -> Option<Ipv4Addr> {
    let server_id_opt = req.options.iter().find(|opt| opt.code == OptionCode::ServerId)?;
    let raw_server_id = BigEndian::read_u32(&server_id_opt.value);
    Some(Ipv4Addr::from(raw_server_id))
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::configuration::SubnetMask;
    use crate::protocol::{ConfigOption, Message, MessageType, OpCode, OptionCode};
    use std::convert::TryFrom;
    use std::net::Ipv4Addr;

    fn new_test_server<F>(time_provider: F) -> Server<F>
    where
        F: Fn() -> i64,
    {
        let mut server = Server::new(time_provider);
        server.config.server_ip = Ipv4Addr::new(192, 168, 1, 1);
        server.config.default_lease_time = 100;
        server.config.routers.push(Ipv4Addr::new(192, 168, 1, 1));
        server
            .config
            .name_servers
            .extend_from_slice(&vec![Ipv4Addr::new(8, 8, 8, 8), Ipv4Addr::new(8, 8, 4, 4)]);
        server.pool.available_addrs.insert(Ipv4Addr::from([192, 168, 1, 2]));
        server
    }

    fn new_test_discover() -> Message {
        let mut disc = Message::new();
        disc.xid = 42;
        disc.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        disc.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPDISCOVER.into()],
        });
        disc
    }

    fn new_test_client_offer() -> Message {
        let mut client_offer = Message::new();
        client_offer.op = OpCode::BOOTREQUEST;
        client_offer.xid = 42;
        client_offer.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        client_offer.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPOFFER.into()],
        });
        // Skipping other settings as they are not needed
        client_offer
    }

    fn new_test_offer() -> Message {
        let mut offer = Message::new();
        offer.op = OpCode::BOOTREPLY;
        offer.xid = 42;
        offer.yiaddr = Ipv4Addr::new(192, 168, 1, 2);
        offer.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        offer
            .options
            .push(ConfigOption { code: OptionCode::IpAddrLeaseTime, value: vec![0, 0, 0, 100] });
        offer.options.push(ConfigOption {
            code: OptionCode::SubnetMask,
            value: SubnetMask::try_from(24).unwrap().octets().to_vec(),
        });
        offer.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPOFFER.into()],
        });
        offer
            .options
            .push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
        offer.options.push(ConfigOption { code: OptionCode::Router, value: vec![192, 168, 1, 1] });
        offer.options.push(ConfigOption {
            code: OptionCode::NameServer,
            value: vec![8, 8, 8, 8, 8, 8, 4, 4],
        });
        offer
            .options
            .push(ConfigOption { code: OptionCode::RenewalTime, value: vec![0, 0, 0, 50] });
        offer
            .options
            .push(ConfigOption { code: OptionCode::RebindingTime, value: vec![0, 0, 0, 25] });
        offer
    }

    fn new_test_request() -> Message {
        let mut req = Message::new();
        req.xid = 42;

        req.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        req.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1, 2] });
        req.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPREQUEST.into()],
        });
        req.options.push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
        req
    }

    fn new_test_client_ack() -> Message {
        let mut client_ack = Message::new();
        client_ack.op = OpCode::BOOTREQUEST;
        client_ack.xid = 42;
        client_ack.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        client_ack.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPACK.into()],
        });
        client_ack
        // Skipping other option settings as they are not needed
    }

   fn new_test_ack() -> Message {
       let mut ack = Message::new();
       ack.op = OpCode::BOOTREPLY;
       ack.xid = 42;
       ack.yiaddr = Ipv4Addr::new(192, 168, 1, 2);
       ack.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
       ack.options
           .push(ConfigOption { code: OptionCode::IpAddrLeaseTime, value: vec![0, 0, 0, 100] });
       ack.options.push(ConfigOption {
           code: OptionCode::SubnetMask,
           value: SubnetMask::try_from(24).unwrap().octets().to_vec(),
       });
       ack.options.push(ConfigOption {
           code: OptionCode::DhcpMessageType,
           value: vec![MessageType::DHCPACK.into()],
       });
       ack.options.push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
       ack.options.push(ConfigOption { code: OptionCode::Router, value: vec![192, 168, 1, 1] });
       ack.options.push(ConfigOption {
           code: OptionCode::NameServer,
           value: vec![8, 8, 8, 8, 8, 8, 4, 4],
       });
       ack.options.push(ConfigOption { code: OptionCode::RenewalTime, value: vec![0, 0, 0, 50] });
       ack.options
           .push(ConfigOption { code: OptionCode::RebindingTime, value: vec![0, 0, 0, 25] });
       ack
   }

    fn new_test_client_nak() -> Message {
        let mut client_nak = Message::new();
        client_nak.op = OpCode::BOOTREQUEST;
        client_nak.xid = 42;
        client_nak.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        client_nak.options.push(ConfigOption {
            code: OptionCode::DhcpMessageType,
            value: vec![MessageType::DHCPNAK.into()],
        });
        client_nak
    }

   fn new_test_nak() -> Message {
       let mut nak = Message::new();
       nak.op = OpCode::BOOTREPLY;
       nak.xid = 42;
       nak.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
       nak.options.push(ConfigOption {
           code: OptionCode::DhcpMessageType,
           value: vec![MessageType::DHCPNAK.into()],
       });
       nak.options.push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
       nak
   }

   /*    fn new_test_release() -> Message {
           let mut release = Message::new();
           release.xid = 42;
           release.ciaddr = Ipv4Addr::new(192, 168, 1, 2);
           release.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
           release.options.push(ConfigOption {
               code: OptionCode::DhcpMessageType,
               value: vec![MessageType::DHCPRELEASE.into()],
           });
           release
       }

       fn new_test_inform() -> Message {
           let mut inform = Message::new();
           inform.xid = 42;
           inform.ciaddr = Ipv4Addr::new(192, 168, 1, 2);
           inform.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
           inform.options.push(ConfigOption {
               code: OptionCode::DhcpMessageType,
               value: vec![MessageType::DHCPINFORM.into()],
           });
           inform
       }

       fn new_test_inform_ack() -> Message {
           let mut ack = Message::new();
           ack.op = OpCode::BOOTREPLY;
           ack.xid = 42;
           ack.ciaddr = Ipv4Addr::new(192, 168, 1, 2);
           ack.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
           ack.options.push(ConfigOption {
               code: OptionCode::DhcpMessageType,
               value: vec![MessageType::DHCPINFORM.into()],
           });
           ack.options.push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
           ack.options.push(ConfigOption { code: OptionCode::Router, value: vec![192, 168, 1, 1] });
           ack.options.push(ConfigOption {
               code: OptionCode::NameServer,
               value: vec![8, 8, 8, 8, 8, 8, 4, 4],
           });
           ack
       }

       fn new_test_decline() -> Message {
           let mut decline = Message::new();
           decline.xid = 42;
           decline.chaddr = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
           decline.options.push(ConfigOption {
               code: OptionCode::DhcpMessageType,
               value: vec![MessageType::DHCPDECLINE.into()],
           });
           decline
               .options
               .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1, 2] });
           decline
               .options
               .push(ConfigOption { code: OptionCode::ServerId, value: vec![192, 168, 1, 1] });
           decline
       }*/

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_returns_correct_response() {
        let disc = new_test_discover();

        let mut server = new_test_server(|| 42);
        let got = server.dispatch(disc).unwrap();

        let want = new_test_offer();

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_updates_server_state() {
        println!("hello world");

        let disc = new_test_discover();
        let mac_addr = disc.chaddr;
        let mut server = new_test_server(|| 42);
        let _ = server.dispatch(disc).unwrap();

        assert_eq!(server.pool.available_addrs.len(), 0);
        assert_eq!(server.pool.allocated_addrs.len(), 1);
        assert_eq!(server.cache.len(), 1);
        let want_config = server.cache.get(&mac_addr).unwrap();
        assert_eq!(want_config.client_addr, Ipv4Addr::new(192, 168, 1, 2));
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_client_binding_returns_bound_addr() {
        let disc = new_test_discover();
        let mut server = new_test_server(|| 42);
        let mut client_config = CachedConfig::default();
        let client_addr = Ipv4Addr::new(192, 168, 1, 42);
        client_config.client_addr = client_addr;
        server.pool.allocated_addrs.insert(client_addr);
        server.cache.insert(disc.chaddr, client_config);

        let got = server.dispatch(disc).unwrap();

        let mut want = new_test_offer();
        want.yiaddr = Ipv4Addr::new(192, 168, 1, 42);

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_expired_client_binding_returns_available_old_addr() {
        let disc = new_test_discover();
        let mut server = new_test_server(|| 42);
        let mut client_config = CachedConfig::default();
        client_config.client_addr = Ipv4Addr::new(192, 168, 1, 42);
        client_config.expiration = 0;
        server.cache.insert(disc.chaddr, client_config);
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 42));

        let got = server.dispatch(disc).unwrap();

        let mut want = new_test_offer();
        want.yiaddr = Ipv4Addr::new(192, 168, 1, 42);

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_unavailable_expired_client_binding_returns_new_addr() {
        let disc = new_test_discover();
        let mut server = new_test_server(|| 42);
        let mut client_config = CachedConfig::default();
        client_config.client_addr = Ipv4Addr::new(192, 168, 1, 42);
        client_config.expiration = 0;
        server.cache.insert(disc.chaddr, client_config);
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 42));

        let got = server.dispatch(disc).unwrap();

        let mut want = new_test_offer();
        want.yiaddr = Ipv4Addr::new(192, 168, 1, 2);

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_available_requested_addr_returns_requested_addr() {
        let mut disc = new_test_discover();
        disc.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1, 3] });

        let mut server = new_test_server(|| 42);
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 3));
        let got = server.dispatch(disc).unwrap();

        let mut want = new_test_offer();

        want.yiaddr = Ipv4Addr::new(192, 168, 1, 3);

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_bad_requested_addr_returns_error() {
        let mut disc = new_test_discover();
        disc.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1] });

        let mut server = new_test_server(|| 42);

        let got = server.dispatch(disc);

        let want : Result<Message, ServerError> = Err(ServerError::BadRequestedIpv4Addr("out of range indexing on opt.value".to_owned()));

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_discover_unavailable_requested_addr_returns_next_addr() {
        let mut disc = new_test_discover();
        disc.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1, 42] });

        let mut server = new_test_server(|| 42);
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.pool.available_addrs.insert(Ipv4Addr::new(192, 168, 1, 3));
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 42));
        let got = server.dispatch(disc).unwrap();

        let mut want = new_test_offer();
        want.yiaddr = Ipv4Addr::new(192, 168, 1, 2);
        assert_eq!(got, want);
    }

    #[test]
    fn test_dispatch_with_discover_no_available_addr_returns_error() {
        let disc = new_test_discover();
        let mut server = new_test_server(|| 42);

        server.pool.available_addrs.clear();
        let got = server.dispatch(disc);

        let want : Result<Message, ServerError> = Err(ServerError::ServerAddressPoolFailure(AddressPoolError::NoMoreAvailableIpv4Addr));
        assert_eq!(got, want);
    }

    #[test]
    fn test_dispatch_with_client_offer_message_returns_error() {
        let client_offer = new_test_client_offer();

        let mut server = new_test_server(|| 42);

        let got = server.dispatch(client_offer);

        let want : Result<Message, ServerError> = Err(ServerError::InvalidClientMessage(MessageType::DHCPOFFER));
        assert_eq!(got, want);
    }

    #[test]
    fn test_dispatch_with_client_ack_message_returns_error() {
        let client_ack = new_test_client_ack();

        let mut server = new_test_server(|| 42);

        let got = server.dispatch(client_ack);

        let want : Result<Message, ServerError> = Err(ServerError::InvalidClientMessage(MessageType::DHCPACK));
        assert_eq!(got, want);
    }

    #[test]
    fn test_dispatch_with_client_nak_message_returns_error() {
        let client_nak = new_test_client_nak();

        let mut server = new_test_server(|| 42);

        let got = server.dispatch(client_nak);

        let want : Result<Message, ServerError> = Err(ServerError::InvalidClientMessage(MessageType::DHCPNAK));

        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_selecting_request_valid_selecting_request_returns_ack() {
        let mut req = new_test_request();
        let requested_ip_addr = Ipv4Addr::new(192, 168, 1, 2);
        req.ciaddr = requested_ip_addr;
        req.options.remove(0);

        let mut server = new_test_server(|| 42);
        server.cache.insert(
            req.chaddr,
            CachedConfig {
                client_addr: requested_ip_addr,
                options: vec![],
                expiration: std::i64::MAX,
            },
        );

        let _ = server.pool.allocate_addr(requested_ip_addr);

        let got = server.dispatch(req).unwrap();

        let mut want = new_test_ack();
        want.ciaddr = requested_ip_addr;
        assert_eq!(got, want);
    }


    #[test]
    #[ignore]
    fn test_dispatch_with_selecting_request_no_address_allocation_to_client_returns_none() {
        let mut req = new_test_request();
        req.ciaddr = Ipv4Addr::new(192, 168, 1, 2);
        req.options.remove(0);

        let mut server = new_test_server(|| 42);
        let got = server.dispatch(req);

        assert!(got.is_err());
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_selecting_request_wrong_server_id_returns_none() {
        let mut req = new_test_request();
        let requested_ip_addr = Ipv4Addr::new(192, 168, 1, 2);
        req.ciaddr = requested_ip_addr;
        req.options.remove(0);

        let mut server = new_test_server(|| 42);
        server.cache.insert(
            req.chaddr,
            CachedConfig {
                client_addr: requested_ip_addr,
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        let _ = server.pool.allocate_addr(requested_ip_addr);

        server.config.server_ip = Ipv4Addr::new(1, 2, 3, 4);
        let got = server.dispatch(req);

        assert!(got.is_err());
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_selecting_request_valid_selecting_request_maintains_server_invariants() {
        let requested_ip_addr = Ipv4Addr::new(192, 168, 1, 2);
        let mut req = new_test_request();
        req.ciaddr = requested_ip_addr;
        req.options.remove(0);

        let mut server = new_test_server(|| 42);
        server.cache.insert(
            req.chaddr,
            CachedConfig {
                client_addr: requested_ip_addr,
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        let _ = server.pool.allocate_addr(requested_ip_addr);

        let _ = server.dispatch(req.clone()).unwrap();

        assert!(server.cache.contains_key(&req.chaddr));
        assert!(server.pool.addr_is_allocated(requested_ip_addr));
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_selecting_request_no_address_allocation_maintains_server_invariants() {
        let requested_ip_addr = Ipv4Addr::new(192, 168, 1, 2);
        let mut req = new_test_request();
        req.ciaddr = requested_ip_addr;
        req.options.remove(0);

        let mut server = new_test_server(|| 42);
        let _ = server.dispatch(req.clone());

        assert!(!server.cache.contains_key(&req.chaddr));
        assert!(!server.pool.addr_is_allocated(Ipv4Addr::new(192, 168, 1, 2)));
    }


    #[test]
    #[ignore]
    fn test_dispatch_with_init_boot_request_correct_address_returns_ack() {
        let mut req = new_test_request();
        req.options.remove(2);
        let requested_ip_addr = get_requested_ip_addr(&req).unwrap();

        let mut server = new_test_server(|| 42);
        server.cache.insert(
            req.chaddr,
            CachedConfig {
                client_addr: requested_ip_addr,
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        let _ = server.pool.allocate_addr(requested_ip_addr);

        let got = server.dispatch(req).unwrap();

        let want = new_test_ack();
        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_init_boot_request_incorrect_address_returns_nak() {
        let mut req = new_test_request();
        req.options.remove(0);
        req.options.remove(1);
        req.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![192, 168, 1, 42] });

        let mut server = new_test_server(|| 42);
        let assigned_ip = Ipv4Addr::new(192, 168, 1, 2);
        server.cache.insert(
            req.chaddr,
            CachedConfig { client_addr: assigned_ip, options: vec![], expiration: std::i64::MAX },
        );
        let _ = server.pool.allocate_addr(assigned_ip);

        let got = server.dispatch(req).unwrap();

        let want = new_test_nak();
        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_init_boot_request_unknown_client_returns_none() {
        let mut req = new_test_request();
        req.options.remove(2);

        let mut server = new_test_server(|| 42);
        let got = server.dispatch(req);

        assert!(got.is_err());
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_init_boot_request_client_on_wrong_subnet_returns_nak() {
        let mut req = new_test_request();
        req.options.remove(0);
        req.options.remove(1);
        req.options
            .push(ConfigOption { code: OptionCode::RequestedIpAddr, value: vec![10, 0, 0, 1] });

        let mut server = new_test_server(|| 42);
        let got = server.dispatch(req).unwrap();

        let want = new_test_nak();
        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_renewing_request_valid_request_returns_ack() {
        let mut req = new_test_request();
        req.options.remove(0);
        req.options.remove(1);
        let client_ip = Ipv4Addr::new(192, 168, 1, 2);
        req.ciaddr = client_ip;

        let mut server = new_test_server(|| 42);
        server.cache.insert(
            req.chaddr,
            CachedConfig { client_addr: client_ip, options: vec![], expiration: std::i64::MAX },
        );
        let _ = server.pool.allocate_addr(client_ip);

        let got = server.dispatch(req).unwrap();

        let mut want = new_test_ack();
        want.ciaddr = client_ip;
        assert_eq!(got, want);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_renewing_request_unknown_client_returns_none() {
        let mut req = new_test_request();
        req.options.remove(0);
        req.options.remove(1);
        let client_ip = Ipv4Addr::new(192, 168, 1, 2);
        req.ciaddr = client_ip;

        let mut server = new_test_server(|| 42);
        let got = server.dispatch(req);

        assert!(got.is_err());
    }

/*    #[test]
    #[ignore]
    fn test_get_client_state_with_selecting_returns_selecting() {
        let mut msg = new_test_request();
        msg.ciaddr = Ipv4Addr::new(192, 168, 1, 2);
        msg.options.remove(0);

        let got = get_client_state(&msg);

        assert_eq!(got, ClientState::Selecting);
    }

    #[test]
    #[ignore]
    fn test_get_client_state_with_initreboot_returns_initreboot() {
        let mut msg = new_test_request();
        msg.options.remove(2);

        let got = get_client_state(&msg);

        assert_eq!(got, ClientState::InitReboot);
    }

    #[test]
    #[ignore]
    fn test_get_client_state_with_renewing_returns_renewing() {
        let mut msg = new_test_request();
        msg.options.remove(0);
        msg.options.remove(1);
        msg.ciaddr = Ipv4Addr::new(1, 2, 3, 4);

        let got = get_client_state(&msg);

        assert_eq!(got, ClientState::Renewing);
    }

    #[test]
    #[ignore]
    fn test_get_client_state_with_unknown_returns_unknown() {
        let mut msg = new_test_request();
        msg.options.clear();

        let got = get_client_state(&msg);

        assert_eq!(got, ClientState::Unknown);
    }*/

 /*   #[test]
    #[ignore]
    fn test_release_expired_leases_with_none_expired_releases_none() {
        let mut server = new_test_server(|| 42);
        server.pool.available_addrs.clear();
        server.cache.insert(
            MacAddr { octets: [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 2),
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.cache.insert(
            MacAddr { octets: [0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 3),
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 3));
        server.cache.insert(
            MacAddr { octets: [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 4),
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 4));

        server.release_expired_leases();

        assert_eq!(server.cache.len(), 3);
        assert_eq!(server.pool.available_addrs.len(), 0);
        assert_eq!(server.pool.allocated_addrs.len(), 3);
    }

    #[test]
    #[ignore]
    fn test_release_expired_leases_with_all_expired_releases_all() {
        let mut server = new_test_server(|| 42);
        server.pool.available_addrs.clear();
        server.cache.insert(
            MacAddr { octets: [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 2),
                options: vec![],
                expiration: 0,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.cache.insert(
            MacAddr { octets: [0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 3),
                options: vec![],
                expiration: 0,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 3));
        server.cache.insert(
            MacAddr { octets: [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 4),
                options: vec![],
                expiration: 0,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 4));

        server.release_expired_leases();

        assert_eq!(server.cache.len(), 0);
        assert_eq!(server.pool.available_addrs.len(), 3);
        assert_eq!(server.pool.allocated_addrs.len(), 0);
    }

    #[test]
    #[ignore]
    fn test_release_expired_leases_with_some_expired_releases_expired() {
        let mut server = new_test_server(|| 42);
        server.pool.available_addrs.clear();
        server.cache.insert(
            MacAddr { octets: [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 2),
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 2));
        server.cache.insert(
            MacAddr { octets: [0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 3),
                options: vec![],
                expiration: 0,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 3));
        server.cache.insert(
            MacAddr { octets: [0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC] },
            CachedConfig {
                client_addr: Ipv4Addr::new(192, 168, 1, 4),
                options: vec![],
                expiration: std::i64::MAX,
            },
        );
        server.pool.allocated_addrs.insert(Ipv4Addr::new(192, 168, 1, 4));

        server.release_expired_leases();

        assert_eq!(server.cache.len(), 2);
        assert!(!server
            .cache
            .contains_key(&MacAddr { octets: [0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB] }));
        assert_eq!(server.pool.available_addrs.len(), 1);
        assert_eq!(server.pool.allocated_addrs.len(), 2);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_known_release() {
        let release = new_test_release();
        let mut server = new_test_server(|| 42);
        let client_ip = Ipv4Addr::new(192, 168, 1, 2);
        server.pool.allocate_addr(client_ip);
        let client_mac = MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] };
        let client_config =
            CachedConfig { client_addr: client_ip, options: vec![], expiration: std::i64::MAX };
        server.cache.insert(client_mac, client_config.clone());

        let got = server.dispatch(release);

        assert!(got.is_err(), "server returned a Message value");
        assert!(!server.pool.addr_is_allocated(client_ip), "server did not free client address");
        assert!(server.pool.addr_is_available(client_ip), "server did not free client address");
        assert!(
            server.cache.contains_key(&client_mac),
            "server did not retain cached client settings"
        );
        assert_eq!(
            server.cache.get(&client_mac).unwrap(),
            &client_config,
            "server did not retain cached client settings"
        );
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_unknown_release() {
        let release = new_test_release();
        let mut server = new_test_server(|| 42);
        let client_ip = Ipv4Addr::new(192, 168, 1, 2);
        server.pool.allocate_addr(client_ip);
        let cached_mac = MacAddr { octets: [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA] };
        let client_config =
            CachedConfig { client_addr: client_ip, options: vec![], expiration: std::i64::MAX };
        server.cache.insert(cached_mac, client_config.clone());

        let got = server.dispatch(release);

        assert!(got.is_err(), "server returned a Message value");
        assert!(server.pool.addr_is_allocated(client_ip), "server did not free client address");
        assert!(!server.pool.addr_is_available(client_ip), "server did not free client address");
        assert!(
            server.cache.contains_key(&cached_mac),
            "server did not retain cached client settings"
        );
        assert_eq!(
            server.cache.get(&cached_mac).unwrap(),
            &client_config,
            "server did not retain cached client settings"
        );
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_inform_returns_ack() {
        let inform = new_test_inform();
        let mut server = new_test_server(|| 42);

        let got = server.dispatch(inform).unwrap();

        let want = new_test_inform_ack();

        assert_eq!(got, want, "expected: {:?}\ngot: {:?}", want, got);
    }

    #[test]
    #[ignore]
    fn test_dispatch_with_decline_marks_addr_allocated() {
        let decline = new_test_decline();
        let mut server = new_test_server(|| 42);
        let already_used_ip = Ipv4Addr::new(192, 168, 1, 2);
        server.config.managed_addrs.push(already_used_ip);
        let client_config = CachedConfig {
            client_addr: already_used_ip,
            options: vec![],
            expiration: std::i64::MAX,
        };
        server.cache.insert(decline.chaddr, client_config);

        let got = server.dispatch(decline);

        assert!(got.is_err(), "server returned a Message value");
        assert!(!server.pool.addr_is_available(already_used_ip), "addr still marked available");
        assert!(server.pool.addr_is_allocated(already_used_ip), "addr not marked allocated");
        assert!(
            !server.cache.contains_key(&MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] }),
            "client config retained"
        );
    }

    #[test]
    #[ignore]
    fn test_client_requested_lease_time() {
        let mut disc = new_test_discover();
        disc.options
            .push(ConfigOption { code: OptionCode::IpAddrLeaseTime, value: vec![0, 0, 0, 20] });

        let mut server = new_test_server(|| 42);
        let result = server.dispatch(disc).unwrap();
        assert_eq!(
            BigEndian::read_u32(
                &result.get_config_option(OptionCode::IpAddrLeaseTime).unwrap().value
            ),
            20
        );

        let cached_config =
            server.cache.get(&MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] }).unwrap();
        assert_eq!(cached_config.expiration, 42 + 20);
    }

    #[test]
    #[ignore]
    fn test_client_requested_lease_time_greater_than_max() {
        let mut disc = new_test_discover();
        disc.options
            .push(ConfigOption { code: OptionCode::IpAddrLeaseTime, value: vec![0, 0, 0, 20] });

        let mut server = new_test_server(|| 42);
        server.config.max_lease_time_s = 10;
        let result = server.dispatch(disc).unwrap();
        assert_eq!(
            BigEndian::read_u32(
                &result.get_config_option(OptionCode::IpAddrLeaseTime).unwrap().value
            ),
            10
        );

        let cached_config =
            server.cache.get(&MacAddr { octets: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF] }).unwrap();
        assert_eq!(cached_config.expiration, 42 + 10);
    }*/
}
