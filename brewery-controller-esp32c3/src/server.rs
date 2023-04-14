use core::cell::Cell;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_executor::_export::StaticCell;
use embassy_futures::block_on;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, IpListenEndpoint, StackResources, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32c3_hal::peripherals::RADIO;
use esp32c3_hal::radio::RadioExt;
use esp_wifi::wifi::{WifiState, WifiMode};
use esp_wifi::wifi::{WifiDevice, WifiController, WifiEvent};
use smoltcp::socket;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::PidConfig;

// secrets.env is ignored by git and contains values for SSID and PASSWORD
include!("secrets.env");

const SERVER_PORT: u16 = 80;

pub static DATA_SIGNAL: Signal<CriticalSectionRawMutex, DisplayData> = Signal::new();
pub static PID_CONFIG: Mutex<Cell<PidConfig>> = Mutex::new(Cell::new(crate::INITIAL_PID_CONFIG));

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[derive(Copy, Clone, Debug)]
pub struct DisplayData {
    pub time: u64,
    pub temperature: f32,
    pub setpoint: f32,
    pub power1: u32,
    pub power2: u32,
    pub pid: PidConfig,
}

#[derive(PartialEq, Debug)]
enum ServerRoute {
    GetIndex,
    GetData,
    PostIndex,
}

#[embassy_executor::task]
pub async fn run_server_task(
    spawner: Spawner,
    seed: u64,
    radio: RADIO,
) {
    let (wifi, _) = radio.split();
    let (wifi_interface, controller) = esp_wifi::wifi::new_with_mode(wifi, WifiMode::Sta);

    let config = Config::Dhcp(Default::default());

    // Init network stack
    let stack = &*singleton!(Stack::new(
        wifi_interface,
        config,
        singleton!(StackResources::<3>::new()),
        seed,
    ));

    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(&stack));
    spawner.must_spawn(http_task(&stack));
    spawner.must_spawn(http_task(&stack));
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            info!("Starting wifi");
            controller.start().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match controller.connect().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task(pool_size = 2)]
async fn http_task(stack: &'static Stack<WifiDevice<'static>>) {
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }

    while !stack.config().is_some() {
        Timer::after(Duration::from_millis(100)).await;
    }

    let mut rx_buffer = [0; 16384];
    let mut tx_buffer = [0; 16384];
    let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_net::SmolDuration::from_secs(10)));

    loop {
        if socket.state() == socket::tcp::State::CloseWait {
            socket.abort();
        }

        if socket.state() == socket::tcp::State::Closed {
            info!("Wait for connection...");
            let r = socket.accept(IpListenEndpoint {
                    addr: None,
                    port: SERVER_PORT,
                }).await;
            
            if let Err(e) = r {
                warn!("connect error: {:?}", e);
                continue;
            }

            info!("Connected...");
        }

        let mut route: Option<ServerRoute> = None;
        let mut buffer = [0u8; 1024];
        let mut buffer_len = 0;

        match socket.read(&mut buffer).await {
            Ok(0) => {
                info!("read EOF {}", socket.state());
            }
            Ok(len) => {
                buffer_len = len;

                route = if buffer.starts_with(b"GET / ") {
                    Some(ServerRoute::GetIndex)
                } else if buffer.starts_with(b"GET /data ") {
                    Some(ServerRoute::GetData)
                } else if buffer.starts_with(b"POST / ") {
                    Some(ServerRoute::PostIndex)
                } else {
                    None
                };
            }
            Err(e) => {
                warn!("read error: {:?}", e);
                continue;
            }
        };

        let mut writer = SocketWriter::new(socket);

        let result = match route {
            Some(ServerRoute::GetIndex) => do_get_index(&mut writer).await,
            Some(ServerRoute::GetData) => do_get_data(&mut writer).await,
            Some(ServerRoute::PostIndex) => do_post_index(&mut writer, &buffer[..buffer_len]).await,
            None => do_404(&mut writer).await,
        };

        // Unborrow the socket so it can be used in the next loop iteration
        socket = writer.socket();

        if let Err(e) = result {
            warn!("write error: {:?}", e);
        }
    }
}

async fn do_404(socket: &mut SocketWriter<'_>) -> Result<(), embassy_net::tcp::Error> {
    write!(socket, "HTTP/1.1 404 Not Found\r\n\r\n")?;
    socket.flush().await
}

async fn do_get_index(socket: &mut SocketWriter<'_>) -> Result<(), embassy_net::tcp::Error> {
    let message_body = include_str!("index.html");
    do_response(socket, "text/html; charset=UTF-8", message_body).await
}

async fn do_get_data(socket: &mut SocketWriter<'_>) -> Result<(), embassy_net::tcp::Error> {
    write!(socket,
        "HTTP/1.1 200 OK\r\n\
        Content-Type: text/event-stream\r\n\r\n"
    )?;

    loop {
        let data = DATA_SIGNAL.wait().await;

        write!(socket, "data: {},{},{},{},{},{},{},{}\r\n\r\n",
            data.time,
            data.temperature,
            data.setpoint,
            data.power1,
            data.power2,
            data.pid.controller_gain,
            data.pid.integral_time, 
            data.pid.derivative_time,
        )?;
        socket.flush().await?;
    }
}

async fn do_post_index(socket: &mut SocketWriter<'_>, post_data: &[u8]) -> Result<(), embassy_net::tcp::Error> {
    let data_str = core::str::from_utf8(post_data).unwrap();
    let mut msg_parts = data_str.split("\r\n\r\n");
    
    // Ignore the headers
    msg_parts.next();

    if let Some(body) = msg_parts.next() {
        let body_parts = body.split(",");
        let mut values = [0.0; 3];
        let mut i = 0;

        for part in body_parts {
            if let Ok(v) = part.parse::<f32>() {
                values[i] = v;
                i += 1;
            }
        }

        if i == values.len() {
            let pid_config = PidConfig {
                controller_gain: values[0],
                integral_time: values[1],
                derivative_time: values[2],
            };
        
            critical_section::with(|cs| {
                PID_CONFIG.borrow(cs).set(pid_config);
            });
        
            info!("PID Config changed to: {:?}", pid_config);
        }
    }

    write!(socket, "HTTP/1.1 204 No Content\r\n\r\n")?;
    socket.flush().await
}

async fn do_response(socket: &mut SocketWriter<'_>, content_type: &str, message: &str) -> Result<(), embassy_net::tcp::Error> {
    write!(socket,
        "HTTP/1.1 200 OK\r\n\
        Content-Type: {}\r\n\
        Content-Length: {}\r\n\r\n{}",
        content_type,
        message.len(),
        message,
    )?;
    socket.flush().await
}

struct SocketWriter<'a> {
    socket: TcpSocket<'a>,
}

impl<'a> SocketWriter<'a> {
    fn new(socket: TcpSocket<'a>) -> Self {
        Self { socket }
    }

    async fn flush(&mut self) -> Result<(), embassy_net::tcp::Error> {
        embedded_io::asynch::Write::flush(&mut self.socket).await
    }

    fn write_fmt(&mut self, args: core::fmt::Arguments<'_>) -> Result<(), embassy_net::tcp::Error> {
        match core::fmt::Write::write_fmt(self, args) {
            Ok(_) => Ok(()),
            Err(_) => Err(embassy_net::tcp::Error::ConnectionReset),
        }
    }

    fn socket(self) -> TcpSocket<'a> {
        self.socket
    }
}

impl<'a> core::fmt::Write for SocketWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if self.socket.state() != smoltcp::socket::tcp::State::Established {
            info!("state: {}", self.socket.state());
        }
        match block_on(self.socket.write(s.as_bytes())) {
            Ok(_) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}