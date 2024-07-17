// rust implementation of arduino_connector for the rp2040 processor
// original project: https://github.com/AlexmagToast/LinuxCNC_ArduinoConnector
// Licence GPL


#![no_std]
#![no_main]


use core::fmt::Write;
use defmt::{info, panic, unwrap};
use embassy_rp::gpio;
use gpio::{Pin, Input, Pull, AnyPin, Level};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either, select4, Either4};
use embassy_rp::{bind_interrupts, pio};
use embassy_rp::adc;
use embassy_rp::adc::{Adc, Async};
use embassy_rp::peripherals::{USB, PIO0};
use embassy_rp::usb;
use embassy_rp::usb::{Driver};
use embassy_time::{Ticker, Duration, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;
use static_cell::StaticCell;
use heapless::String as String;
use pio::{Common, FifoJoin, Pio, PioPin, ShiftDirection, StateMachine};
use fixed::traits::ToFixed;
use {defmt_rtt as _, panic_probe as _};

static CHANNEL: channel::Channel<ThreadModeRawMutex, AppEvent, 64> = channel::Channel::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

pub struct PioEncoder<'d, T: pio::Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
    index: usize,
}

impl<'d, T: pio::Instance, const SM: usize> PioEncoder<'d, T, SM> {
    pub fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        index: usize,
        pin_a: impl PioPin,
        pin_b: impl PioPin,
        pullup: bool
    ) -> Self {
        let mut pin_a = pio.make_pio_pin(pin_a);
        let mut pin_b = pio.make_pio_pin(pin_b);
        
        sm.set_pin_dirs(pio::Direction::In, &[&pin_a, &pin_b]);

        if pullup {
            pin_a.set_pull(Pull::Up);
            pin_b.set_pull(Pull::Up);
        }
        
        let prg = pio_proc::pio_asm!("wait 1 pin 1", "wait 0 pin 1", "in pins, 2", "push",);
        
        let mut cfg = pio::Config::default();
        cfg.set_in_pins(&[&pin_a, &pin_b]);
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.clock_divider = 10_000.to_fixed();
        cfg.use_program(&pio.load_program(&prg.program), &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm, index }
    }
    
    pub async fn read(&mut self) -> (Direction, usize) {
        loop {
            match self.sm.rx().wait_pull().await {
                0 => return (Direction::Clockwise, self.index),
                1 => return (Direction::CounterClockwise, self.index),
                _ => {}
            }
        }
    }
}

pub enum Direction {
    Clockwise,
    CounterClockwise,
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello there!");
    
    let p = embassy_rp::init(Default::default());

    // Create USB
    let driver = Driver::new(p.USB, Irqs);

    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Unitware");
        config.product = Some("LinxCNC-connector");
        config.serial_number = Some("0");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let mut class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    let usb = builder.build();

    unwrap!(spawner.spawn(usb_task(usb)));


    //Buttons
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 0, AnyPin::from(p.PIN_0))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 1, AnyPin::from(p.PIN_1))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 2, AnyPin::from(p.PIN_2))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 3, AnyPin::from(p.PIN_3))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 4, AnyPin::from(p.PIN_4))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 5, AnyPin::from(p.PIN_5))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 6, AnyPin::from(p.PIN_6))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 7, AnyPin::from(p.PIN_9))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 8, AnyPin::from(p.PIN_10))));
    unwrap!(spawner.spawn(button_reader(CHANNEL.sender(), 9, AnyPin::from(p.PIN_11))));

    //Encoders
    let Pio { mut common, sm0, sm1, .. } = Pio::new(p.PIO0, Irqs);
    let mut encoder_a = PioEncoder::new(&mut common, sm0, 0, p.PIN_12, p.PIN_13, false);
    let mut encoder_b = PioEncoder::new(&mut common, sm1, 1, p.PIN_16, p.PIN_17, true);
    // unwrap!(spawner.spawn(encoder(CHANNEL.sender(), 123, &mut encoder_a)));

    let adc0: adc::Adc<'_, Async> = adc::Adc::new( p.ADC, Irqs, adc::Config::default());
    unwrap!(spawner.spawn( adc_reader(CHANNEL.sender(), adc0, p.PIN_26, p.PIN_27, p.PIN_28, p.PIN_29)));

    loop {
        class.wait_connection().await;
        info!("Connected");
        let _ = handler(&mut class, &mut encoder_a, &mut encoder_b).await;
        info!("Disconnected");
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn handler<'d, U: usb::Instance + 'd,
                     T: pio::Instance + 'd,
                     const SM0: usize,
                     const SM1: usize>(
                        class: &mut CdcAcmClass<'d, Driver<'d, U>>,
                        encoder: &mut PioEncoder<'d, T, SM0>,
                        encoder_b: &mut PioEncoder<'d, T, SM1>,
                        // adc_ts: &mut MyAdcChannel<'d, M>
                    ) -> Result<(), Disconnected> {
    let mut encoder_count = [0i32; 2];

    loop {
        // let start = Instant::now();
        let mut buf = [0; 64];

        //match with_deadline(start + Duration::from_secs(1), 
        match select4(Timer::after_secs(5), class.read_packet(&mut buf), CHANNEL.receive(),
                select(encoder.read(), encoder_b.read())).await {
            Either4::First(_) => {
                // Timeout
                let e0 = "E0:0\n\r";
                class.write_packet(e0.as_bytes()).await?;
            }        

            // USB Command
            Either4::Second(_n) => {
                let cmd = buf[0];
                
                match cmd {
                    b'E' => {
                        // Connection established	E	read/ write	0:0
                        let e0 = "E0:0\n\r";
                        class.write_packet(e0.as_bytes()).await?;
                    }
                    
                    // b'O' => {
                    //     // Outputs	                O	read only	0,1
                    //     // let e0 = "OUTPUT\n\r";
                    //     // class.write_packet(e0.as_bytes()).await?;
                    // }
                    
                    // b'P' => {
                    //     // PWM Outputs	            P	read only	0-255
                    //     // let e0 = "PWM\n\r";
                    //     // class.write_packet(e0.as_bytes()).await?;
                    // }
                    
                    // b'D' => {
                    //     // Digital LED Outputs	    D	read only	0,1
                    //     // let e0 = "LED\n\r";
                    //     // class.write_packet(e0.as_bytes()).await?;
                    // }
                    
                    _ => ()
                }
            }

            // channel report app events - buttons
            Either4::Third(AppEvent::INPUTEDGE(channel, state)) => {
                // Inputs & Toggle Inputs	I	write only	0,1
                let state = if state { 1 } else { 0 };
                let mut msg: String<8> = String::new();
                core::writeln!(&mut msg, "I{}:{}\r", channel, state).unwrap();
                class.write_packet(msg.as_bytes()).await?;
            }
                
            // channel report app events - adc
            Either4::Third(AppEvent::ADC(channel, level)) => {
                // Analog Inputs	        A	write only	0-1024
                let mut msg: String<16> = String::new();
                core::writeln!(&mut msg, "A{}:{}\r", channel, level).unwrap();
                class.write_packet(msg.as_bytes()).await?;
            }
                
            // Latching Potentiometers	L	write only	0-max Position
            // binary encoded Selector	K	write only	0-32
            // Matrix Keyboard	        M	write only	0,1
            // Joystick	                R	write only	counter

            // Encoders
            Either4::Fourth(enc) => match enc {
                Either::First((dir, n)) |
                Either::Second((dir, n)) => {
                    encoder_count[n] += match dir {
                        Direction::Clockwise => { 1 }
                        Direction::CounterClockwise => {-1}
                    };
            
                    let mut msg: String<16> = String::new();
                    core::writeln!(&mut msg, "R{}:{}\r", n, encoder_count[n]).unwrap();
                    class.write_packet(msg.as_bytes()).await?;
                }
            }
        }
    }
}

enum AppEvent {
    INPUTEDGE(u8, bool),
    ADC(u8, u16),
}


#[embassy_executor::task(pool_size = 10)]
async fn button_reader(control: channel::Sender<'static, ThreadModeRawMutex, AppEvent, 64>, number: u8, pin: AnyPin) {
    let mut btn = Debouncer::new(Input::new(pin, Pull::Up), Duration::from_millis(20));

    loop {
        let is_active = match btn.debounce().await {
            Level::Low => {true},
            Level::High => {false},
        };
        control.send(AppEvent::INPUTEDGE(number, is_active)).await;
    }
}

pub struct Debouncer<'a, T: Pin> {
    input: Input<'a, T>,
    debounce: Duration,
}

impl<'a, T: Pin> Debouncer<'a, T> {
    pub fn new(input: Input<'a, T>, debounce: Duration) -> Self {
        Self { input, debounce }
    }

    pub async fn debounce(&mut self) -> Level {
        loop {
            let l1 = self.input.get_level();

            self.input.wait_for_any_edge().await;

            Timer::after(self.debounce).await;

            let l2 = self.input.get_level();
            if l1 != l2 {
                break l2;
            }
        }
    }
}

#[embassy_executor::task]
async fn adc_reader(control: channel::Sender<'static, ThreadModeRawMutex, AppEvent, 64>,
                    mut adc: Adc<'static, Async>,
                    adc_0: embassy_rp::peripherals::PIN_26,
                    adc_1: embassy_rp::peripherals::PIN_27,
                    adc_2: embassy_rp::peripherals::PIN_28,
                    adc_3: embassy_rp::peripherals::PIN_29)
{
    let mut ticker = Ticker::every(Duration::from_millis(50));

    let mut chan = [
            adc::Channel::new_pin(adc_0, Pull::None),
            adc::Channel::new_pin(adc_1, Pull::None),
            adc::Channel::new_pin(adc_2, Pull::None),
            adc::Channel::new_pin(adc_3, Pull::None),
        ];

    let mut reported = [0; 4];

    loop {
        ticker.next().await;

        for n in 0..4 {
            let reading = adc.read(&mut chan[n]).await.unwrap() as i32;
            let reading = reading >> 2;

            if (reported[n] - reading).abs() > 2 {
            // if reported[n] != reading {
                reported[n] = reading;
                control.send(AppEvent::ADC(n as u8, reading as u16)).await;
            }
        }        
    }                
}
