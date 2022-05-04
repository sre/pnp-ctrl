#![no_std]
#![no_main]

// commands
// 'move <rel|abs> [X<mm.um>] [Y<mm.um>]'

use panic_halt as _;

use core::str;
use core::str::FromStr;
use nb::block;
use cortex_m_rt::entry;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{gpio::PinState, pac, prelude::*};
use stm32f4xx_hal::i2c::I2c;
use stm32f4xx_hal::pac::interrupt;
use stm32f4xx_hal::{pac::TIM2, timer::{CounterUs, Event}};
use usb_device::prelude::*;
use smart_leds::{SmartLedsWrite,White};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use embedded_hal::serial::{Read, Write};
use core::fmt::Write as FmtWrite;
use noline::sync::embedded::IO;
use noline::builder::EditorBuilder;
use noline::error::Error;
use ws2812_spi as ws2812;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use fixed::types::I22F10;
use fixed::types::I30F2;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

const NUM_LEDS_CAM: usize = 12;
const NUM_LEDS: usize = 200;

// 279mm / 168.5mm
const BED_CAM_POS: [i32; 2] = [186*200*256, 112*200*256+17066];
static mut BOOKMARKS: [[i32; 2]; 10] = [[0; 2]; 10];

#[allow(dead_code)]
struct RGBcs {
    /// RGB1 CS - Camera Bed (WS2812 RGB)
    channel1: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
    /// RGB2 CS - Camera Head (WS2812 RGB)
    channel2: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
    /// RGB3 CS
    channel3: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
    /// RGB4 CS
    channel4: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
    /// RGB5 CS - Machine Light (SK6812 RGBW)
    channel5: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
    /// RGB6 CS - Test LED (SK6812 RGBW)
    channel6: stm32f4xx_hal::gpio::EPin<stm32f4xx_hal::gpio::Output>,
}

static TIMER: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM2() {
    static mut TIM: Option<CounterUs<TIM2>> = None;
    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            TIMER.borrow(cs).replace(None).unwrap()
        })
    });

    tim.clear_interrupt(stm32f4xx_hal::timer::Event::Update);
}

fn ledset_rgb(enable: bool) -> ([smart_leds::RGBA<u8, White<u8>>; NUM_LEDS_CAM]) {
    let mut led_data = [smart_leds::RGBW::default(); NUM_LEDS_CAM];
    for i in 0..NUM_LEDS_CAM {
        if enable {
            led_data[i] = smart_leds::RGBW {r:255, g:255, b:255, a:smart_leds::White(0)};
        } else {
            led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(0)};
        }
    }
    led_data
}

enum MCP23S17Input {
    VIN,
    XR,
    XL,
    YLB,
    YLF,
    YRB,
    YRF,
}

enum PNPHead {
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight,
}

impl FromStr for PNPHead {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "lf" => Ok(PNPHead::FrontLeft),
            "fl" => Ok(PNPHead::FrontLeft),
            "rf" => Ok(PNPHead::FrontRight),
            "fr" => Ok(PNPHead::FrontRight),
            "lb" => Ok(PNPHead::BackLeft),
            "bl" => Ok(PNPHead::BackLeft),
            "rb" => Ok(PNPHead::BackRight),
            "br" => Ok(PNPHead::BackRight),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum PositioningMode {
    Absolute,
    Relative,
}

impl FromStr for PositioningMode {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "abs" => Ok(PositioningMode::Absolute),
            "absolute" => Ok(PositioningMode::Absolute),
            "rel" => Ok(PositioningMode::Relative),
            "relative" => Ok(PositioningMode::Relative),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Axis {
    X,
    Y,
}

impl FromStr for Axis {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "X" => Ok(Axis::X),
            "Y" => Ok(Axis::Y),
            _ => Err(()),
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct Position {
    axis: Axis,
    position_um: i32,
}

impl Position {
    fn from_usteps(axis: Axis, position_usteps: i32) -> Self {
        Position {
            axis: axis,
            position_um: position_usteps * 15 / 2 / 256
        }
    }
}

impl FromStr for Position {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s.len() < 2 {
            return Err(());
        }

        let axis = Axis::from_str(&s[..1]);
        let fp = I22F10::overflowing_from_str(&s[1..]);

        if fp.is_err() {
            return Err(());
        }
        let (fp, overflow) = fp.unwrap();

        if overflow {
            return Err(());
        }

        let mut converted = I30F2::from_num(0i32);
        if converted.overflowing_mul_acc(fp, I30F2::from_num(1000i32)) {
            return Err(());
        }
        let converted = converted.round_to_zero();

        Ok(Position {
            axis: axis?,
            position_um: converted.to_num(),
        })
    }
}

#[derive(Copy, Clone, Debug)]
struct Coordinate {
    mode: PositioningMode,
    pos_x: Option<i32>,
    pos_y: Option<i32>,
}

impl Coordinate {
    fn new(mode: PositioningMode, pos1: Position, pos2: Option<Position>) -> Self {
        if pos2.is_none() {
            if pos1.axis == Axis::X {
                Coordinate { mode: mode, pos_x: Some(pos1.position_um), pos_y: None }
            } else {
                Coordinate { mode: mode, pos_x: None, pos_y: Some(pos1.position_um) }
            }
        } else {
            if pos1.axis == Axis::X {
                Coordinate { mode: mode, pos_x: Some(pos1.position_um), pos_y: Some(pos2.unwrap().position_um) }
            } else {
                Coordinate { mode: mode, pos_x: Some(pos2.unwrap().position_um), pos_y: Some(pos1.position_um) }
            }
        }
    }

    fn um_to_usteps(self: &Self) -> Self {
        /* 1 rotation has 200 * 256 µSteps, 1 rotation equals 1.5mm = 1500µm */
        /* => µm to µsteps required multiplication with (200*256)/1500 */
        /* this overflows 31bits at slightly below 42mm, so it needs to be optimized: */
        /* (200*256)/1500 = (2*256)/15 => overflows at 4m, which is a lot more than the machine size */
        let x = if self.pos_x.is_some() { Some(self.pos_x.unwrap() * 2 * 256 / 15) } else { None };
        let y = if self.pos_y.is_some() { Some(self.pos_y.unwrap() * 2 * 256 / 15) } else { None };
        Coordinate { mode: self.mode, pos_x: x, pos_y: y }
    }

    fn to_absolute(self: &Self, current_x_abs: i32, current_y_abs: i32) -> Self {
        let x = if self.pos_x.is_some() {
            match self.mode {
                PositioningMode::Absolute => self.pos_x,
                PositioningMode::Relative => Some(current_x_abs + self.pos_x.unwrap()),
            }
        } else {
            Some(current_x_abs)
        };

        let y = if self.pos_y.is_some() {
            match self.mode {
                PositioningMode::Absolute => self.pos_y,
                PositioningMode::Relative => Some(current_y_abs + self.pos_y.unwrap()),
            }
        } else {
            Some(current_y_abs)
        };

        Coordinate { mode: self.mode, pos_x: x, pos_y: y }
    }
}

enum LedColor {
    Off,
    White,
    DarkWhite,
    Red,
    Green,
    Blue,
    All,
    Test,
    Test2,
    Test3,
}

enum DigitalOutputCmd {
    Toggle,
    On,
    Off,
}

impl FromStr for DigitalOutputCmd {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "toggle" => Ok(DigitalOutputCmd::Toggle),
            "1" => Ok(DigitalOutputCmd::On),
            "on" => Ok(DigitalOutputCmd::On),
            "enable" => Ok(DigitalOutputCmd::On),
            "0" => Ok(DigitalOutputCmd::Off),
            "off" => Ok(DigitalOutputCmd::Off),
            "disable" => Ok(DigitalOutputCmd::Off),
            _ => Err(()),
        }
    }
}

/// read unique ID register
fn uid_get_reg(offset: u8) -> u8 {
    let uid_base = 0x1FFF_7A10 as *mut u8;

    if offset > 11 {
        panic!();
    }

    unsafe {
        uid_base.add(offset as usize).read_volatile()
    }
}

/// Return serial number as used by STM32 built-in bootloader
fn uid_get_sn() -> [u8; 6] {
    let sn: [u8; 6] = [
        uid_get_reg(11),
        uid_get_reg(10) + uid_get_reg(2),
        uid_get_reg(9),
        uid_get_reg(8) + uid_get_reg(0),
        uid_get_reg(7),
        uid_get_reg(6),
    ];

    sn
}

fn ledset_rgbw(color: LedColor) -> ([smart_leds::RGBA<u8, White<u8>>; NUM_LEDS]) {
    let mut led_data = [smart_leds::RGBW::default(); NUM_LEDS];

    match color {
        LedColor::Off => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(0)};
            }
        },
        LedColor::White => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(255)};
            }
        },
        LedColor::DarkWhite => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(42)};
            }
        },

        LedColor::Red => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:255, g:0, b:0, a:smart_leds::White(0)};
            }
        },
        LedColor::Green => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:0, g:255, b:0, a:smart_leds::White(0)};
            }
        },
        LedColor::Blue => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:255, a:smart_leds::White(0)};
            }
        },
        LedColor::All => {
            for i in 0..NUM_LEDS {
                led_data[i] = smart_leds::RGBW {r:255, g:255, b:255, a:smart_leds::White(255)};
            }
        },
        LedColor::Test => {
            for i in 0..NUM_LEDS {
                match i % 4 {
                    1 => led_data[i] = smart_leds::RGBW {r:255, g:0, b:0, a:smart_leds::White(0)},
                    2 => led_data[i] = smart_leds::RGBW {r:0, g:255, b:0, a:smart_leds::White(0)},
                    3 => led_data[i] = smart_leds::RGBW {r:0, g:0, b:255, a:smart_leds::White(0)},
                    _ => led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(255)},
                }
            }
        },
        LedColor::Test2 => {
            // outer ring except last element
            for i in 0..7*6 {
                led_data[i] = smart_leds::RGBW {r:128, g:0, b:0, a:smart_leds::White(0)};
            }
            // inner cross back-left
            for i in 7*6..7*6+6 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // middle dot
            led_data[48] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            // inner cross left-back-inner
            for i in 49..49+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // ring left-back
            for i in 52..52+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            }
            // inner cross left-back-outer
            for i in 76..76+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // inner cross left-front-outer
            for i in 79..79+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // ring left-front
            for i in 82..82+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            }
            // inner cross left-front-inner
            for i in 106..106+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // middle dot
            led_data[109] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            // inner cross front
            for i in 110..110+12 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // middle dot
            led_data[122] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            // inner cross right-front-inner
            for i in 123..123+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // ring right-front
            for i in 126..126+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            }
            // inner cross right-front-outer
            for i in 150..150+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // inner cross right-back-outer
            for i in 153..153+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // ring right-back
            for i in 156..156+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            }
            // inner cross right-back-inner
            for i in 180..180+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // middle dot
            led_data[183] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            // inner cross back-left
            for i in 184..184+6 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // last element of outer ring
            for i in 190..190+6 {
                led_data[i] = smart_leds::RGBW {r:128, g:0, b:0, a:smart_leds::White(0)};
            }
        },
        LedColor::Test3 => {
            // outer ring except last element
            for i in 0..7*6 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // inner cross back-left
            for i in 7*6..7*6+6 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // middle dot
            led_data[48] = smart_leds::RGBW {r:128, g:0, b:0, a:smart_leds::White(0)};
            // inner cross left-back-inner
            for i in 49..49+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // ring left-back
            for i in 52..52+24 {
                led_data[i] = smart_leds::RGBW {r:128, g:0, b:0, a:smart_leds::White(0)};
            }
            // inner cross left-back-outer
            for i in 76..76+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // inner cross left-front-outer
            for i in 79..79+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // ring left-front
            for i in 82..82+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            }
            // inner cross left-front-inner
            for i in 106..106+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // middle dot
            led_data[109] = smart_leds::RGBW {r:0, g:128, b:0, a:smart_leds::White(0)};
            // inner cross front
            for i in 110..110+12 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // middle dot
            led_data[122] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            // inner cross right-front-inner
            for i in 123..123+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // ring right-front
            for i in 126..126+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:128, a:smart_leds::White(0)};
            }
            // inner cross right-front-outer
            for i in 150..150+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // inner cross right-back-outer
            for i in 153..153+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // ring right-back
            for i in 156..156+24 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            }
            // inner cross right-back-inner
            for i in 180..180+3 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // middle dot
            led_data[183] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(128)};
            // inner cross back-left
            for i in 184..184+6 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
            // last element of outer ring
            for i in 190..190+6 {
                led_data[i] = smart_leds::RGBW {r:0, g:0, b:0, a:smart_leds::White(5)};
            }
        },
    }

    led_data
}

// The usb-device API doesn't play well with the `block!` from
// `nb`. Added a simple wrapper to be able to use a shared
// implementation for both the UART and USB examples.
struct SerialWrapper<'a> {
    device: &'a mut UsbDevice<'a, UsbBus<USB>>,
    serial: &'a mut SerialPort<'a, UsbBus<USB>>,
    ready: bool,
}

impl<'a> SerialWrapper<'a> {
    fn new(
        device: &'a mut UsbDevice<'a, UsbBus<USB>>,
        serial: &'a mut SerialPort<'a, UsbBus<USB>>,
    ) -> Self {
        Self {
            device,
            serial,
            ready: false,
        }
    }

    fn poll(&mut self) -> bool {
        self.device.poll(&mut [self.serial])
    }

    fn is_ready(&mut self) -> bool {
        if !self.ready {
            self.ready = self.poll();
        }

        self.ready
    }

    fn try_op<'b, T, E>(
        &'b mut self,
        f: impl FnOnce(&'b mut SerialPort<'a, UsbBus<USB>>) -> nb::Result<T, E>,
    ) -> nb::Result<T, E> {
        if self.is_ready() {
            let res = f(self.serial);

            match res {
                Err(nb::Error::WouldBlock) => self.ready = false,
                _ => (),
            }

            res
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'a> Read<u8> for SerialWrapper<'a> {
    type Error = UsbError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.try_op(|serial| Read::read(serial))
    }
}

impl<'a> Write<u8> for SerialWrapper<'a> {
    type Error = UsbError;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.try_op(|serial| Write::write(serial, word))
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.try_op(|serial| Write::flush(serial))
    }
}

impl<'a> FmtWrite for SerialWrapper<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            self.write(b).or_else(|_| Err(core::fmt::Error {}))?;
        }

        Ok(())
    }
}

fn bootloader_set_magic(key: u32) {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.write_volatile(key) };
}

fn system_reset() {
    cortex_m::interrupt::disable();
    cortex_m::peripheral::SCB::sys_reset();
}

/// Returns device serial number as hex string slice.
fn get_serial_str() -> &'static str {
    static mut SERIAL: [u8; 12] = [b' '; 12];
    let serial = unsafe { SERIAL.as_mut() };
    let uid = uid_get_sn();

    fn hex(mut nibble: u8, high: bool) -> u8 {
        if high {
            nibble >>= 4;
        }

        nibble &= 0xf;

        match nibble {
            0..=9 => nibble + b'0',
            0xa..=0xf => nibble - 0xa + b'A',
            _ => b' ',
        }
    }

    for (i, d) in serial.iter_mut().enumerate() {
        *d = hex(uid[i/2], i%2 == 0)
    }

    unsafe { str::from_utf8_unchecked(serial) }
}

#[derive(PartialEq)]
enum TMCType {
    X,
    Y,
    UNKNOWN,
}

fn tmc_identify<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>) -> Result<TMCType, tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    let gconf = tmc.read_register(tmc5072::Registers::GCONF)?;

    if gconf.1 & tmc5072::GCONF::SINGLE_DRIVER.bits() == tmc5072::GCONF::SINGLE_DRIVER.bits() {
        Ok(TMCType::X)
    } else if gconf.1 & tmc5072::GCONF::DC_SYNC.bits() == tmc5072::GCONF::DC_SYNC.bits() {
        Ok(TMCType::Y)
    } else {
        Ok(TMCType::UNKNOWN)
    }
}

fn motor_setup_one<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>, motor: tmc5072::Motor) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    tmc.chopconf_set(motor, tmc5072::ChopConf {
        diss2g: false,
        dedge: false,
        intpol16: false,
        mres: tmc5072::ChopConfMicroStepResolution::S256,
        vhighchm: false,
        vhighfs: false,
        vsense: false,
        tbl: tmc5072::ChopConfBlankTimeSelect::Clocks36,
        chm: false,
        rndtf: false,
        disfdcc: false,
        hend_offset: tmc5072::ChopConfHEND::HN2,
        hstrt_tfd: tmc5072::ChopConfHSTRT::S4,
        toff: tmc5072::ChopConfTOFF::Off5,
    })?;

    tmc.iholdirun_set(motor, tmc5072::IHoldIRun {
        i_hold_delay: 1,
        i_run: 31,
        i_hold: 5,
    })?;

    tmc.pwmconf_set(motor, tmc5072::PWMConf {
        freewheel: tmc5072::PWMConfFreewheel::Normal,
        autoscale: true,
        pwmfreq: tmc5072::PWMConfFreq::F2_2048,
        pwm_grad: 1,
        pwm_ampl: 200,
    })?;

    if tmc_identify(tmc)? == TMCType::X || tmc_identify(tmc)? == TMCType::Y {
        tmc.rampcfg_set(motor, tmc5072::RampConfig {
            VSTART: 1,
            A1: 1000,
            V1: 50000,
            AMAX: 5120,
            VMAX: 512000,
            DMAX: 5120,
            D1: 1400,
            VSTOP: 10,
            TZEROWAIT: 50,
            VHIGH: 400000,
            VCOOLTHRS: 30000,
        })?;
    } else {
        tmc.rampcfg_set(motor, tmc5072::RampConfig {
            VSTART: 1,
            A1: 1000,
            V1: 50000,
            AMAX: 500,
            VMAX: 256000,
            DMAX: 700,
            D1: 1400,
            VSTOP: 10,
            TZEROWAIT: 50,
            VHIGH: 400000,
            VCOOLTHRS: 30000,
        })?;
    }

    tmc.ramp_mode_set(motor, tmc5072::RampMode::Positioning)?;

    // disable stallguard
    tmc.write_motor_register(motor, tmc5072::MotorRegister::SW_MODE, 0)?;

    Ok(())
}

fn motor_autohome_dual<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    for motor in [tmc5072::Motor::Motor1, tmc5072::Motor::Motor2] {
        /* hold motor */
        tmc.ramp_mode_set(motor, tmc5072::RampMode::Hold)?;

        /* configure homing ramp */
        tmc.rampcfg_set(motor, tmc5072::RampConfig {
            VSTART: 256,
            A1: 200*256,
            V1: 200*256,
            AMAX: 5120*1,
            VMAX: 200*256*5,
            DMAX: 5120,
            D1: 200*256*2,
            VSTOP: 256,
            TZEROWAIT: 50,
            VHIGH: 200*256*10,
            VCOOLTHRS: 200*256*2,
        })?;

        /* configure SGT */
        tmc.write_motor_register(motor, tmc5072::MotorRegister::COOLCONF, 2 << 16)?;

        /* disable stallGuard2 */
        tmc.write_motor_register(motor, tmc5072::MotorRegister::SW_MODE, 0x0)?;

        /* clear stallguard flag */
        tmc.read_motor_register(motor, tmc5072::MotorRegister::RAMP_STAT)?;
    }

    for motor in [tmc5072::Motor::Motor1, tmc5072::Motor::Motor2] {
        /* drive to Y max */
        tmc.ramp_mode_set(motor, tmc5072::RampMode::VelocityPos)?;
    }

    /* wait for motor acceleration phase */
    for _i in 0..5_000 {
        let v1 = tmc.velocity_get(tmc5072::Motor::Motor1)?;
        let v2 = tmc.velocity_get(tmc5072::Motor::Motor2)?;
        if v1 >= 200*256*5 && v2 >= 200*256*5 {
            break;
        }
    }

    for motor in [tmc5072::Motor::Motor1, tmc5072::Motor::Motor2] {
        /* enable stallGuard2 */
        tmc.write_motor_register(motor, tmc5072::MotorRegister::SW_MODE, 0x400)?;

        /* clear stallguard flag */
        tmc.read_motor_register(motor, tmc5072::MotorRegister::RAMP_STAT)?;
    };

    Ok(())
}

fn motor_autohome_dual_full<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>, pos: i32) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    let mut drvstatus1;
    let mut drvstatus2;
    motor_autohome_dual(tmc)?;

    loop {
        drvstatus1 = tmc.status_get(tmc5072::Motor::Motor1)?;
        drvstatus2 = tmc.status_get(tmc5072::Motor::Motor2)?;

        if (drvstatus1 & (1 << 24) != 0) && (drvstatus2 & (1 << 24) != 0) {
            break;
        }
    }

    for motor in [tmc5072::Motor::Motor1, tmc5072::Motor::Motor2] {
        tmc.position_set_home(motor, pos)?;
        tmc.position_set(motor, pos)?;

        motor_setup_one(tmc, motor)?;
    }
    motor_setup_one(tmc, tmc5072::Motor::Motor2)?;

    Ok(())
}

fn motor_autohome<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>, motor: tmc5072::Motor) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    tmc.rampcfg_set(motor, tmc5072::RampConfig {
        VSTART: 256,
        A1: 200*256,
        V1: 200*256,
        AMAX: 5120*1,
        VMAX: 200*256*5,
        DMAX: 5120,
        D1: 200*256*2,
        VSTOP: 256,
        TZEROWAIT: 50,
        VHIGH: 200*256*10,
        VCOOLTHRS: 200*256*2,
    })?;

    /* configure SGT */
    tmc.write_motor_register(motor, tmc5072::MotorRegister::COOLCONF, 1 << 16)?;

    /* continous left */
    tmc.ramp_mode_set(motor, tmc5072::RampMode::VelocityNeg)?;

    /* disable stallGuard2 */
    tmc.write_motor_register(motor, tmc5072::MotorRegister::SW_MODE, 0x0)?;

    /* clear stallguard flag */
    tmc.read_motor_register(motor, tmc5072::MotorRegister::RAMP_STAT)?;

    /* wait for motor acceleration phase */
    for _i in 0..10_000 {
        let v = tmc.velocity_get(motor)?;
        if v <= 200*256*5 {
            break;
        }
    }

    /* enable stallGuard2 */
    tmc.write_motor_register(motor, tmc5072::MotorRegister::SW_MODE, 0x400)?;

    /* clear stallguard flag */
    tmc.read_motor_register(motor, tmc5072::MotorRegister::RAMP_STAT)?;

    Ok(())
}

fn motor_autohome_full<SPI, CS, E>(tmc: &mut tmc5072::Tmc5072<SPI, CS>, motor: tmc5072::Motor) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    let mut drvstatus;
    motor_autohome(tmc, motor)?;

    loop {
        drvstatus = tmc.status_get(motor)?;
        if drvstatus & (1 << 24) != 0 {
            break;
        }
    }

    if drvstatus & (1 << 24) != 0 {
        tmc.position_set_home(motor, -200*256)?;
        tmc.position_set(motor, 0)?;
        motor_setup_one(tmc, motor)?;
    }

    Ok(())
}

fn motor_setup_all<SPI, CS, E>(trinamics: [&mut tmc5072::Tmc5072<SPI, CS>; 5]) -> Result<(), tmc5072::Error<E>>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    /* configure GCONF (hardware setup) for all motor controllers and lock the register */
    trinamics[0].write_register(tmc5072::Registers::GCONF, (tmc5072::GCONF::SINGLE_DRIVER | tmc5072::GCONF::SHAFT1 | tmc5072::GCONF::LOCK_GCONF).bits())?;
    trinamics[1].write_register(tmc5072::Registers::GCONF, (tmc5072::GCONF::DC_SYNC | tmc5072::GCONF::SHAFT1 | tmc5072::GCONF::SHAFT2 | tmc5072::GCONF::ENC2_ENABLE | tmc5072::GCONF::LOCK_GCONF).bits())?;
    trinamics[2].write_register(tmc5072::Registers::GCONF, (tmc5072::GCONF::LOCK_GCONF).bits())?;
    trinamics[3].write_register(tmc5072::Registers::GCONF, (tmc5072::GCONF::LOCK_GCONF).bits())?;
    trinamics[4].write_register(tmc5072::Registers::GCONF, (tmc5072::GCONF::LOCK_GCONF).bits())?;

    for tmc in trinamics {
        motor_setup_one(tmc, tmc5072::Motor::Motor1)?;
        motor_setup_one(tmc, tmc5072::Motor::Motor2)?;
    }

    Ok(())
}

fn bookmark_set(id: u8, x: i32, y: i32) -> bool {
    let id = id as usize;
    let max = unsafe { BOOKMARKS.len() };

    if id >= max {
        return false;
    }

    unsafe {
        BOOKMARKS[id][0] = x;
        BOOKMARKS[id][1] = y;
    }

    true
}

fn bookmark_get(id: u8) -> (i32,i32) {
    let id = id as usize;
    let max = unsafe { BOOKMARKS.len() };

    if id >= max {
        return (0,0);
    }

    unsafe {
        (BOOKMARKS[id][0], BOOKMARKS[id][1])
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(56.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x05e1))
        .manufacturer("sre")
        .product("PnP Controller - pnp@sebastianreichel.dev")
        .serial_number(get_serial_str())
        .device_class(USB_CLASS_CDC)
        .build();

    let mut timer = dp.TIM2.counter(&clocks);
    //timer.start(100.millis()).unwrap();
    timer.start(5.secs()).unwrap();
    timer.listen(Event::Update);
    cortex_m::interrupt::free(|cs| *TIMER.borrow(cs).borrow_mut() = Some(timer));
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }

    let spi1 = dp.SPI1.spi(
        (gpioa.pa5, gpioa.pa6, gpioa.pa7),
        mcp23s17::MODE,
        2000.kHz(),
        &clocks,
    );
    let spi_bus = shared_bus::BusManagerSimple::new(spi1);

    let mut tmc5072_z = tmc5072::Tmc5072::new(spi_bus.acquire_spi(), gpiob.pb0.into_push_pull_output().erase()).unwrap();
    let mut tmc5072_x = tmc5072::Tmc5072::new(spi_bus.acquire_spi(), gpiob.pb1.into_push_pull_output().erase()).unwrap();
    let mut tmc5072_y = tmc5072::Tmc5072::new(spi_bus.acquire_spi(), gpiob.pb2.into_push_pull_output().erase()).unwrap();
    let mut tmc5072_rf = tmc5072::Tmc5072::new(spi_bus.acquire_spi(), gpiob.pb10.into_push_pull_output().erase()).unwrap();
    let mut tmc5072_rb = tmc5072::Tmc5072::new(spi_bus.acquire_spi(), gpiob.pb12.into_push_pull_output().erase()).unwrap();

    motor_setup_all([&mut tmc5072_x, &mut tmc5072_y, &mut tmc5072_z, &mut tmc5072_rf, &mut tmc5072_rb]).unwrap();

    let mut mcp23s17 = mcp23s17::MCP23S17::default(spi_bus.acquire_spi(), gpiob.pb5.into_push_pull_output()).unwrap();
    let _mcp23s17_irq = gpioa.pa4;

    /* initialize GPIOs */
    mcp23s17.digital_write(0, true).unwrap();
    mcp23s17.pin_mode(0, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_R_EN
    mcp23s17.digital_write(1, false).unwrap();
    mcp23s17.pin_mode(1, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_FL
    mcp23s17.digital_write(2, false).unwrap();
    mcp23s17.pin_mode(2, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_FR
    mcp23s17.digital_write(3, false).unwrap();
    mcp23s17.pin_mode(3, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_BL
    mcp23s17.digital_write(4, false).unwrap();
    mcp23s17.pin_mode(4, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_BR
    mcp23s17.digital_write(5, false).unwrap();
    mcp23s17.pin_mode(5, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_PUMP_2
    mcp23s17.digital_write(6, false).unwrap();
    mcp23s17.pin_mode(6, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_PUMP_1
    mcp23s17.pin_mode(7, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_DET_24V
    mcp23s17.pin_mode(8, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_XR
    mcp23s17.pin_mode(9, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_XL
    mcp23s17.pin_mode(10, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YLB
    mcp23s17.pin_mode(11, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YLF
    mcp23s17.pin_mode(12, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YRB
    mcp23s17.pin_mode(13, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YRF
    mcp23s17.digital_write(14, true).unwrap();
    mcp23s17.pin_mode(14, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_XY_EN
    mcp23s17.digital_write(15, true).unwrap();
    mcp23s17.pin_mode(15, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_Z_EN

    let ledspi = dp.SPI2.spi(
        (gpiob.pb13, gpiob.pb14, gpiob.pb15),
        ws2812::MODE,
        2500.kHz(),
        &clocks,
    );

    let mut buf = [0u8; 20+NUM_LEDS*4*4];
    let mut ws = ws2812::prerendered::Ws2812::new_sk6812w(ledspi, &mut buf);

    let i2c = I2c::new(
        dp.I2C1,
        (
            gpiob.pb8.into_alternate_open_drain(), // scl
            gpiob.pb9.into_alternate_open_drain(), // sda
        ),
        100.kHz(),
        &clocks,
    );
    let mut ina219 = ina219::INA219::new(i2c, 0x40);

    let mut rgbctrl = RGBcs {
        channel1: gpioc.pc5.into_push_pull_output_in_state(PinState::Low).erase(),
        channel2: gpioc.pc4.into_push_pull_output_in_state(PinState::Low).erase(),
        channel3: gpioc.pc3.into_push_pull_output_in_state(PinState::Low).erase(),
        channel4: gpioc.pc2.into_push_pull_output_in_state(PinState::Low).erase(),
        channel5: gpioc.pc1.into_push_pull_output_in_state(PinState::Low).erase(),
        channel6: gpioc.pc0.into_push_pull_output_in_state(PinState::Low).erase(),
    };

    rgbctrl.channel5.set_high();
    ws.write(ledset_rgbw(LedColor::DarkWhite).iter().cloned()).unwrap();
    rgbctrl.channel5.set_low();

    rgbctrl.channel6.set_high();
    ws.write([smart_leds::RGBW {r:0, g:10, b:0, a:smart_leds::White(0)}; 1].iter().cloned()).unwrap();
    rgbctrl.channel6.set_low();

    let mut cam_light_head_enabled = false;
    let mut cam_light_bed_enabled = false;

    let mut io = IO::new(SerialWrapper::new(&mut usb_dev, &mut serial));
    let mut editor = loop {
        if !io.inner().poll() || !io.inner().serial.dtr() || !io.inner().serial.rts() {
            continue;
        }

        // If attempting to write before reading, the next read will
        // get occasional garbage input. I'm not sure where the
        // garbage comes from, but it could be a bug in usb-device or
        // usbd-serial. Becase noline needs to write during
        // initialization, I've added this blocking read here to wait
        // for user input before proceeding.
        block!(io.inner().read()).unwrap();
        break EditorBuilder::new_static::<128>()
            .with_static_history::<128>()
            .build_sync(&mut io)
            .unwrap();
    };

    loop {
        match editor.readline("> ", &mut io) {
            Ok(s) => {
                if s.len() > 0 {
                    let mut split = s.split(' ');
                    let cmd = split.next();
                    match cmd {
                        Some("reset") => {
                            writeln!(io, "reset...\r\n").unwrap();
                            system_reset();
                        },
                        Some("bootloader") => {
                            writeln!(io, "switch to DFU bootloader...\r\n").unwrap();
                            bootloader_set_magic(0xb007c0de);
                            system_reset();
                        },
                        Some("echo") => {
                            writeln!(io, "{}\r", &s[5..]).unwrap();
                        },
                        Some("light") => {
                            let color = match split.next() {
                                Some("off") => Some(LedColor::Off),
                                Some("test") => Some(LedColor::Test),
                                Some("test2") => Some(LedColor::Test2),
                                Some("test3") => Some(LedColor::Test3),
                                Some("white") => Some(LedColor::White),
                                Some("dim") => Some(LedColor::DarkWhite),
                                Some("red") => Some(LedColor::Red),
                                Some("green") => Some(LedColor::Green),
                                Some("blue") => Some(LedColor::Blue),
                                Some("max") => Some(LedColor::All),
                                _ => None,
                            };
                            match color {
                                Some(c) => {
                                    rgbctrl.channel5.set_high();
                                    ws.write(ledset_rgbw(c).iter().cloned()).unwrap();
                                    rgbctrl.channel5.set_low();
                                },
                                None => {
                                    writeln!(io, "unsupported color\r\n").unwrap();
                                }
                            }
                        },
                        Some("ina219") => {
                            match ina219.calibrate(0x5000) {
                                Ok(_) => {}
                                _ => { writeln!(io, "failed to write calibration register\r\n").unwrap(); }
                            }
                            match ina219.voltage() {
                                Ok(voltage) => { writeln!(io, "ina219 voltage: {}.{} V\r", voltage/1000, voltage%1000).unwrap(); },
                                _ => { writeln!(io, "ina219: failed to read voltage\r").unwrap(); }
                            }
                            match ina219.current() {
                                Ok(current) => { let current = current.abs(); writeln!(io, "ina219 current: {}.{} A\r", current/1000, current%1000).unwrap(); },
                                _ => { writeln!(io, "ina219: failed to read current\r").unwrap(); }
                            }
                            match ina219.power() {
                                Ok(power) => { let power = (power as u32) * 20; writeln!(io, "ina219 power:   {}.{} W\r", power/1000, power%1000).unwrap(); },
                                _ => { writeln!(io, "ina219: failed to read power\r").unwrap(); }
                            }
                        },
                        Some("mcp23s17") => {
                            if split.next() == Some("init") {
                                mcp23s17.pin_mode(0, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_R_EN
                                mcp23s17.pin_mode(1, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_FL
                                mcp23s17.pin_mode(2, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_FR
                                mcp23s17.pin_mode(3, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_BL
                                mcp23s17.pin_mode(4, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_VALVE_BR
                                mcp23s17.pin_mode(5, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_PUMP_2
                                mcp23s17.pin_mode(6, mcp23s17::PinMode::OUTPUT).unwrap(); // CTRL_PUMP_1
                                mcp23s17.pin_mode(7, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_DET_24V
                                mcp23s17.pin_mode(8, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_XR
                                mcp23s17.pin_mode(9, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_XL
                                mcp23s17.pin_mode(10, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YLB
                                mcp23s17.pin_mode(11, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YLF
                                mcp23s17.pin_mode(12, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YRB
                                mcp23s17.pin_mode(13, mcp23s17::PinMode::INPUT).unwrap(); // CTRL_SW_YRF
                                mcp23s17.pin_mode(14, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_XY_EN
                                mcp23s17.pin_mode(15, mcp23s17::PinMode::OUTPUT).unwrap(); // ~TMC_Z_EN
                            } else {
                                writeln!(io, "only 'mcp23s17 init' is supported!\r").unwrap();
                            }
                        },
                        Some("switch") => {
                            if split.next() == Some("get") {
                                let channel = match split.next() {
                                    Some("24v") => Some(MCP23S17Input::VIN),
                                    Some("xr") => Some(MCP23S17Input::XR),
                                    Some("xl") => Some(MCP23S17Input::XL),
                                    Some("ylb") => Some(MCP23S17Input::YLB),
                                    Some("ylf") => Some(MCP23S17Input::YLF),
                                    Some("yrb") => Some(MCP23S17Input::YRB),
                                    Some("yrf") => Some(MCP23S17Input::YRF),
                                    _ => None,
                                };

                                let channel = match channel {
                                    Some(MCP23S17Input::VIN) => Some(7),
                                    Some(MCP23S17Input::XR) => Some(8),
                                    Some(MCP23S17Input::XL) => Some(9),
                                    Some(MCP23S17Input::YLB) => Some(10),
                                    Some(MCP23S17Input::YLF) => Some(11),
                                    Some(MCP23S17Input::YRB) => Some(12),
                                    Some(MCP23S17Input::YRF) => Some(13),
                                    None => None,
                                };

                                if channel.is_some() {
                                    let val = !mcp23s17.digital_read(channel.unwrap()).unwrap();
                                    writeln!(io, "switch state: {}\r", val).unwrap();
                                } else {
                                    writeln!(io, "Invalid switch name\r").unwrap();
                                }
                            } else {
                                writeln!(io, "Only supports 'switch get <switch>'\r").unwrap();
                            }
                        }
                        Some("valve") => {
                            let valve = match split.next() {
                                Some("1") => Some(1),
                                Some("2") => Some(2),
                                Some("3") => Some(3),
                                Some("4") => Some(4),
                                _ => None,
                            };

                            let cmd = DigitalOutputCmd::from_str(split.next().unwrap_or(""));

                            if valve.is_some() && cmd.is_ok() {
                                let enabled = match cmd.unwrap() {
                                    DigitalOutputCmd::Toggle => !mcp23s17.digital_read(1).unwrap(),
                                    DigitalOutputCmd::On => true,
                                    DigitalOutputCmd::Off => false,
                                };
                                mcp23s17.digital_write(valve.unwrap(), enabled).unwrap();
                            } else {
                                writeln!(io, "invalid valve command, use 'valve <id> <on/off/toggle>'!\r").unwrap();
                            }
                        },
                        Some("pumps") => {
                            let cmd = DigitalOutputCmd::from_str(split.next().unwrap_or(""));

                            if cmd.is_ok() {
                                let enabled = match cmd.unwrap() {
                                    DigitalOutputCmd::Toggle => !mcp23s17.digital_read(5).unwrap(),
                                    DigitalOutputCmd::On => true,
                                    DigitalOutputCmd::Off => false,
                                };
                                mcp23s17.digital_write(5, enabled).unwrap();
                                mcp23s17.digital_write(6, enabled).unwrap();
                            } else {
                                writeln!(io, "invalid pumps command, use 'pumps <on/off/toggle>'!\r").unwrap();
                            }
                        },
                        Some("cam-light") => {
                            let channelinfo = match split.next().unwrap_or("") {
                                "bed" => Some((&mut rgbctrl.channel1, &mut cam_light_head_enabled)),
                                "head" => Some((&mut rgbctrl.channel2, &mut cam_light_bed_enabled)),
                                _ => None,
                            };

                            let cmd = DigitalOutputCmd::from_str(split.next().unwrap_or(""));

                            if cmd.is_ok() && channelinfo.is_some() {
                                let channelinfo = channelinfo.unwrap();

                                *channelinfo.1 = match cmd.unwrap() {
                                    DigitalOutputCmd::Toggle => !*channelinfo.1,
                                    DigitalOutputCmd::On => true,
                                    DigitalOutputCmd::Off => false,
                                };
                                channelinfo.0.set_high();
                                ws.rgb_mode(true);
                                ws.write(ledset_rgb(*channelinfo.1).iter().cloned()).unwrap();
                                ws.rgb_mode(false);
                                channelinfo.0.set_low();
                            } else {
                                writeln!(io, "invalid cam-light command, use 'cam-light <head/bed> <on/off/toggle>'!\r").unwrap();
                            }
                        },
                        Some("goto") => {
                            let target = split.next().unwrap_or("");

                            match target {
                                "bedcam" => {
                                    tmc5072_y.position_set(tmc5072::Motor::Motor1, BED_CAM_POS[1]).unwrap();
                                    tmc5072_y.position_set(tmc5072::Motor::Motor2, BED_CAM_POS[1]).unwrap();
                                    tmc5072_x.position_set(tmc5072::Motor::Motor1, BED_CAM_POS[0]).unwrap();
                                },
                                "bookmark" => {
                                    let bookmark = split.next().unwrap_or("");
                                    let bookmark = u8::from_str(bookmark);
                                    if bookmark.is_err() {
                                        writeln!(io, "could not parse bookmark number").unwrap();
                                    } else {
                                        let bookmark = bookmark.unwrap();
                                        let (x_usteps, y_usteps) = bookmark_get(bookmark);
                                        if x_usteps == 0 && y_usteps == 0 {
                                            writeln!(io, "empty bookmark").unwrap();
                                        } else {
                                            tmc5072_y.position_set(tmc5072::Motor::Motor1, y_usteps).unwrap();
                                            tmc5072_y.position_set(tmc5072::Motor::Motor2, y_usteps).unwrap();
                                            tmc5072_x.position_set(tmc5072::Motor::Motor1, x_usteps).unwrap();
                                        }
                                    }
                                },
                                _ => {
                                    writeln!(io, "invalid goto command, use 'goto bedcam'!\r").unwrap();
                                },
                            };
                        },
                        Some("move") => {
                            let mode = PositioningMode::from_str(split.next().unwrap_or(""));
                            let pos1 = Position::from_str(split.next().unwrap_or(""));
                            let pos2 = Position::from_str(split.next().unwrap_or(""));

                            if mode.is_err() {
                                writeln!(io, "Error: invalid mode!\r").unwrap();
                            } if pos1.is_err() {
                                writeln!(io, "Error: invalid position!\r").unwrap();
                            } else if pos1.is_ok() && pos2.is_ok() && pos1.unwrap().axis == pos2.unwrap().axis {
                                writeln!(io, "Error: same axis was supplied two times!\r").unwrap();
                            } else {
                                let coordinate_um = Coordinate::new(mode.unwrap(), pos1.unwrap(), pos2.ok());
                                let coordinate_usteps = coordinate_um.um_to_usteps();

                                let current_x_usteps = tmc5072_x.position_get(tmc5072::Motor::Motor1);
                                let current_y_usteps = tmc5072_y.position_get(tmc5072::Motor::Motor1);

                                if current_x_usteps.is_ok() && current_y_usteps.is_ok() {
                                    let coordinate_usteps_abs = coordinate_usteps.to_absolute(current_x_usteps.unwrap(), current_y_usteps.unwrap());
                                    let target_x = coordinate_usteps_abs.pos_x.unwrap(); // to_absolute ensures that pos_x has data
                                    let target_y = coordinate_usteps_abs.pos_y.unwrap(); // to_absolute ensures that pos_y has data

                                    tmc5072_y.position_set(tmc5072::Motor::Motor1, target_y).unwrap();
                                    tmc5072_y.position_set(tmc5072::Motor::Motor2, target_y).unwrap();
                                    tmc5072_x.position_set(tmc5072::Motor::Motor1, target_x).unwrap();
                                } else {
                                    writeln!(io, "Error: failed to get current position!\r").unwrap();
                                }
                            }
                        },
                        Some("position") => {
                            let current_x_usteps = tmc5072_x.position_get(tmc5072::Motor::Motor1).unwrap();
                            let current_y_usteps = tmc5072_y.position_get(tmc5072::Motor::Motor1).unwrap();
                            let pos_x = Position::from_usteps(Axis::X, current_x_usteps);
                            let pos_y = Position::from_usteps(Axis::Y, current_y_usteps);

                            writeln!(io, "Position: X={}.{} Y={}.{}\r", pos_x.position_um/1000, pos_x.position_um%1000, pos_y.position_um/1000, pos_y.position_um % 1000).unwrap();
                        },
                        Some("bookmark") => {
                            let subcmd = split.next().unwrap_or("");
                            let id = split.next().unwrap_or("");
                            let id = u8::from_str(id);

                            if id.is_err() {
                                writeln!(io, "could not parse bookmark number").unwrap();
                            } else {
                                let id = id.unwrap();
                                match subcmd {
                                    "set" => {
                                        let x = tmc5072_x.position_get(tmc5072::Motor::Motor1).unwrap();
                                        let y = tmc5072_y.position_get(tmc5072::Motor::Motor1).unwrap();
                                        if !bookmark_set(id, x, y) {
                                            writeln!(io, "bookmark number out of range").unwrap();
                                        }
                                    },
                                    "get" => {
                                        let (x_usteps, y_usteps) = bookmark_get(id);
                                        let pos_x = Position::from_usteps(Axis::X, x_usteps);
                                        let pos_y = Position::from_usteps(Axis::Y, y_usteps);
                                        writeln!(io, "Position: X={}.{} Y={}.{}\r", pos_x.position_um/1000, pos_x.position_um%1000, pos_y.position_um/1000, pos_y.position_um % 1000).unwrap();
                                    },
                                    "reset" => {
                                        bookmark_set(id, 0, 0);
                                    },
                                    _ => {
                                        writeln!(io, "usage: bookmark <set|get|reset> <number>").unwrap();
                                    },
                                }
                            }
                        },
                        Some("focus")|Some("unfocus") => {
                            let cmd = cmd.unwrap();
                            let head = split.next().unwrap_or("");
                            let head = PNPHead::from_str(head);

                            if head.is_ok() {
                                let head = head.unwrap();

                                let motor = match head {
                                    PNPHead::FrontLeft => tmc5072::Motor::Motor1,
                                    PNPHead::FrontRight => tmc5072::Motor::Motor1,
                                    PNPHead::BackLeft => tmc5072::Motor::Motor2,
                                    PNPHead::BackRight => tmc5072::Motor::Motor2,
                                };

                                let target = if cmd == "focus" {
                                    match head {
                                        PNPHead::FrontLeft => 6000,
                                        PNPHead::BackLeft => 6000,
                                        PNPHead::FrontRight => -6000,
                                        PNPHead::BackRight => -6000,
                                    }
                                } else {
                                    0
                                };

                                tmc5072_z.position_set(motor, target).unwrap();
                                let result = tmc5072_z.position_wait(motor, target, 10000); // ~2000/sec? -> 5sec?

                                if !result.is_ok() {
                                    writeln!(io, "failed to wait, motors not initialized/enabled?\r").unwrap();
                                }
                            } else {
                                    writeln!(io, "invalid head, specify one of fl/fr/bl/br!\r").unwrap();
                            }
                        },
                        Some("pick")|Some("place") => {
                            let cmd = cmd.unwrap();
                            let head = split.next().unwrap_or("");
                            let head = PNPHead::from_str(head);

                            if head.is_ok() {
                                let head = head.unwrap();

                                let motor = match head {
                                    PNPHead::FrontLeft => tmc5072::Motor::Motor1,
                                    PNPHead::FrontRight => tmc5072::Motor::Motor1,
                                    PNPHead::BackLeft => tmc5072::Motor::Motor2,
                                    PNPHead::BackRight => tmc5072::Motor::Motor2,
                                };

                                let target = match head {
                                    PNPHead::FrontLeft => 7500,
                                    PNPHead::BackLeft => 7500,
                                    PNPHead::FrontRight => -7500,
                                    PNPHead::BackRight => -7500,
                                };

                                // 1. move head down
                                tmc5072_z.position_set(motor, target).unwrap();
                                let result = tmc5072_z.position_wait(motor, target, 10000); // ~2000/sec? -> 5sec?

                                if result.is_ok() {
                                    // 2. update valve
                                    match head {
                                        PNPHead::FrontLeft => mcp23s17.digital_write(1, cmd == "pick").unwrap(),
                                        PNPHead::FrontRight => mcp23s17.digital_write(2, cmd == "pick").unwrap(),
                                        PNPHead::BackLeft => mcp23s17.digital_write(3, cmd == "pick").unwrap(),
                                        PNPHead::BackRight => mcp23s17.digital_write(4, cmd == "pick").unwrap(),
                                    }

                                    // 3. move head back
                                    tmc5072_z.position_set(motor, 0).unwrap();
                                    tmc5072_z.position_wait(motor, 0, 10000).unwrap();
                                } else {
                                    writeln!(io, "failed to wait, motors not initialized/enabled?\r").unwrap();
                                }
                            } else {
                                    writeln!(io, "invalid head, specify one of fl/fr/bl/br!\r").unwrap();
                            }
                        },
                        Some("nozzle-to-cam")|Some("cam-to-nozzle") => {
                            let to_cam = cmd.unwrap() == "nozzle-to-cam";
                            let head = split.next().unwrap_or("");
                            let head = PNPHead::from_str(head);

                            if head.is_ok() {
                                let head = head.unwrap();

                                let pos_x = tmc5072_x.position_get(tmc5072::Motor::Motor1);
                                let pos_y = tmc5072_y.position_get(tmc5072::Motor::Motor1);

                                if pos_x.is_ok() && pos_y.is_ok() {
                                    let pos_x = pos_x.unwrap();
                                    let pos_y = pos_y.unwrap();

                                    let offsets = match head {
                                        PNPHead::FrontLeft  => (-547840, -2737493),
                                        PNPHead::FrontRight => ( 535893, -2737493),
                                        PNPHead::BackLeft   => (0, 0), // TODO: FIXME: missing offset
                                        PNPHead::BackRight  => (0, 0), // TODO: FIXME: missing offset
                                    };

                                    let target_x = if to_cam { pos_x - offsets.0 } else { pos_x + offsets.0 };
                                    let target_y = if to_cam { pos_y - offsets.1 } else { pos_y + offsets.1 };

                                    tmc5072_y.position_set(tmc5072::Motor::Motor1, target_y).unwrap();
                                    tmc5072_y.position_set(tmc5072::Motor::Motor2, target_y).unwrap();
                                    tmc5072_x.position_set(tmc5072::Motor::Motor1, target_x).unwrap();
                                } else {
                                    writeln!(io, "could not get current motor position!\r").unwrap();
                                }
                            } else {
                                    writeln!(io, "invalid head, specify one of fl/fr/bl/br!\r").unwrap();
                            }
                        },
                        Some("home") => {
                            let axis = split.next();
                            match axis {
                                Some("X")|Some("x") => {
                                    motor_autohome_full(&mut tmc5072_x, tmc5072::Motor::Motor1).unwrap();
                                },
                                Some("Y")|Some("y") => {
                                    motor_autohome_dual_full(&mut tmc5072_y, 200*256*223).unwrap(); // Length: ~334.5mm
                                },
                                Some("XY")|Some("xy")|Some("both") => {
                                    motor_autohome_full(&mut tmc5072_x, tmc5072::Motor::Motor1).unwrap();
                                    motor_autohome_dual_full(&mut tmc5072_y, 200*256*223).unwrap(); // Length: ~334.5mm
                                },
                                _ => {
                                    writeln!(io, "invalid axis, specify X or Y!\r").unwrap();
                                },
                            };
                        },
                        Some("tmc5072")|Some("tmc")|Some("trinamic") => {
                            let subcmd = split.next();
                            match subcmd {
                                Some("enable") => {
                                    mcp23s17.digital_write(0, false).unwrap(); // ~TMC_R_EN
                                    mcp23s17.digital_write(14, false).unwrap(); // ~TMC_XY_EN
                                    mcp23s17.digital_write(15, false).unwrap(); // ~TMC_Z_EN
                                    writeln!(io, "set all Trinamic DRV_ENN pins to low!\r").unwrap();
                                },
                                Some("disable") => {
                                    mcp23s17.digital_write(0, true).unwrap(); // ~TMC_R_EN
                                    mcp23s17.digital_write(14, true).unwrap(); // ~TMC_XY_EN
                                    mcp23s17.digital_write(15, true).unwrap(); // ~TMC_Z_EN
                                    writeln!(io, "set all Trinamic DRV_ENN pins to high!\r").unwrap();
                                },
                                Some("x") | Some("y") | Some("z") | Some("rf") | Some("rb") => {
                                    let channel = subcmd.unwrap();
                                    let tmc = match channel {
                                        "x" => &mut tmc5072_x,
                                        "y" => &mut tmc5072_y,
                                        "z" => &mut tmc5072_z,
                                        "rf" => &mut tmc5072_rf,
                                        "rb" => &mut tmc5072_rb,
                                        _ => panic!(),
                                    };

                                    let subsubcmd = split.next();
                                    match subsubcmd {
                                        Some("reg-get") => {
                                            let addr = split.next();
                                            if addr.is_some() {
                                                let addr = addr.unwrap().trim_start_matches("0x");
                                                let addr = u8::from_str_radix(addr, 16);

                                                if addr.is_ok() {
                                                    let addr = addr.unwrap();
                                                    let val = tmc.read_register(tmc5072::Registers::try_from(addr).unwrap()).unwrap();
                                                    writeln!(io, "tmc5072_{}.read_register({:#04x}) = {}\r", channel, addr, val).unwrap();
                                                } else {
                                                    writeln!(io, "register address is not valid hexadecimal!\r").unwrap();
                                                }
                                            } else {
                                                writeln!(io, "missing register address!\r").unwrap();
                                            }
                                        },
                                        Some("reg-set") => {
                                            let addr = split.next();
                                            let val = split.next();

                                            if addr.is_some() && val.is_some() {
                                                let addr = addr.unwrap().trim_start_matches("0x");
                                                let addr = u8::from_str_radix(addr, 16);

                                                let val = val.unwrap().trim_start_matches("0x");
                                                let val = u32::from_str_radix(val, 16);

                                                if addr.is_ok() && val.is_ok() {
                                                    let addr = addr.unwrap();
                                                    let val = val.unwrap();
                                                    tmc.write_register(tmc5072::Registers::try_from(addr).unwrap(), val).unwrap();
                                                    writeln!(io, "tmc5072_{}.write_register({:#04x}, {:#018x})\r", channel, addr, val).unwrap();
                                                } else {
                                                    writeln!(io, "register address or value is not valid hexadecimal!\r").unwrap();
                                                }
                                            } else {
                                                writeln!(io, "missing register address or value!\r").unwrap();
                                            }
                                        },
                                        Some("test") => {
                                            // continously enable front-right rotation motor
                                            tmc.write_register(tmc5072::Registers::GCONF, 0x00000000).unwrap();
                                            tmc.write_register(tmc5072::Registers::MOTOR2_CHOPCONF, 0x000100C5).unwrap(); // CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
                                            tmc.write_register(tmc5072::Registers::MOTOR2_IHOLD_IRUN, 0x00011F05).unwrap(); // IHOLD_IRUN: IHOLD=5, IRUN=31 (max. current), IHOLDDELAY=1
                                            tmc.write_register(tmc5072::Registers::MOTOR2_PWMCONF, 0x00011F05).unwrap(); // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
                                            tmc.write_register(tmc5072::Registers::MOTOR2_VHIGH, 0x00061A80).unwrap(); // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
                                            tmc.write_register(tmc5072::Registers::MOTOR2_VCOOLTHRS, 0x00007530).unwrap(); // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
                                            tmc.write_register(tmc5072::Registers::MOTOR2_AMAX, 0x00001388).unwrap(); // AMAX = 5000
                                            tmc.write_register(tmc5072::Registers::MOTOR2_DMAX, 0x00001388).unwrap(); // DMAX = 5000
                                            tmc.write_register(tmc5072::Registers::MOTOR2_VMAX, 0x0000C350).unwrap(); // VMAX = 50000 (maximum value is 0x7ffe00)
                                            tmc.write_register(tmc5072::Registers::MOTOR2_VSTOP, 0x0000000A).unwrap(); // VSTOP = 10 Stop velocity (Near to zero)
                                            tmc.write_register(tmc5072::Registers::MOTOR2_RAMPMODE, 0x00000001).unwrap(); // RAMPMODE = 1
                                        },
                                        Some("dual-set-home") => {
                                            writeln!(io, "set home at current position\r").unwrap();
                                            tmc.position_set_home(tmc5072::Motor::Motor1, 0).unwrap();
                                            tmc.position_set_home(tmc5072::Motor::Motor2, 0).unwrap();
                                        },
                                        Some("dual-autohome") => {
                                            writeln!(io, "autohome using TMC StallGuard 2...\r").unwrap();
                                            motor_autohome_dual(tmc).unwrap();
                                        },
                                        Some("dual-autohome-full") => {
                                            writeln!(io, "full autohome using TMC StallGuard 2...\r").unwrap();
                                            motor_autohome_dual_full(tmc, 0).unwrap();
                                        },
                                        Some("dual-usteps") => {
                                            let usteps = split.next();
                                            if usteps.is_some() {
                                                let target = i32::from_str_radix(usteps.unwrap(), 10);
                                                if target.is_ok() {
                                                    let target = target.unwrap();

                                                    /* Write position target */
                                                    tmc.position_set(tmc5072::Motor::Motor1, target).unwrap();
                                                    tmc.position_set(tmc5072::Motor::Motor2, target).unwrap();
                                                } else {
                                                    writeln!(io, "invalid µStep argument (must be 32 bit signed integer in base 10)\r").unwrap();
                                                }
                                            } else {
                                                writeln!(io, "missing argument: µSteps!\r").unwrap();
                                            }
                                        },
                                        Some("motor1")|Some("motor2") => {
                                            let motor = match subsubcmd.unwrap() {
                                                "motor1" => tmc5072::Motor::Motor1,
                                                "motor2" => tmc5072::Motor::Motor2,
                                                _ => panic!(),
                                            };

                                            match split.next() {
                                                Some("init") => {
                                                    motor_setup_one(tmc, motor).unwrap();
                                                },
                                                Some("autohome") => {
                                                    writeln!(io, "autohome using TMC StallGuard 2...\r").unwrap();
                                                    motor_autohome(tmc, motor).unwrap();
                                                },
                                                Some("autohome-full") => {
                                                    writeln!(io, "full autohome using TMC StallGuard 2...\r").unwrap();
                                                    motor_autohome_full(tmc, motor).unwrap();
                                                },
                                                Some("clear-sg") => {
                                                    tmc.read_motor_register(motor, tmc5072::MotorRegister::RAMP_STAT).unwrap();
                                                },
                                                Some("set-home") => {
                                                    writeln!(io, "set home at current position\r").unwrap();
                                                    tmc.position_set_home(motor, 0).unwrap();
                                                },
                                                Some("move-home") => {
                                                    writeln!(io, "move to 0\r").unwrap();
                                                    tmc.position_set(motor, 0).unwrap();
                                                },
                                                Some("move-1") => {
                                                    writeln!(io, "move to -1 rotation (-51200 µSteps)\r").unwrap();
                                                    tmc.position_set(motor, -(200*256)).unwrap(); // negative = counter clockwise, 200 steps per rotation, 256 microsteps per step
                                                },
                                                Some("move-usteps") => {
                                                    let usteps = split.next();
                                                    if usteps.is_some() {
                                                        let target = i32::from_str_radix(usteps.unwrap(), 10);
                                                        if target.is_ok() {
                                                            tmc.position_set(motor, target.unwrap()).unwrap();
                                                        } else {
                                                            writeln!(io, "invalid µStep argument (must be 32 bit signed integer in base 10)\r").unwrap();
                                                        }
                                                    } else {
                                                        writeln!(io, "missing argument: µSteps!\r").unwrap();
                                                    }
                                                },
                                                _ => {
                                                    writeln!(io, "invalid tmc5072 motor command. Supported commands: init/move-home/move-1/move-usteps!\r").unwrap();
                                                },
                                            }
                                        }
                                        _ => {
                                            writeln!(io, "invalid tmc5072 sub-command. Supported commands: reg-get/reg-set/test/motor1/motor2!\r").unwrap();
                                        },
                                    };
                                },
                                _ => {
                                    writeln!(io, "invalid tmc5072 command, use 'tmc5072 <enable/disable/[channel]>' with channel being one of x/y/z/rf/rb'!\r").unwrap();
                                },
                            }
                        },
                        _ => {
                            writeln!(io, "Unsupported Command: {}\r", s).unwrap();
                        },
                    }
                } else {
                    // Writing emtpy slice causes panic
                    writeln!(io, "Missing Command\r").unwrap();
                }
            },
            Err(err) => {
                let error = match err {
                    Error::WriteError(err) | Error::ReadError(err) => match err {
                        UsbError::WouldBlock => "Wouldblock",
                        UsbError::ParseError => "ParseEror",
                        UsbError::BufferOverflow => "BufferOverflow",
                        UsbError::EndpointOverflow => "EndpointOverflow",
                        UsbError::EndpointMemoryOverflow => "EndpointMemoryOverflow",
                        UsbError::InvalidEndpoint => "InvalidEndpoint",
                        UsbError::Unsupported => "Unsupported",
                        UsbError::InvalidState => "InvalidState",
                    },
                    Error::ParserError => "ParserError",
                    Error::Aborted => "Aborted",
                };

                writeln!(io, "Error: {}\r", error).unwrap();
            }
        }
    }
}
