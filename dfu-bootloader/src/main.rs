//! # DFU bootloader
//!
//! There are two modes of operation: minimal and DFU.
//!
//! After reset, bootloader starts in a minimal mode,
//! it's goal is to determine if bootloader must switch
//! to DFU mode, and if not, try to jump to a main
//! firmware.
//!
//! In minimal mode, the bootloader checks for a magic
//! value in RAM and the first few bytes of a firmware
//! (should look like a proper stack pointer).
//!
//! When DFU mode is active required peripherals and USB
//! are enabled, host can issue DFU commands.
//!
//! First 0x10 bytes of RAM are reserved. In "memory.x" linker script
//! RAM section has 0x10 offset from an actual RAM start. The first
//! 4 bytes of RAM may have a magic value to force the bootloader
//! to enter DFU mode programmatically. Both DFU and main firmware
//! must agree on used addresses and values for this to work.
//!

#![no_std]
#![no_main]

use core::str;
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{pac, prelude::*};
use stm32f4xx_hal::pac::{interrupt, RCC, TIM2};
use stm32f4xx_hal::rcc::{Enable, Reset, BusTimerClock};
use stm32f4xx_hal::timer::{Event};
use stm32f4xx_hal::otg_fs::{UsbBus, USB, UsbBusType};
use stm32f4xx_hal::flash::{FlashExt, LockedFlash, UnlockedFlash};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_dfu::*;
use core::mem::MaybeUninit;

/// If this value is found at the address 0x2000_0000 (beginning of RAM),
/// bootloader will enter DFU mode.
const KEY_STAY_IN_BOOT: u32 = 0xb007c0de;

/// If this value is found at the address 0x2000_0000 (beginning of RAM),
/// bootloader will skip timer.
const KEY_GOTO_PROGRAM: u32 = 0xbadb007;

/// Board flash configuration. MEM_INFO_STRING below must also be changed.
const FLASH_START: u32 = 0x0800_0000;
const FLASH_SIZE_BYTES: u32 = 256 * 1024;
const BOOTLOADER_SIZE_BYTES: u32 = 16 * 1024;
const FW_ADDRESS: u32 = 0x0800_4000;

static mut FLASH: MaybeUninit<LockedFlash> = MaybeUninit::uninit();
static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBusType>> = MaybeUninit::uninit();
static mut USB_DEVICE: MaybeUninit<UsbDevice<UsbBusType>> = MaybeUninit::uninit();
static mut USB_DFU: MaybeUninit<DFUClass<UsbBusType, STM32Mem>> = MaybeUninit::uninit();
static mut EP_MEMORY: [u32; 256] = [0; 256];

#[derive(PartialEq)]
enum BootMode {
    UNKNOWN,
    BOOTLOADER,
    PROGRAM,
}

pub struct STM32Mem<'a> {
    unlocked: UnlockedFlash<'a>,
    buffer: [u8; 256],
}

impl<'a> STM32Mem<'a> {
    fn new(locked: &'a mut LockedFlash) -> Self {
        Self {
            unlocked: locked.unlocked(),
            buffer: [0; 256],
        }
    }
}

fn stop_bootloader_timeout() {
    pac::NVIC::mask(stm32f4xx_hal::pac::Interrupt::TIM2);
}

impl<'a> DFUMemIO for STM32Mem<'a> {
    const INITIAL_ADDRESS_POINTER: u32 = FLASH_START;
    const PROGRAM_TIME_MS: u32 = 500; // time it takes to program 128 bytes
    const ERASE_TIME_MS: u32 = 50;
    const FULL_ERASE_TIME_MS: u32 = 50 * 240;
    const TRANSFER_SIZE: u16 = 256; // max. usb-device limit

    const MEM_INFO_STRING: &'static str = "@Flash/0x08000000/1*16Ka,3*16Kg,1*64Kg,1*128Kg";
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;

    fn read(&mut self, address: u32, length: usize) -> core::result::Result<&[u8], DFUMemError> {
        stop_bootloader_timeout();
        let flash_top: u32 = FLASH_START + FLASH_SIZE_BYTES;

        if address < FLASH_START {
            return Err(DFUMemError::Address);
        }
        if address >= flash_top {
            return Ok(&[]);
        }

        let len = length.min((flash_top - address) as usize);

        let mem = unsafe { &*core::ptr::slice_from_raw_parts(address as *const u8, len) };
        Ok(mem)
    }

    fn erase(&mut self, address: u32) -> core::result::Result<(), DFUMemError> {
        stop_bootloader_timeout();
        let sector = match address {
            0x08004000 => 1,
            0x08008000 => 2,
            0x0800c000 => 3,
            0x08010000 => 4,
            _ => { return Err(DFUMemError::Address) },
        };

        // does not erase?!
        match self.unlocked.erase(sector) {
            Ok(_) => Ok(()),
            Err(stm32f4xx_hal::flash::Error::Operation) => Err(DFUMemError::Erase),
            Err(_) => Err(DFUMemError::Unknown),
        }
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        Err(DFUMemError::Unknown)
    }

    fn store_write_buffer(&mut self, src: &[u8]) -> core::result::Result<(), ()> {
        stop_bootloader_timeout();
        self.buffer[..src.len()].copy_from_slice(src);
        Ok(())
    }

    fn program(&mut self, address: u32, length: usize) -> core::result::Result<(), DFUMemError> {
        stop_bootloader_timeout();
        if address < FLASH_START {
            return Err(DFUMemError::Address);
        }

        let offset = address - FLASH_START;

        if offset < BOOTLOADER_SIZE_BYTES {
            return Err(DFUMemError::Address);
        }

        if offset as usize >= (FLASH_SIZE_BYTES as usize - length) {
            return Err(DFUMemError::Address);
        }

        match self.unlocked.program(offset as usize, self.buffer[..length].iter()) {
            Ok(_) => Ok(()),
            Err(_) => Err(DFUMemError::Unknown),
        }
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        cortex_m::interrupt::disable();
        cortex_m::peripheral::SCB::sys_reset();
    }
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

const fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u32) {
    let ticks = clock / freq;
    let psc = (ticks - 1) / (1 << 16);
    let arr = ticks / (psc + 1) - 1;
    (psc as u16, arr)
}

/// Initialize, configure all peripherals, and setup USB DFU.
/// Interrupts must be disabled.
fn dfu_init(mode: BootMode) {
    let device = pac::Peripherals::take().unwrap();

    let flash = unsafe {
        FLASH.as_mut_ptr().write(LockedFlash::new(device.FLASH));
        &mut *FLASH.as_mut_ptr()
    };

    let rcc = device.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(56.MHz())
        .require_pll48clk()
        .freeze();

    /* During Bootup go into DFU mode for 4 seconds for recovery purposes */
    // This is implemented manually because the timer code needs > 4K memory
    if mode == BootMode::UNKNOWN {
        // enable and reset timer module
        unsafe {
            let rcc = &(*RCC::ptr());
            TIM2::enable(&rcc);
            TIM2::reset(&rcc);
        }

        // disable counter
        device.TIM2.cr1.modify(|_, w| w.cen().clear_bit());

        // reset counter
        device.TIM2.cnt.reset();

        // use clock to calculate psc and arr
        let tim2clk = TIM2::timer_clock(&clocks);
        let (psc, arr) = compute_arr_presc(1, tim2clk.raw()); // 1 second
        let psc = psc * 4; // 4 seconds

        // Set prescale to psc
        device.TIM2.psc.write(|w| w.psc().bits(psc) );

        // Set auto reload to arr
        device.TIM2.arr.write(|w| w.bits(arr));

        // Trigger update event to load the registers
        device.TIM2.cr1.modify(|_, w| w.urs().set_bit());
        device.TIM2.egr.write(|w| w.ug().set_bit());
        device.TIM2.cr1.modify(|_, w| w.urs().clear_bit());

        // enable counter
        device.TIM2.cr1.modify(|_, w| w.cen().set_bit());

        // listen
        device.TIM2.dier.modify(|r, w| unsafe { w.bits(r.bits() | Event::Update.bits()) });
    }

    let gpioa = device.GPIOA.split();

    let usb = USB {
        usb_global: device.OTG_FS_GLOBAL,
        usb_device: device.OTG_FS_DEVICE,
        usb_pwrclk: device.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let bus = unsafe {
        USB_BUS.as_mut_ptr().write(UsbBus::new(usb, &mut EP_MEMORY ));
        &*USB_BUS.as_ptr()
    };

    /* DFU */
    let stm32mem = STM32Mem::new(flash);

    unsafe {
        USB_DFU.as_mut_ptr().write(DFUClass::new(&bus, stm32mem));
    }

    /* USB device */
    let usb_dev = UsbDeviceBuilder::new(&bus, UsbVidPid(0x0483, 0xdf11))
        .manufacturer("sre")
        .product("PnP Controller (Bootloader)")
        .serial_number(get_serial_str())
        .device_release(0x0200)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

    unsafe {
        USB_DEVICE.as_mut_ptr().write(usb_dev);
    }

    unsafe {
        pac::NVIC::unmask(stm32f4xx_hal::pac::Interrupt::OTG_FS);
        pac::NVIC::unmask(stm32f4xx_hal::pac::Interrupt::TIM2);
    }
}

fn minimal_init() {
    unsafe {
        // enable PWR
        (*RCC::ptr()).apb1enr.modify(|_, w| w.pwren().set_bit());
    }

    cortex_m::asm::delay(100);
}

/// Reset registers that were used for a
/// check if DFU mode must be enabled to a
/// default values before starting main firmware.
fn quick_uninit() {
    unsafe {
        (*RCC::ptr()).apb1enr.reset();
    }
}

/// Initialize stack pointer and jump to a main firmware.
#[inline(never)]
fn jump_to_app() -> ! {
    let vt = FW_ADDRESS as *const u32;
    unsafe {
        cortex_m::asm::bootload(vt);
    }
}

/// Check if FW looks OK and jump to it, or return.
fn try_start_app() {
    let sp = unsafe { (FW_ADDRESS as *const u32).read() };
    if sp & 0xfffe_0000 == 0x2000_0000 {
        quick_uninit();
        jump_to_app();
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

/// Read magic value to determine if device must enter DFU mode
fn bootcode_get_val() -> u32 {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.read_volatile() }
}

/// Write magic value in RAM so that DFU would be triggered only once
fn bootcode_set_val(val: u32) {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.write_volatile(val) };
}

/// Return bootmode configured in uninit area of the RAM and update
/// it, so that next boot starts into PROGRAM
fn bootmode() -> BootMode {
    let bootcode = bootcode_get_val();
    bootcode_set_val(KEY_GOTO_PROGRAM);

    match bootcode {
        KEY_STAY_IN_BOOT => BootMode::BOOTLOADER,
        KEY_GOTO_PROGRAM => BootMode::PROGRAM,
        _ => BootMode::UNKNOWN,
    }
}

#[entry]
fn main() -> ! {
    let mode = bootmode();

    if mode == BootMode::PROGRAM {
        minimal_init();
        try_start_app();
    }

    cortex_m::interrupt::disable();

    dfu_init(mode);

    cortex_m::asm::dsb();
    unsafe { cortex_m::interrupt::enable() };

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn OTG_FS() {
    let usb_dev = unsafe { &mut *USB_DEVICE.as_mut_ptr() };
    let dfu = unsafe { &mut *USB_DFU.as_mut_ptr() };

    usb_dev.poll(&mut [dfu]);
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::disable();
    cortex_m::peripheral::SCB::sys_reset();
}
