#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![forbid(unsafe_op_in_unsafe_fn)]

use arduino_hal::{
    hal::port::{PB5, PD3, PD4, PD6},
    port::{Pin, mode::Output},
};
use core::ptr::addr_of;
use embedded_hal::digital::{OutputPin, PinState};

const WIDTH: usize = 128;
const HEIGHT: usize = 64;

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F1: [u8; 6] = [
    0b01000000,
    0b00100000,
    0b00010000,
    0b00010000,
    0b00100000,
    0b01000000,
];

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F2: [u8; 6] = [
    0b00100000,
    0b00010000,
    0b00010000,
    0b00010000,
    0b00010000,
    0b00100000,
];

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F3: [u8; 6] = [
    0b00001000,
    0b00001000,
    0b00010000,
    0b00010000,
    0b00001000,
    0b00001000,
];

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F4: [u8; 6] = [
    0b00001000,
    0b00010000,
    0b00100000,
    0b00100000,
    0b00010000,
    0b00001000,
];

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F5: [u8; 6] = [
    0b00010000,
    0b00100000,
    0b00100000,
    0b00100000,
    0b00100000,
    0b00010000,
];

#[unsafe(link_section = ".progmem.data")]
#[rustfmt::skip]
static ARROW_F6: [u8; 6] = [
    0b00100000,
    0b00100000,
    0b00010000,
    0b00010000,
    0b00100000,
    0b00100000,
];

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

type Sck = Pin<Output, PD3>;
type Sda = Pin<Output, PD4>;
type Dc = Pin<Output, PD6>;
type Led = Pin<Output, PB5>;

#[arduino_hal::entry]
fn main() -> ! {
    // display();
    // loop {}

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
    let mut serial = move |byte: u8| serial.write_byte(byte);

    let mut en = pins.d8.into_output();
    let mut vcc = pins.d9.into_output();
    let mut gnd = pins.d10.into_output();
    let mut txd = pins.d11.into_pull_up_input();
    let mut rxd = pins.d12.into_output();
    let mut state = pins.d13.into_pull_up_input();

    gnd.set_low();
    vcc.set_high();

    loop {
        log(&mut serial, "Hello, World!");
        log(&mut serial, "My name is Nic!");
        arduino_hal::delay_ms(1000);
    }
}

fn log(serial: &mut impl FnMut(u8), msg: &str) {
    for byte in msg.bytes() {
        serial(byte);
    }
    serial(b'\n');
}

#[allow(unused)]
fn display() {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut led = pins.d13.into_output();
    let mut gnd = pins.d1.into_output();
    let mut vcc = pins.d2.into_output();
    let mut sck = pins.d3.into_output();
    let mut sda = pins.d4.into_output();
    let mut res = pins.d5.into_output();
    let mut dc = pins.d6.into_output();
    let mut cs = pins.d7.into_output();

    // Init
    gnd.set_low();
    vcc.set_high();
    res.set_low();
    arduino_hal::delay_ns(50);
    res.set_high();
    cs.set_low();

    // Display on
    cmd(&mut dc, &mut sck, &mut sda, 0xA4);
    cmd(&mut dc, &mut sck, &mut sda, 0xAF);
    let set_addressing = 0x20;
    let horizontal_addressing = 0x00;
    cmd(&mut dc, &mut sck, &mut sda, set_addressing);
    cmd(&mut dc, &mut sck, &mut sda, horizontal_addressing);
    led.set_low();

    let mut frame_buffer = [0u8; WIDTH * HEIGHT / 8];

    fn cmd(dc: &mut Dc, sck: &mut Sck, sda: &mut Sda, cmd: u8) {
        dc.set_low();
        sck.set_low();
        for bit in (0..8).rev() {
            let is_high = ((cmd >> bit) & 1) != 0;
            sda.set_state(PinState::from(is_high));
            sck.set_high();
            sck.set_low();
        }
    }

    loop {
        for addr in [
            addr_of!(ARROW_F1),
            addr_of!(ARROW_F1),
            addr_of!(ARROW_F1),
            addr_of!(ARROW_F1),
            addr_of!(ARROW_F2),
            addr_of!(ARROW_F3),
            addr_of!(ARROW_F4),
            addr_of!(ARROW_F5),
            addr_of!(ARROW_F6),
        ] {
            frame_buffer.fill(0);
            unsafe {
                render_sprite(&mut frame_buffer, 6, 1, addr as _, 20, 20);
            }
            render(&frame_buffer, &mut dc, &mut sck, &mut sda, &mut led);
            arduino_hal::delay_ms(150);
        }
    }
}

fn set_pixel(frame_buffer: &mut [u8], x: usize, y: usize, pixel: bool) {
    let page = y / 8;
    let col_bit = y % 8;
    let index = page * WIDTH + x;
    let page_col = frame_buffer[index];
    let shifted = (pixel as u8) << col_bit;
    if index < WIDTH * HEIGHT / 8 {
        frame_buffer[index] = (page_col & !shifted) | shifted;
    }
}

fn read_pixel(frame_buffer: &[u8], x: usize, y: usize) -> bool {
    let page = y / 8;
    let col_bit = y % 8;
    let index = page * WIDTH + x;
    if index < WIDTH * HEIGHT / 8 {
        let page_col = frame_buffer[index];
        ((page_col >> col_bit) & 1) != 0
    } else {
        false
    }
}

fn render(frame_buffer: &[u8], dc: &mut Dc, sck: &mut Sck, sda: &mut Sda, led: &mut Led) {
    led.set_high();
    dc.set_high();
    for page in 0..8 {
        for seg in 0..128 {
            let x = 127 - seg;
            for page_offset in 0..8 {
                let y = 56 - page * 8 + page_offset;
                let pixel = read_pixel(&frame_buffer, x, y);
                sda.set_state(PinState::from(pixel));
                sck.set_high();
                sck.set_low();
            }
        }
    }
    led.set_low();
}

/// ## Safety
///
/// `bytes` points to an array of packed pixel data in flash memory.
unsafe fn render_sprite(
    frame_buffer: &mut [u8],
    width: usize,
    height: usize,
    bytes: *const u8,
    x: usize,
    y: usize,
) {
    for sy in 0..height {
        for sx in 0..width {
            let page_col = unsafe { read_byte(bytes.add(sy * width + sx)) };
            for col_bit in 0..8 {
                let pixel = ((page_col >> col_bit) & 1) != 0;
                set_pixel(frame_buffer, sx + x, sy + y + 7 - col_bit, pixel);
            }
        }
    }
}

/// ## Safety
///
/// `p_addr` points to a value of type `T` located in flash memory.
unsafe fn read_value<T>(p_addr: *const T) -> T {
    let mut value = core::mem::MaybeUninit::uninit();

    let mut src = p_addr as *const u8;
    let mut dst = value.as_mut_ptr() as *mut u8;

    for _ in 0..core::mem::size_of::<T>() {
        unsafe {
            *dst = read_byte(src);
            dst = dst.add(1);
            src = src.add(1);
        }
    }

    unsafe { value.assume_init() }
}

/// ## Safety
///
/// `p_addr` points to a location in flash memory.
unsafe fn read_byte(p_addr: *const u8) -> u8 {
    // https://github.com/Cryptjar/avr-progmem-rs

    // Allocate a byte for the output (actually a single register r0
    // will be used).
    let res: u8;

    // The inline assembly to read a single byte from given address
    unsafe {
        core::arch::asm!(
            // Just issue the single `lpm` assembly instruction, which reads
            // implicitly indirectly the address from the Z register, and
            // stores implicitly the read value in the register 0.
            "lpm {}, Z",
            // Output is in a register
            out(reg) res,
            // Input the program memory address to read from
            in("Z") p_addr,
            // No clobber list.
        );
    }

    // Just output the read value
    res
}
