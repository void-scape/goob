#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![feature(sync_unsafe_cell)]
#![forbid(unsafe_op_in_unsafe_fn)]

use arduino_hal::{
    hal::port::{PB0, PB3, PB4, PB5, PD3, PD4, PD6},
    port::{
        Pin,
        mode::{Floating, Input, Output},
    },
};
use avr_device::interrupt;
use core::{cell::RefCell, ptr::addr_of};
use embedded_hal::digital::{OutputPin, PinState};

// extern crate alloc;

/// Prevents compiling `unwrap` messages.
#[macro_export]
macro_rules! unwrap {
    ($v:expr) => {
        match $v {
            Some(v) => v,
            None => panic!(),
        }
    };
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // disable interrupts - firmware has panicked so no ISRs should continue running
    avr_device::interrupt::disable();

    // crate::println!("PANIC");
    // unsafe { (&mut *HEAP.get()).fill(0) };
    // if let Some(loc) = info.location() {
    //     crate::println!("^-{}:{}:{}", loc.file(), loc.line(), loc.column());
    // }

    // SAFETY: Because main() already has references to the peripherals this is an unsafe
    // operation - but because no other code can run after the panic handler was called,
    // this is okay.
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    let pins = arduino_hal::pins!(dp);
    let mut led = pins.d13.into_output();
    loop {
        led.toggle();
        arduino_hal::delay_ms(100);
    }
}

// #[global_allocator]
// static ALLOC: Allocator = Allocator;
// static HEAP: core::cell::SyncUnsafeCell<[u8; 512]> = core::cell::SyncUnsafeCell::new([0; 512]);
// #[allow(unused)]
// fn debug_heap() {
//     let mut ptr = HEAP.get() as *mut u8;
//     let heap_end = ptr as usize + 512;
//     let mut count = 0;
//     let mut entries = 0;
//     loop {
//         unsafe {
//             if ptr as usize + 2 > heap_end {
//                 break;
//             }
//
//             let header = u16::from_le_bytes([*ptr, *(ptr.add(1))]);
//             let bytes = header & (u16::MAX >> 1);
//             let is_full = header >> 15 != 0;
//             if bytes == 0 {
//                 break;
//             }
//             if is_full {
//                 count += 1;
//             }
//             ptr = ptr.add(bytes as usize + 2);
//             entries += 1;
//         }
//     }
//     println!("Ocuppied/Entries: {}/{}", count, entries);
//     let ptr = HEAP.get() as *mut u8;
//     for page in 0..8 {
//         let offset = page * 16;
//         print!("{:#04X}: ", offset);
//         for i in 0..16 {
//             unsafe {
//                 print!("{:02X} ", *ptr.add(offset + i));
//             }
//         }
//         println!();
//     }
// }
// struct Allocator;
// unsafe impl core::alloc::GlobalAlloc for Allocator {
//     unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
//         let mut ptr = HEAP.get() as *mut u8;
//         let heap_end = ptr as usize + 512;
//
//         loop {
//             let header = unsafe { u16::from_le_bytes([*ptr, *(ptr.add(1))]) };
//             let bytes = header & (u16::MAX >> 1);
//             let is_full = header >> 15 != 0;
//             let align = layout.align();
//
//             // Reserve 1 byte to store padding
//             let aligned_addr = (ptr as usize + 3 + align - 1) & !(align - 1);
//             let padding = aligned_addr - (ptr as usize + 2);
//             let total_needed = padding + layout.size();
//
//             let can_fit = !is_full && total_needed <= bytes as usize;
//
//             if bytes == 0 {
//                 if ptr as usize + 2 + total_needed > heap_end {
//                     return core::ptr::null_mut();
//                 }
//                 unsafe {
//                     let le_bytes = ((total_needed as u16) | (1 << 15)).to_le_bytes();
//                     *ptr = le_bytes[0];
//                     *(ptr.add(1)) = le_bytes[1];
//                     // pad
//                     *(aligned_addr as *mut u8).sub(1) = padding as u8;
//                     return aligned_addr as *mut u8;
//                 }
//             }
//
//             if can_fit {
//                 let le_bytes = (header | (1 << 15)).to_le_bytes();
//                 unsafe {
//                     *ptr = le_bytes[0];
//                     *(ptr.add(1)) = le_bytes[1];
//                     // pad
//                     *(aligned_addr as *mut u8).sub(1) = padding as u8;
//                     return aligned_addr as *mut u8;
//                 }
//             }
//
//             unsafe {
//                 ptr = ptr.add(bytes as usize + 2);
//             }
//         }
//     }
//     unsafe fn dealloc(&self, ptr: *mut u8, _: core::alloc::Layout) {
//         unsafe {
//             let padding = *ptr.sub(1) as usize;
//             let header_ptr = ptr.sub(padding + 2);
//             let header = u16::from_le_bytes([*header_ptr, *(header_ptr.add(1))]);
//             let le_bytes = (header & (u16::MAX >> 1)).to_le_bytes();
//             *header_ptr = le_bytes[0];
//             *(header_ptr.add(1)) = le_bytes[1];
//         }
//     }
// }

type Serial = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
static SERIAL: interrupt::Mutex<RefCell<Option<Serial>>> =
    interrupt::Mutex::new(RefCell::new(None));
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        __log(format_args!($($arg)*));
    };
}
#[macro_export]
macro_rules! println {
    () => {
        __log(format_args!("\n"));
    };
    ($fmt:literal$($arg:tt)*) => {
        __log(format_args!(concat!($fmt, "\n")$($arg)*));
    };
}
fn set_global_serial(serial: Serial) {
    avr_device::interrupt::free(|cs| {
        *SERIAL.borrow(cs).borrow_mut() = Some(serial);
    });
}
fn __log(args: core::fmt::Arguments) {
    avr_device::interrupt::free(|cs| {
        let mut serial_ref = SERIAL.borrow(cs).borrow_mut();
        if let Some(port) = serial_ref.as_mut() {
            if let Some(str) = args.as_str() {
                for byte in str.bytes() {
                    port.write_byte(byte);
                }
            } else {
                // use alloc::string::ToString;
                // let str = args.to_string();
                // for byte in str.bytes() {
                //     port.write_byte(byte);
                // }
            }
        }
    });
}

type Txd = Pin<Input<Floating>, PB3>;
type Rxd = Pin<Output, PB4>;
type En = Pin<Output, PB0>;

#[arduino_hal::entry]
fn main() -> ! {
    // display();
    sokobon();
    // hc05();
}

fn sokobon() -> ! {
    let dp = unwrap!(arduino_hal::Peripherals::take());
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
    // set addressing
    cmd(&mut dc, &mut sck, &mut sda, 0x20);
    // horizontal addressing
    cmd(&mut dc, &mut sck, &mut sda, 0x00);
    led.set_low();

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

    #[rustfmt::skip]
    let mut map: [u8; 128] = [
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 3, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 5, 1, 1, 1, 1, 1, 1, 1,
    ];

    let floor = [0; 8];
    #[rustfmt::skip]
    let wall = [
        0b11111111,
        0b10000001,
        0b10000001,
        0b10000001,
        0b10000001,
        0b10000001,
        0b10000001,
        0b11111111,
    ];
    #[rustfmt::skip]
    let player = [
        0b00111100,
        0b01000010,
        0b10000001,
        0b10110001,
        0b10000101,
        0b10110001,
        0b01000010,
        0b00111100,
    ];
    #[rustfmt::skip]
    let rock = [
        0b00011110,
        0b01100001,
        0b10000001,
        0b10000001,
        0b10000001,
        0b10000001,
        0b01100001,
        0b00011110,
    ];
    #[rustfmt::skip]
    let hole = [
        0b00000010,
        0b00000111,
        0b00000111,
        0b00000111,
        0b00000111,
        0b00000111,
        0b00000111,
        0b00000010,
    ];
    #[rustfmt::skip]
    let door = [
        0b11111111,
        0b10000001,
        0b10000001,
        0b10000001,
        0b10001101,
        0b10001101,
        0b10000001,
        0b11111111,
    ];
    let sprites = [&floor, &wall, &player, &rock, &hole, &door];

    let map_width = 16;
    let map_height = 8;
    let mut frame_buffer = [0u8; WIDTH * HEIGHT / 8];
    loop {
        frame_buffer.fill(0);
        for y in 0..map_height {
            let py = y * 8;
            for x in 0..map_width {
                let px = x * 8;
                let sprite = map[y * map_width + x] as usize;
                render_sprite(&mut frame_buffer, 8, 1, sprites[sprite], px, py);
            }
        }
        render(&frame_buffer, &mut dc, &mut sck, &mut sda, &mut led);
        arduino_hal::delay_ms(1000);
    }
}

#[allow(unused)]
fn hc05() -> ! {
    // let dp = unwrap!(arduino_hal::Peripherals::take());
    // let pins = arduino_hal::pins!(dp);
    // // let serial = arduino_hal::default_serial!(dp, pins, 115200);
    // // set_global_serial(serial);
    //
    // // let mut vcc = pins.d9.into_output();
    // // let mut gnd = pins.d10.into_output();
    // let mut txd = pins.d11.into_floating_input();
    // let mut rxd = pins.d12.into_output();
    // let mut en = pins.d8.into_output();
    // let mut led = pins.d13.into_output();
    //
    // // boot into command mode
    // en.set_low();
    // // gnd.set_low();
    // // vcc.set_high();
    // // wait to enter command mode
    // arduino_hal::delay_ms(8000);
    //
    // // let baud = 38400;
    // // let mut serial = arduino_hal::default_serial!(dp, pins, baud);
    // // for byte in "AT\r\n".bytes() {
    // //     serial.write_byte(byte);
    // // }
    // // let mut str = alloc::string::String::new();
    // // loop {
    // //     if str.ends_with("OK\r\n") {
    // //         break;
    // //     }
    // // str.push(serial.read_byte() as char);
    // // }
    // // at_cmd(&mut rxd, baud, "AT\r\n");
    // // let _resp = resp(&mut txd, baud);
    // // println!("{}", resp);
    // // led.set_high();
    //
    // fn at_cmd(rxd: &mut Rxd, baud: usize, msg: &str) {
    //     println!("Writing: '{}'", msg);
    //     let delay_us = (1.0 / baud as f32 * 1000.0 * 1000.0) as u32;
    //     rxd.set_high();
    //     for byte in msg.bytes() {
    //         avr_device::interrupt::free(|_| {
    //             // start bit
    //             rxd.set_low();
    //             arduino_hal::delay_us(delay_us);
    //             for bit in 0..8 {
    //                 rxd.set_state(PinState::from(((byte >> bit) & 1) != 0));
    //                 arduino_hal::delay_us(delay_us);
    //             }
    //             // end bit
    //             rxd.set_high();
    //             arduino_hal::delay_us(delay_us);
    //         });
    //     }
    //     println!("finished write");
    // }
    //
    // fn resp(txd: &mut Txd, baud: usize) -> alloc::string::String {
    //     println!("Awaiting response");
    //     let mut str = alloc::string::String::default();
    //     let delay_us = (1.0 / baud as f32 * 1000.0 * 1000.0) as u32;
    //     loop {
    //         if str.ends_with("OK\r\n") {
    //             break;
    //         }
    //         while txd.is_high() {}
    //         // delay to sample in bit center
    //         arduino_hal::delay_us(delay_us / 2);
    //         assert!(txd.is_low());
    //         // delay to first bit
    //         arduino_hal::delay_us(delay_us);
    //         let mut byte: u8 = 0;
    //         for bit in 0..8 {
    //             if txd.is_high() {
    //                 byte |= 1 << bit;
    //             }
    //             arduino_hal::delay_us(delay_us);
    //         }
    //         str.push(byte as char);
    //         println!("  push: '{}'", byte as char);
    //     }
    //     str
    // }

    loop {
        arduino_hal::delay_ms(1000);
    }
}

type Sck = Pin<Output, PD3>;
type Sda = Pin<Output, PD4>;
type Dc = Pin<Output, PD6>;
type Led = Pin<Output, PB5>;

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

#[allow(unused)]
fn display() -> ! {
    let dp = unwrap!(arduino_hal::Peripherals::take());
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
                render_sprite_flash(&mut frame_buffer, 6, 1, addr as _, 20, 20);
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

fn render_sprite(
    frame_buffer: &mut [u8],
    width: usize,
    height: usize,
    bytes: &[u8],
    x: usize,
    y: usize,
) {
    for sy in 0..height {
        for sx in 0..width {
            let page_col = bytes[sy * width + sx];
            for col_bit in 0..8 {
                let pixel = ((page_col >> col_bit) & 1) != 0;
                set_pixel(frame_buffer, sx + x, sy + y + 7 - col_bit, pixel);
            }
        }
    }
}

/// ## Safety
///
/// `bytes` points to an array of packed pixel data in flash memory.
unsafe fn render_sprite_flash(
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
