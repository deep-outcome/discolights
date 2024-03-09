#![no_std]
#![no_main]

#[cfg(feature = "panic_halt")]
use panic_halt as _;

use cortex_m::interrupt::{free as interrupt_free, Mutex};
use cortex_m_rt::entry;

use microbit::hal::gpio::{Level, Pin};
use microbit::hal::gpio::{Output, PushPull};

use microbit::hal::prelude::OutputPin;
use microbit::hal::{
    pac::{interrupt, RTC0},
    rtc::{Rtc, RtcInterrupt},
};

use microbit::board::Board;
use microbit::hal::clocks::Clocks;

struct Program<const N: usize> {
    ctl: Pin<Output<PushPull>>,
    schema: [i16; N],
    next: usize,
}

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, Ordering};

static RED_PRG: Mutex<RefCell<Option<Program<10>>>> = Mutex::new(RefCell::new(None));
static YEL_PRG: Mutex<RefCell<Option<Program<8>>>> = Mutex::new(RefCell::new(None));
static GRE_PRG: Mutex<RefCell<Option<Program<13>>>> = Mutex::new(RefCell::new(None));

static RTC: Mutex<RefCell<Option<Rtc<RTC0>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn entry() -> ! {
    let brd = Board::take().unwrap();

    let clocks = Clocks::new(brd.CLOCK);
    clocks.start_lfclk();

    // 32_768 kHz / (32+1) kHz ≈ 993
    // 993 Hz ⇒ circa every each ms
    const PRESCALER: u32 = 32;

    let mut nvic = brd.NVIC;

    let mut rtc = Rtc::new(brd.RTC0, PRESCALER).unwrap();
    rtc.enable_interrupt(RtcInterrupt::Tick, Some(&mut nvic));
    rtc.enable_counter();

    let pins = brd.pins;
    let red_ctl = pins.p0_02.into_push_pull_output(Level::Low).degrade();
    let yel_ctl = pins.p0_03.into_push_pull_output(Level::Low).degrade();
    let gre_ctl = pins.p0_04.into_push_pull_output(Level::Low).degrade();

    let red_mix = [1000, -1200, 800, -300, 100, -100, 200, -200, 140, -134_i16];
    let yel_mix = [800, -300, 330, -370, 550, -880, 123, -555_i16];
    #[rustfmt::skip]
    let gre_mix = [-890, 990, -1111, 876, -345, 875, -432, 777, -321, 444, -1000, 100, -100_i16,];

    let red_prg = Program {
        ctl: red_ctl,
        schema: red_mix,
        next: 0,
    };

    let yel_prg = Program {
        ctl: yel_ctl,
        schema: yel_mix,
        next: 0,
    };

    let gre_prg = Program {
        ctl: gre_ctl,
        schema: gre_mix,
        next: 0,
    };

    interrupt_free(move |cs| {
        _ = RED_PRG.borrow(cs).borrow_mut().replace(red_prg);
        _ = YEL_PRG.borrow(cs).borrow_mut().replace(yel_prg);
        _ = GRE_PRG.borrow(cs).borrow_mut().replace(gre_prg);
        _ = RTC.borrow(cs).borrow_mut().replace(rtc);
    });

    loop {}
}

#[interrupt]
unsafe fn RTC0() {
    static RED_COUNTDOWN: AtomicU16 = AtomicU16::new(1);
    static YEL_COUNTDOWN: AtomicU16 = AtomicU16::new(1);
    static GRE_COUNTDOWN: AtomicU16 = AtomicU16::new(1);

    interrupt_free(|cs| {
        let mut borrow = RTC.borrow(cs).borrow_mut();
        let rtc = borrow.take().unwrap();
        rtc.reset_event(RtcInterrupt::Tick);
        borrow.replace(rtc);
    });

    if RED_COUNTDOWN.fetch_sub(1, Ordering::Relaxed) == 1 {
        adv_prg(&RED_COUNTDOWN, &RED_PRG);
    }

    if YEL_COUNTDOWN.fetch_sub(1, Ordering::Relaxed) == 1 {
        adv_prg(&YEL_COUNTDOWN, &YEL_PRG);
    }

    if GRE_COUNTDOWN.fetch_sub(1, Ordering::Relaxed) == 1 {
        adv_prg(&GRE_COUNTDOWN, &GRE_PRG);
    }
}

fn adv_prg<const N: usize>(countdown: &AtomicU16, prg: &Mutex<RefCell<Option<Program<N>>>>) {
    let interval = interrupt_free(|cs| {
        let mut borrow = prg.borrow(cs).borrow_mut();
        let mut prg = borrow.take().unwrap();

        let interval = adv_prg_nucleus(&prg.schema, &mut prg.next, &mut prg.ctl);
        borrow.replace(prg);
        interval
    });

    countdown.swap(interval, Ordering::Relaxed);
}

fn adv_prg_nucleus(schema: &[i16], next: &mut usize, ctl: &mut Pin<Output<PushPull>>) -> u16 {
    let next_val = *next;
    let mut interval = schema[next_val];

    *next = (next_val + 1) % schema.len();

    if interval < 0 {
        _ = ctl.set_low();
        interval *= -1;
    } else {
        _ = ctl.set_high();
    }

    interval as u16
}

#[cfg(feature = "panic_abort")]
mod panic_abort {
    use core::panic::PanicInfo;

    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }
}

// cargo flash --target thumbv7em-none-eabihf --chip nRF52833_xxAA --release --features panic_abort
// cargo flash --target thumbv7em-none-eabihf --chip nRF52833_xxAA --features panic_halt
// cargo build --release  --target thumbv7em-none-eabihf --features panic_abort
// cargo build --target thumbv7em-none-eabihf --features panic_halt
