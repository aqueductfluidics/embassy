//! Timer driver.
use core::borrow::BorrowMut;
use core::cell::Cell;
use core::sync::atomic::{compiler_fence, AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::driver::{AlarmHandle, Driver};

use crate::{interrupt, pac, hal};

#[allow(non_snake_case)]
#[cfg(all(feature = "rt", feature = "time-driver-gpt1"))]
#[interrupt]
fn GPT1() {
    DRIVER.on_interrupt()
}

#[allow(non_snake_case)]
#[cfg(all(feature = "rt", feature = "time-driver-gpt2"))]
#[interrupt]
fn GPT2() {
    DRIVER.on_interrupt()
}

struct AlarmState {
    timestamp: Cell<u64>,

    // This is really a Option<(fn(*mut ()), *mut ())>
    // but fn pointers aren't allowed in const yet
    callback: Cell<*const ()>,
    ctx: Cell<*mut ()>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            callback: Cell::new(ptr::null()),
            ctx: Cell::new(ptr::null_mut()),
        }
    }
}

const ALARM_COUNT: usize = 2;

#[cfg(feature = "time-driver-gpt1")]
type T = pac::gpt::GPT1;
#[cfg(feature = "time-driver-gpt1")]
const N: u8 = 1;

#[cfg(feature = "time-driver-gpt2")]
type T = pac::gpt::GPT2;
#[cfg(feature = "time-driver-gpt2")]
const N: u8 = 2;


pub(crate) struct RtcDriver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    alarm_count: AtomicU8,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
    gpt: Mutex<CriticalSectionRawMutex, Cell<Option<hal::gpt::Gpt<N>>>>,
}

// Clock timekeeping works with something we call "periods", which are time intervals
// of 2^31 ticks. The Clock counter value is 32 bits, so one "overflow cycle" is 2 periods.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at counter value 0x8000_0000)
//
// Therefore, when `period` is even, counter is in 0..0x7FFF_FFFF. When odd, counter is in 0x8000_0000..0xFFFF_FFFF
// This allows for now() to return the correct value even if it races an overflow.
//
// To get `now()`, `period` is read first, then `counter` is read. If the counter value matches
// the expected range for the `period` parity, we're done. If it doesn't, this means that
// a new period start has raced us between reading `period` and `counter`, so we assume the `counter` value
// corresponds to the next period.
//
// `period` is a 32bit integer, so It overflows on 2^32 * 2^15 / 32768 seconds of uptime, which is 136 years.
fn calc_now(period: u32, counter: u32) -> u64 {
    ((period as u64) << 31) + ((counter as u32 ^ ((period & 1) << 31)) as u64)
}

const ALARM_STATE_NEW: AlarmState = AlarmState::new();

embassy_time::time_driver_impl!(static DRIVER: RtcDriver = RtcDriver {
    period: AtomicU32::new(0),
    alarm_count: AtomicU8::new(0),
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [ALARM_STATE_NEW; ALARM_COUNT]),
    gpt: Mutex::const_new(CriticalSectionRawMutex::new(), Cell::new(None)),
});

impl RtcDriver {
    fn init(&'static self, cs: critical_section::CriticalSection) {
        critical_section::with(|cs| {
            let r = unsafe { T::instance() };

            let mut timer = hal::gpt::Gpt::<N>::new(r);
            timer.disable();

            timer.set_clock_source(hal::gpt::ClockSource::PeripheralClock);
            timer.set_divider(0);
            timer.set_output_compare_count(hal::gpt::OutputCompareRegister::OCR3, 0x8000_0000);
            timer.set_mode(hal::gpt::Mode::FreeRunning);
            timer.set_reset_on_enable(true);
            timer.enable();

            timer.set_output_interrupt_on_compare(hal::gpt::OutputCompareRegister::OCR3, true);
            timer.set_rollover_interrupt_enable(true);

            let pinned = self.gpt.borrow(cs);
            pinned.set(Some(timer));
        });
    }

    fn on_interrupt(&self) {
        critical_section::with(|cs| {
            let r = unsafe { T::instance() };
            let gpt = self.gpt.borrow(cs).get_mut().unwrap();

            if gpt.is_rollover() {
                gpt.clear_rollover();
            }

            // // Overflow
            // if pac::read_reg!(pac::gpt, r, SR, ROV == 1) {
            //     pac::modify_reg!(pac::gpt, r, SR, ROV: 1);
            //     self.next_period();
            // }

            // // Half overflow
            // if pac::read_reg!(pac::gpt, r, SR, OF3 == 1) {
            //     pac::modify_reg!(pac::gpt, r, SR, OF3: 1);
            //     self.next_period();
            // }

            // if pac::read_reg!(pac::gpt, r, IR, OF1IE == 1) && pac::read_reg!(pac::gpt, r, SR, OF1 == 1) {
            //     pac::modify_reg!(pac::gpt, r, SR, OF1: 1);
            //     self.trigger_alarm(0, cs);
            // }

            // if pac::read_reg!(pac::gpt, r, IR, OF2IE == 1) && pac::read_reg!(pac::gpt, r, SR, OF2 == 1) {
            //     pac::modify_reg!(pac::gpt, r, SR, OF2: 1);
            //     self.trigger_alarm(1, cs);
            // }
        })
    }

    fn next_period(&self) {
        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 31;
    }

    fn get_alarm<'a>(&'a self, cs: CriticalSection<'a>, alarm: AlarmHandle) -> &'a AlarmState {
        // safety: we're allowed to assume the AlarmState is created by us, and
        // we never create one that's out of bounds.
        unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) }
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.

        // safety:
        // - we can ignore the possibility of `f` being unset (null) because of the safety contract of `allocate_alarm`.
        // - other than that we only store valid function pointers into alarm.callback
        let f: fn(*mut ()) = unsafe { mem::transmute(alarm.callback.get()) };
        f(alarm.ctx.get());
    }
}

impl Driver for RtcDriver {
    fn now(&self) -> u64 {
        let r = unsafe { T::instance() };

        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = pac::read_reg!(pac::gpt, r, CNT);
        calc_now(period, counter)
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        critical_section::with(|_| {
            let id = self.alarm_count.load(Ordering::Relaxed);
            if id < ALARM_COUNT as u8 {
                self.alarm_count.store(id + 1, Ordering::Relaxed);
                Some(AlarmHandle::new(id))
            } else {
                None
            }
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);

            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let r = unsafe { T::instance() };

            let n = alarm.id() as usize;
            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);

            let t = self.now();
            if timestamp <= t {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.
                match n {
                    0 => {
                        pac::modify_reg!(pac::gpt, r, IR, OF1IE: 0);
                    }
                    1 => {
                        pac::modify_reg!(pac::gpt, r, IR, OF2IE: 0);
                    }
                    _ => {
                        // noop
                    }
                }

                alarm.timestamp.set(u64::MAX);

                return false;
            }

            // Write the CCR value regardless of whether we're going to enable it now or not.
            // This way, when we enable it later, the right value is already set.
            match n {
                0 => {
                    pac::write_reg!(pac::gpt, r, OCR[0], timestamp as u32);
                }
                1 => {
                    pac::write_reg!(pac::gpt, r, OCR[1], timestamp as u32);
                }
                _ => {
                    // noop
                }
            }
            

            // Enable it.
            match n {
                0 => {
                    pac::modify_reg!(pac::gpt, r, IR, OF1IE: true as u32);
                }
                1 => {
                    pac::modify_reg!(pac::gpt, r, IR, OF2IE: true as u32);
                }
                _ => {
                    // noop
                }
            }

            // Reevaluate if the alarm timestamp is still in the future
            let t = self.now();
            if timestamp <= t {
                // If alarm timestamp has passed since we set it, we have a race condition and
                // the alarm may or may not have fired.
                // Disarm the alarm and return `false` to indicate that.
                // It is the caller's responsibility to handle this ambiguity.
                match n {
                    0 => {
                        pac::modify_reg!(pac::gpt, r, IR, OF1IE: false as u32);
                    }
                    1 => {
                        pac::modify_reg!(pac::gpt, r, IR, OF2IE: false as u32);
                    }
                    _ => {
                        // noop
                    }
                }

                alarm.timestamp.set(u64::MAX);

                return false;
            }

            // We're confident the alarm will ring in the future.
            true
        })
    }
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs)
}
