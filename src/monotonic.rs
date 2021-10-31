use core::u32;
use core::{cmp::Ordering, convert::Infallible, fmt, ops};
use rtic::Monotonic;
use stm32f0xx_hal::pac;

pub struct Tim6Monotonic;

const CORE_CLOCK: u32 = 8_000_000;
const PRESCALER: u32 = 2048;
const HZ: u32 = CORE_CLOCK / PRESCALER;

impl Tim6Monotonic {
    /// Initialize the timer instance.
    pub fn initialize(timer: pac::TIM6) {
        // Enable and reset TIM6 in RCC
        //
        // Correctness: Since we only modify TIM6 related registers in the RCC
        // register block, and since we own pac::TIM6, we should be safe.
        unsafe {
            let rcc = &*pac::RCC::ptr();

            // Enable timer
            rcc.apb1enr.modify(|_, w| w.tim6en().set_bit());

            // Reset timer
            rcc.apb1rstr.modify(|_, w| w.tim6rst().set_bit());
            rcc.apb1rstr.modify(|_, w| w.tim6rst().clear_bit());
        }

        // Set up prescaler
        timer.psc.write(|w| w.psc().bits(PRESCALER as u16));

        // Enable counter
        timer.cr1.modify(|_, w| w.cen().set_bit());

        // Explicitly drop timer instance so it cannot be reused or reconfigured
        drop(timer);
    }
}

impl Monotonic for Tim6Monotonic {
    type Instant = Instant;

    fn ratio() -> rtic::Fraction {
        // monotonic * fraction = sys clock
        rtic::Fraction {
            numerator: PRESCALER,
            denominator: 1,
        }
    }

    /// Returns the current time
    ///
    /// # Correctness
    ///
    /// This function is *allowed* to return nonsensical values if called before `reset` is invoked
    /// by the runtime. Therefore application authors should *not* call this function during the
    /// `#[init]` phase.
    fn now() -> Self::Instant {
        Instant::now()
    }

    /// Resets the counter to *zero*
    ///
    /// # Safety
    ///
    /// This function will be called *exactly once* by the RTFM runtime after `#[init]` returns and
    /// before tasks can start; this is also the case in multi-core applications. User code must
    /// *never* call this function.
    unsafe fn reset() {
        let tim = &*pac::TIM6::ptr();

        // Pause
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        // Reset counter
        tim.cnt.reset();
        // Continue
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn zero() -> Self::Instant {
        Instant { inner: 0 }
    }
}

/// A measurement of the counter. Opaque and useful only with `Duration`.
///
/// # Correctness
///
/// Adding or subtracting a `Duration` of more than `(1 << 15)` cycles to an `Instant` effectively
/// makes it "wrap around" and creates an incorrect value. This is also true if the operation is
/// done in steps, e.g. `(instant + dur) + dur` where `dur` is `(1 << 14)` ticks.
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct Instant {
    inner: i16,
}

impl Instant {
    /// Returns an instant corresponding to "now".
    pub fn now() -> Self {
        let now = {
            let tim = unsafe { &*pac::TIM6::ptr() };
            tim.cnt.read().cnt().bits()
        };

        Instant { inner: now as i16 }
    }

    /// Returns the amount of time elapsed since this instant was created.
    pub fn elapsed(&self) -> Duration {
        Instant::now() - *self
    }

    /// Returns the underlying count
    pub fn counts(&self) -> u16 {
        self.inner as u16
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(&self, earlier: Instant) -> Duration {
        let diff = self.inner.wrapping_sub(earlier.inner);
        assert!(diff >= 0, "second instant is later than self");
        Duration { inner: diff as u16 }
    }

    pub fn delay_ms(ms: u16) {
        let instant: Instant = Instant::now();
        loop {
            if instant.elapsed() > ms.millis() {
                break;
            }
        }
    }
}

impl fmt::Debug for Instant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Instant")
            .field(&(self.inner as u16))
            .finish()
    }
}

impl ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, dur: Duration) {
        // NOTE this is a debug assertion because there's no foolproof way to detect a wrap around;
        // the user may write `(instant + dur) + dur` where `dur` is `(1<<15)-1` ticks.
        debug_assert!(dur.inner < (1 << 15));
        self.inner = self.inner.wrapping_add(dur.inner as i16);
    }
}

impl ops::Add<Duration> for Instant {
    type Output = Self;
    fn add(mut self, dur: Duration) -> Self {
        self += dur;
        self
    }
}

impl ops::SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, dur: Duration) {
        // NOTE see the NOTE in `<Instant as AddAssign<Duration>>::add_assign`
        debug_assert!(dur.inner < (1 << 15));
        self.inner = self.inner.wrapping_sub(dur.inner as i16);
    }
}

impl ops::Sub<Duration> for Instant {
    type Output = Self;
    fn sub(mut self, dur: Duration) -> Self {
        self -= dur;
        self
    }
}

impl ops::Sub for Instant {
    type Output = Duration;
    fn sub(self, other: Instant) -> Duration {
        self.duration_since(other)
    }
}

impl Ord for Instant {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.inner.wrapping_sub(rhs.inner).cmp(&0)
    }
}

impl PartialOrd for Instant {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

/// A `Duration` type to represent a span of time.
#[derive(Clone, Copy, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Duration {
    inner: u16,
}

impl Duration {
    /// Creates a new `Duration` from the specified number of timer ticks
    pub fn from_ticks(ticks: u16) -> Self {
        Duration { inner: ticks }
    }

    /// Returns the total number of timer ticks contained by this `Duration`
    pub fn as_ticks(&self) -> u16 {
        self.inner
    }
}

// Used internally by RTIC to convert the duration into a known type
impl TryInto<u32> for Duration {
    type Error = Infallible;

    fn try_into(self) -> Result<u32, Infallible> {
        Ok(self.as_ticks() as u32)
    }
}

impl ops::AddAssign for Duration {
    fn add_assign(&mut self, dur: Duration) {
        self.inner += dur.inner;
    }
}

impl ops::Add for Duration {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Duration {
            inner: self.inner + other.inner,
        }
    }
}

impl ops::Mul<u16> for Duration {
    type Output = Self;
    fn mul(self, other: u16) -> Self {
        Duration {
            inner: self.inner * other,
        }
    }
}

impl ops::MulAssign<u16> for Duration {
    fn mul_assign(&mut self, other: u16) {
        *self = *self * other;
    }
}

impl ops::SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Duration) {
        self.inner -= rhs.inner;
    }
}

impl ops::Sub for Duration {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Duration {
            inner: self.inner - rhs.inner,
        }
    }
}

/// Adds the `secs`, `millis` and `micros` methods to the `u16` type.
///
/// WARNING: You cannot represent values higher than 8388 milliseconds without
/// overflow!
pub trait U16Ext {
    /// Converts the `u16` value as seconds into ticks
    fn secs(self) -> Duration;

    /// Converts the `u16` value as milliseconds into ticks
    fn millis(self) -> Duration;

    /// Converts the `u16` value as microseconds into ticks
    fn micros(self) -> Duration;
}

impl U16Ext for u16 {
    fn secs(self) -> Duration {
        debug_assert!(self <= 8, "Cannot represent values >8s in a `Duration`");
        Duration {
            inner: (HZ as u64 * self as u64) as u16,
        }
    }

    fn millis(self) -> Duration {
        debug_assert!(
            self <= 8388,
            "Cannot represent values >8388 in a `Duration`"
        );
        Duration {
            inner: (HZ as u64 * self as u64 / 1_000) as u16,
        }
    }

    fn micros(self) -> Duration {
        Duration {
            inner: (HZ as u64 * self as u64 / 1_000_000) as u16,
        }
    }
}
