#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyPin, Floating, Input, Output, PushPull, IO},
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
    peripherals::Peripherals,
    prelude::*,
    systimer::SystemTimer,
};
use tb6612fng::Motor;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

struct CircularBuffer {
    buffer: [u32; 50], // Fixed size buffer
    index: usize,      // Current index to insert the new value
    count: usize,      // Number of elements currently in the buffer
}

impl CircularBuffer {
    fn new() -> Self {
        CircularBuffer {
            buffer: [0; 50], // Initialize with default value of 0
            index: 0,
            count: 0,
        }
    }

    fn push(&mut self, value: u32) {
        self.buffer[self.index] = value; // Insert the new value
        self.index = (self.index + 1) % 50; // Move to the next index
        if self.count < 50 {
            self.count += 1; // Increase count until full
        }
    }

    fn average(&self) -> u32 {
        let mut sum = 0u32; // Initialize sum as u32
        for &value in &self.buffer[..self.count] {
            sum += value; // Sum all values
        }
        if self.count > 0 {
            sum / self.count as u32 // Calculate average
        } else {
            0 // Return 0 if the buffer is empty
        }
    }

    fn is_stopped(&self, threshold: u32) -> bool {
        let avg = self.average();
        for &value in &self.buffer[..self.count] {
            if (value as i32 - avg as i32).abs() > threshold as i32 {
                return false; // Deviation found
            }
        }
        true // No deviation found
    }
}

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

fn check_distance(
    echo: &mut AnyPin<Input<Floating>>,
    trig: &mut AnyPin<Output<PushPull>>,
    delay: &mut Delay,
) -> u32 {
    // 1) Set pin ouput to low for 5 us to get clean low pulse
    delay.delay(5.millis());
    trig.set_low();

    // 2) Set pin output to high (trigger) for 10us
    trig.set_high();
    delay.delay(10.millis());
    trig.set_low();

    // Wait until pin goes high
    while !echo.is_high() {}

    // Kick off timer measurement
    let echo_start = SystemTimer::now();

    // Wait until pin goes low
    while !echo.is_low() {}

    // Collect current timer count
    let echo_end = SystemTimer::now();

    // Calculate the elapsed timer count
    let echo_dur = echo_end.wrapping_sub(echo_start);

    // Calculate the distance in cms using formula in datasheet
    let distance_cm = echo_dur / 16 / 58;

    // Print the distance output
    log::info!("Distance {} cm\r", distance_cm);
    distance_cm as u32
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let mut speed = 0u8;

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    init_heap();

    esp_println::logger::init_logger_from_env();
    let motor_Ain1 = io.pins.gpio5.into_push_pull_output();
    let motor_Ain2 = io.pins.gpio6.into_push_pull_output();
    let motor_Bin1 = io.pins.gpio8.into_push_pull_output();
    let motor_Bin2 = io.pins.gpio9.into_push_pull_output();
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let mut mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    let mut pwm_pinA = mcpwm
        .operator0
        .with_pin_a(io.pins.gpio4, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut pwm_pinB = mcpwm
        .operator1
        .with_pin_a(io.pins.gpio7, PwmPinConfig::UP_ACTIVE_HIGH);

    // Init ultrasonic sensor
    let mut echo = io.pins.gpio19.into_floating_input().degrade().into();
    let mut trig = io.pins.gpio18.into_push_pull_output().degrade().into();
    //echo_pin.listen(Event::AnyEdge);
    //interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority3).unwrap();

    // start timer with timestamp values in the range of 0..=99 and a frequency of 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // pin will be high 50% of the time
    pwm_pinA.set_timestamp(50);
    pwm_pinB.set_timestamp(50);

    let mut motor1 = Motor::new(motor_Ain1, motor_Ain2, pwm_pinA);
    let mut motor2 = Motor::new(motor_Bin1, motor_Bin2, pwm_pinB);

    /*
    let timer = esp_hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let _init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        esp_hal::rng::Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    */
    // let mut delay = Delay::new(&clocks);
    // let hcsr04 = HcSR04::hc_sr04_new(trig_pin, &mut delay, &mut my_counter);
    // let distance = hcsr04.get_distance::<f32>(DistanceUnit::MilliMeter);

    let mut dist_history = CircularBuffer::new();
    loop {
        let dist = check_distance(&mut echo, &mut trig, &mut delay);
        delay.delay(50.millis());
        //motor1.drive_forward(100).expect("");
        //motor2.drive_backwards(100).expect("");
        //delay.delay(1500.millis());
        //motor1.stop();
        //motor2.stop();
        //delay.delay(5500.millis());
        let m1cmd = motor1.current_drive_command();
        let m2cmd = motor2.current_drive_command();
        dist_history.push(dist as u32);
        /*
        log::info!(
            "{:?}\n{:?}\n{:?}\n{:?}",
            m1cmd,
            m2cmd,
            dist,
            dist_history.is_stopped(30)
        );
        */

        if dist < 40 || dist_history.is_stopped(30) {
            log::info!(
                "dist: {:?}\nis_stopped: {:?}",
                dist_history.average(),
                dist_history.is_stopped(30)
            );
            speed = 100;
            motor1.drive_backwards(speed).expect("");
            motor2.drive_backwards(speed).expect("");
            delay.delay(100.millis());
            motor1.stop();
            motor2.stop();
            motor1.drive_backwards(speed).expect("");
            delay.delay(300.millis());
            motor1.stop();
            motor2.stop();
            delay.delay(1500.millis());
        }

        if dist > 40 && dist < 2000 {
            log::info!("Closing in at: {:?}\n", dist);
            speed = 40;
            motor1.drive_forward(speed).expect("");
            motor2.drive_forward(speed).expect("");
        }
    }
}
