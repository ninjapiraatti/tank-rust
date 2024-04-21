#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    peripherals::{Peripherals},
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
    prelude::*
};
use tb6612fng::{Motor};

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
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

    loop {
        log::info!("Heeeello world!");
        delay.delay(500.millis());
        motor1.drive_forward(100).expect("");
        motor2.drive_backwards(100).expect("");
        delay.delay(1500.millis());
        motor1.stop();
        motor2.stop();
        delay.delay(5500.millis());
        let m1cmd = motor1.current_drive_command();
        let m2cmd = motor2.current_drive_command();
        log::info!("{:?} | {:?}", m1cmd, m2cmd);
    }
}
