#![no_std]

use embassy_time::{Duration, Timer};
use esp_hal::{
    rmt::{asynch::TxChannelAsync, Channel, PulseCode},
    Async,
};

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum DSHOT_TELEMETRY_CMD {
    MOTOR_STOP = 0x0,
    BEEP1 = 0x01,
    BEEP2 = 0x02,
    BEEP3 = 0x03,
    BEEP4 = 0x04,
    BEEP5 = 0x05,
    ESC_INFO = 0x06,
    SPIN_DIRECTION_1 = 0x07,
    SPIN_DIRECTION_2 = 0x08,
    MODE_3D_OFF = 0x09,
    MODE_3D_ON = 0x0A,
    SETTINGS_REQUEST = 0x0B,
    SAVE_SETTINGS = 0x0C,
    EXTENDED_TELEMETRY_ENABLE = 0x0D,
    EXTENDED_TELEMETRY_DISABLE = 0x0E,
}

#[derive(Debug, Clone, Copy)]
pub struct BitTime {
    t0_h: u16,
    t0_l: u16,
    t1_h: u16,
    t1_l: u16,
}

impl BitTime {
    pub fn new(t0_h: u16, t1_h: u16) -> Self {
        Self {
            t0_h,
            t1_h,
            t0_l: t1_h,
            t1_l: t0_h,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum DShotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

impl DShotSpeed {
    pub fn bit_period_ns(&self) -> u32 {
        match self {
            // 6.67µs per bit
            Self::DShot150 => 6667,
            // 3.33µs per bit
            Self::DShot300 => 3333,
            // 1.67µs per bit
            Self::DShot600 => 1667,
            // 0.83µs per bit
            Self::DShot1200 => 833,
        }
    }

    // TODO: update for other clock speeds and dividers
    /// These are for an 80 MHz clock with a clock divider setting of 1
    pub fn bit_times(&self) -> BitTime {
        match &self {
            // 200 = 2.5 µs,  400 = 5.0 µs
            Self::DShot150 => BitTime::new(200, 400),
            // 100 = 1.25 µs,  200 = 2.5 µs
            Self::DShot300 => BitTime::new(100, 200),
            // 50 = 0.625 µs,  100 = 1.25 µs
            Self::DShot600 => BitTime::new(50, 100),
            // 25 = 0.3125 µs, 50 = 0.625 µs
            Self::DShot1200 => BitTime::new(25, 50),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DShot<TxCh> {
    channel: TxCh,
    speed: DShotSpeed,
    bit_times: BitTime,
}

impl<TxCh: TxChannelAsync> DShot<TxCh> {
    pub fn new(channel: TxCh, speed: DShotSpeed, bit_times: Option<BitTime>) -> Self {
        Self {
            channel,
            speed,
            bit_times: bit_times.unwrap_or(speed.bit_times()),
        }
    }

    pub fn create_frame(value: u16) -> u16 {
        let value = value & 0x07FF;
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF;
        (value << 5) | (crc << 1)
    }

    // TODO: add telemetry
    pub fn create_pulses(&mut self, throttle_value: u16) -> [PulseCode; 17] {
        let frame = Self::create_frame(throttle_value);
        let mut pulses = [PulseCode::default(); 17];

        for i in 0..16 {
            let bit = (frame >> (15 - i)) & 1;

            pulses[i] = if bit == 1 {
                PulseCode {
                    level1: true,
                    length1: self.bit_times.t1_h,
                    level2: false,
                    length2: self.bit_times.t1_l,
                }
            } else {
                PulseCode {
                    level1: true,
                    length1: self.bit_times.t0_h,
                    level2: false,
                    length2: self.bit_times.t0_l,
                }
            };
        }

        // Add empty pulse to end of pulses frame
        pulses[16] = PulseCode::default();
        pulses
    }

    pub async fn write_throttle(&mut self, throttle: u16) -> Result<(), &'static str> {
        let pulses = self.create_pulses(throttle);
        self.channel
            .transmit(&pulses)
            .await
            .map_err(|_| "Failed to send frame")?;
        Ok(())
    }

    pub async fn arm(&mut self) -> Result<(), &'static str> {
        for _ in 0..100 {
            self.write_throttle(0).await?;
            Timer::after(Duration::from_millis(20)).await;
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct QuadMotors {
    pub motor1: DShot<Channel<Async, 0>>,
    pub motor2: DShot<Channel<Async, 1>>,
    pub motor3: DShot<Channel<Async, 2>>,
    pub motor4: DShot<Channel<Async, 3>>,
}

impl QuadMotors {
    pub fn new(
        motor1: DShot<Channel<Async, 0>>,
        motor2: DShot<Channel<Async, 1>>,
        motor3: DShot<Channel<Async, 2>>,
        motor4: DShot<Channel<Async, 3>>,
    ) -> Self {
        Self {
            motor1,
            motor2,
            motor3,
            motor4,
        }
    }

    pub async fn arm(&mut self) -> Result<(), &'static str> {
        for _ in 0..100 {
            self.motor1.write_throttle(0).await?;
            self.motor2.write_throttle(0).await?;
            self.motor3.write_throttle(0).await?;
            self.motor4.write_throttle(0).await?;

            Timer::after(Duration::from_millis(20)).await;
        }

        Ok(())
    }

    pub async fn update_throttles(&mut self, throttles: &[u16; 4]) -> Result<(), &'static str> {
        self.motor1.write_throttle(throttles[0]).await?;
        self.motor2.write_throttle(throttles[1]).await?;
        self.motor3.write_throttle(throttles[2]).await?;
        self.motor4.write_throttle(throttles[3]).await?;

        Ok(())
    }
}
