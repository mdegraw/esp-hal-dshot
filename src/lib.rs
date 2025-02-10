#![no_std]

use embedded_hal_async::delay::DelayNs;
use esp_hal::rmt::{asynch::TxChannelAsync, PulseCode};
use num_traits::float::FloatCore;

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
pub struct BitTicks {
    t0_h: u16,
    t0_l: u16,
    t1_h: u16,
    t1_l: u16,
}

impl BitTicks {
    pub fn new(t1_h: u16, t0_h: u16) -> Self {
        Self {
            t0_h,
            t1_h,
            t0_l: t1_h,
            t1_l: t0_h,
        }
    }

    pub fn from_clk(clk_speed: u32, clk_divider: u8, bit_times: BitTimes) -> Self {
        let tick_len = (1. / clk_speed as f32) * (clk_divider as f32) * 1_000_000.;

        let t1_h = (bit_times.t1_h / tick_len).round() as u16;
        let t0_h = (bit_times.t0_h / tick_len).round() as u16;

        Self::new(t1_h, t0_h)
    }
}

/// High and Low Times in µs
#[derive(Debug, Clone, Copy)]
pub struct BitTimes {
    t0_h: f32,
    t1_h: f32,
}

impl BitTimes {
    pub fn new(t1_h: f32, t0_h: f32) -> Self {
        Self { t0_h, t1_h }
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

    /// High and Low Times in µs
    pub fn bit_times(&self) -> BitTimes {
        match &self {
            Self::DShot150 => BitTimes::new(5.00, 2.50),
            Self::DShot300 => BitTimes::new(2.50, 1.25),
            Self::DShot600 => BitTimes::new(1.25, 0.625),
            Self::DShot1200 => BitTimes::new(0.625, 0.313),
        }
    }

    /// These are for an 80 MHz clock with a clock divider setting of 1
    pub fn default_bit_ticks(&self) -> BitTicks {
        match &self {
            Self::DShot150 => BitTicks::new(400, 200),
            Self::DShot300 => BitTicks::new(200, 100),
            Self::DShot600 => BitTicks::new(100, 50),
            Self::DShot1200 => BitTicks::new(50, 25),
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct DShot<TxCh> {
    channel: TxCh,
    speed: DShotSpeed,
    bit_ticks: BitTicks,
}

impl<TxCh: TxChannelAsync> DShot<TxCh> {
    pub fn new(
        channel: TxCh,
        speed: DShotSpeed,
        clk_speed: Option<u32>,
        clk_divider: Option<u8>,
    ) -> Self {
        let clk_speed = clk_speed.unwrap_or(80_000_000);
        let clk_divider = clk_divider.unwrap_or(1);

        let bit_ticks = BitTicks::from_clk(clk_speed, clk_divider, speed.bit_times());

        Self {
            channel,
            speed,
            bit_ticks,
        }
    }

    pub fn calculate_crc(frame: u16) -> u16 {
        (frame ^ (frame >> 4) ^ (frame >> 8)) & 0xF
    }

    pub fn create_frame(value: u16, telemetry: bool) -> u16 {
        // Mask to 11 bits (0-2047 range) and set telemetry bit
        let frame = ((value & 0x07FF) << 1) | telemetry as u16;

        let crc = Self::calculate_crc(frame);

        (frame << 4) | crc
    }

    #[allow(clippy::needless_range_loop)]
    pub fn create_pulses(&mut self, throttle_value: u16, telemetry: bool) -> [PulseCode; 17] {
        let frame = Self::create_frame(throttle_value, telemetry);
        let mut pulses = [PulseCode::default(); 17];
        for i in 0..16 {
            let bit = (frame >> (15 - i)) & 1;

            pulses[i] = if bit == 1 {
                PulseCode {
                    level1: true,
                    length1: self.bit_ticks.t1_h,
                    level2: false,
                    length2: self.bit_ticks.t1_l,
                }
            } else {
                PulseCode {
                    level1: true,
                    length1: self.bit_ticks.t0_h,
                    level2: false,
                    length2: self.bit_ticks.t0_l,
                }
            };
        }

        // Add empty pulse to end of pulses frame
        pulses[16] = PulseCode::default();
        pulses
    }

    pub async fn write_throttle(
        &mut self,
        throttle: u16,
        telemetry: bool,
    ) -> Result<(), &'static str> {
        let pulses = self.create_pulses(throttle, telemetry);
        self.channel
            .transmit(&pulses)
            .await
            .map_err(|_| "Failed to send frame")?;
        Ok(())
    }

    pub async fn arm(&mut self, delay: &mut impl DelayNs) -> Result<(), &'static str> {
        for _ in 0..100 {
            self.write_throttle(0, false).await?;
            delay.delay_ms(20).await;
        }

        Ok(())
    }
}
