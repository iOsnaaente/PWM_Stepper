#ifdef GY521_SELF_TEST

#include "../board_config.h"
#include "Serial/SerialDebugger.h"

#include <Wire.h>

// Simple GY-521 (MPU-6050) self-test:
// - No motors, WiFi, or ESP-NOW
// - Initializes MPU-6050 over I2C and prints linear acceleration (g)
//   and angular rate (deg/s) as fast as practical.

static const uint8_t MPU_ADDR = 0x68; // GY-521 default I2C address

static bool mpu_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static bool mpu_read_bytes(uint8_t startReg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false; // repeated start
    size_t read = Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true);
    if (read != len) return false;
    for (size_t i = 0; i < len; ++i) {
        buf[i] = Wire.read();
    }
    return true;
}

static bool mpu_init() {
    // Wake up device (clear sleep bit)
    if (!mpu_write_reg(0x6B, 0x00)) return false; // PWR_MGMT_1
    delay(100);
    // Set full-scale ranges: accel ±2g (0), gyro ±250 deg/s (0)
    if (!mpu_write_reg(0x1C, 0x00)) return false; // ACCEL_CONFIG
    if (!mpu_write_reg(0x1B, 0x00)) return false; // GYRO_CONFIG
    // Optional: low-pass filter / sample rate config could be added here
    return true;
}

void setup() {
    serial_debugger_init();
    DEBUG_SERIAL("GY521", "GY-521 (MPU-6050) self-test mode");

    // Initialize I2C on the same pins as the OLED
    Wire.begin((int)OLED_SDA_PIN, (int)OLED_SCL_PIN);
    Wire.setClock(400000); // 400 kHz I2C for faster reads

    if (!mpu_init()) {
        DEBUG_SERIAL("GY521", "Failed to initialize MPU-6050 at 0x%02X", MPU_ADDR);
    } else {
        DEBUG_SERIAL("GY521", "MPU-6050 initialized at 0x%02X", MPU_ADDR);
    }
}

void loop() {
    uint8_t raw[14];
    if (mpu_read_bytes(0x3B, raw, sizeof(raw))) {
        int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
        int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

        // Convert to physical units (assuming FS_SEL=0 on both):
        //   accel: 16384 LSB/g
        //   gyro : 131 LSB/(deg/s)
        float ax_g = (float)ax / 16384.0f;
        float ay_g = (float)ay / 16384.0f;
        float az_g = (float)az / 16384.0f;
        float gx_dps = (float)gx / 131.0f;
        float gy_dps = (float)gy / 131.0f;
        float gz_dps = (float)gz / 131.0f;

        USB_BUS.printf("ACC[g] X=%.3f Y=%.3f Z=%.3f | GYR[dps] X=%.3f Y=%.3f Z=%.3f\r\n",
                       ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
    }

    // Short delay to avoid flooding USB and give the sensor time
    delay(2); // ~500 Hz target, adjust as needed
}

#endif // GY521_SELF_TEST
