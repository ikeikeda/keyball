/*
Copyright 2022 MURAOKA Taro (aka KoRoN, @kaoriya)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Modifications 2025 by Nobuki Inoue (your @black_trooper)
- Added support for PMW3610
*/

#include "quantum.h"
#include "pmw3610.h"

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)
#define CONSTRAIN_HID_XY(amt) ((amt) < XY_REPORT_MIN ? XY_REPORT_MIN : ((amt) > XY_REPORT_MAX ? XY_REPORT_MAX : (amt)))

#define PMW3610_PRODUCT_ID          0x3E
#define PMW3610_REVISION_ID         0x01
#define PMW3610_SPI_CLOCK_ON        0xBA
#define PMW3610_SPI_CLOCK_OFF       0xB5
#define PMW3610_POWER_UP_CMD        0x5A
#define PMW3610_OBSERVATION1_READY  0x0F
#define PMW3610_PERFORMANCE_INIT    0x0D
#define PMW3610_RUN_DOWNSHIFT_INIT  0x04
#define PMW3610_REST1_RATE_INIT     0x04
#define PMW3610_REST1_DOWNSHIFT_INIT 0x0F

// Datasheet: >= 10ms after POWER_UP_RESET before the next SPI access.
#ifndef PMW3610_RESET_DELAY_MS
#    define PMW3610_RESET_DELAY_MS 10
#endif
#ifndef PMW3610_OBSERVATION_DELAY_MS
#    define PMW3610_OBSERVATION_DELAY_MS 10
#endif

void pmw3610_spi_start(void) {
    writePinLow(PMW3610_NCS_PIN);
    wait_us(1);
}

void pmw3610_spi_stop(void) {
    writePinHigh(PMW3610_NCS_PIN);
}

uint8_t pmw3610_reg_read(uint8_t reg_addr) {
    pmw3610_spi_start();

    setPinOutput(PMW3610_SDIO_PIN);
    for (int8_t i = 7; i >= 0; i--) {
        writePinLow(PMW3610_SCLK_PIN);
        writePin(PMW3610_SDIO_PIN, reg_addr & (1 << i));
        wait_us(1);
        writePinHigh(PMW3610_SCLK_PIN);
        wait_us(1);
    }
    setPinInput(PMW3610_SDIO_PIN);
    wait_us(4);
    uint8_t data = 0;
    for (int8_t i = 7; i >= 0; i--) {
        writePinLow(PMW3610_SCLK_PIN);
        wait_us(1);
        writePinHigh(PMW3610_SCLK_PIN);
        wait_us(1);
        data |= readPin(PMW3610_SDIO_PIN) << i;
    }
    wait_us(1);
    pmw3610_spi_stop();

    wait_us(1);
    return data;
}

void pmw3610_write_byte(uint8_t byte) {
    setPinOutput(PMW3610_SDIO_PIN);

    for (int8_t i = 7; i >= 0; i--) {
        writePinLow(PMW3610_SCLK_PIN);

        if (byte & (1 << i)) {
            writePinHigh(PMW3610_SDIO_PIN);
        } else {
            writePinLow(PMW3610_SDIO_PIN);
        }

        wait_us(1);

        writePinHigh(PMW3610_SCLK_PIN);
        wait_us(1);
    }

    writePinLow(PMW3610_SCLK_PIN);
}

static void pmw3610_reg_write_raw(uint8_t reg_addr, uint8_t data) {
    pmw3610_spi_start();
    pmw3610_write_byte(reg_addr | 0x80);
    pmw3610_write_byte(data);
    pmw3610_spi_stop();
    wait_us(20);
}

static void pmw3610_spi_clk_on(void) {
    pmw3610_reg_write_raw(PMW3610_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_ON);
    wait_us(300);
}

static void pmw3610_spi_clk_off(void) {
    pmw3610_reg_write_raw(PMW3610_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_OFF);
}

void pmw3610_reg_write(uint8_t reg_addr, uint8_t data) {
    pmw3610_spi_clk_on();
    pmw3610_reg_write_raw(reg_addr, data);
    pmw3610_spi_clk_off();
}

void pmw3610_cpi_set(uint8_t cpi) {
    if (cpi > pmw3610_MAXCPI) {
        cpi = pmw3610_MAXCPI;
    }

    pmw3610_reg_write(PMW3610_SPI_PAGE0, 0xFF);
    pmw3610_reg_write(PMW3610_RES_STEP, cpi);
    pmw3610_reg_write(PMW3610_SPI_PAGE1, 0x00);
}

bool pmw3610_motion_read(pmw3610_motion_t *d) {
    uint8_t mot = pmw3610_reg_read(PMW3610_MOTION);
    if (!(mot & 0x80)) {
        return false;
    }
    uint8_t xl  = pmw3610_reg_read(PMW3610_DELTA_X_L);
    uint8_t yl  = pmw3610_reg_read(PMW3610_DELTA_Y_L);
    uint8_t xyh = pmw3610_reg_read(PMW3610_DELTA_XY_H);
    uint8_t xh = (xyh & 0xF0) >> 4;
    uint8_t yh = (xyh & 0x0F);

    int16_t dx = TOINT16((xl | (xh << 8)), 12);
    int16_t dy = TOINT16((yl | (yh << 8)), 12);

    d->x = CONSTRAIN_HID_XY(dx);
    d->y = CONSTRAIN_HID_XY(dy);

    return true;
}

// TRRS 側は MCU よりセンサ電源が遅れる。NCS を High にしたまま待ち、
// 失敗時はリセットからやり直す。config.h で上書きできる。
#ifndef PMW3610_INIT_DELAY_MS
#    define PMW3610_INIT_DELAY_MS 200
#endif
#ifndef PMW3610_INIT_RETRIES
#    define PMW3610_INIT_RETRIES 3
#endif
#ifndef PMW3610_INIT_RETRY_DELAY_MS
#    define PMW3610_INIT_RETRY_DELAY_MS 200
#endif

static bool pmw3610_init_once(void) {
    // POWER_UP_RESET は SPI クロック on/off で挟まない。
    // リセット直後の無通信時間を確保するため。
    pmw3610_reg_write_raw(PMW3610_POWER_UP_RESET, PMW3610_POWER_UP_CMD);
    wait_ms(PMW3610_RESET_DELAY_MS);

    uint8_t pid = pmw3610_reg_read(PMW3610_PROD_ID);
    uint8_t rev = pmw3610_reg_read(PMW3610_REV_ID);
    dprintf("pmw3610_init: pid=0x%02X rev=0x%02X\n", pid, rev);
    if (pid != PMW3610_PRODUCT_ID || rev != PMW3610_REVISION_ID) {
        return false;
    }

    pmw3610_spi_clk_on();
    pmw3610_reg_write_raw(PMW3610_OBSERVATION1, 0x00);
    wait_ms(PMW3610_OBSERVATION_DELAY_MS);

    uint8_t obs = pmw3610_reg_read(PMW3610_OBSERVATION1);
    if ((obs & PMW3610_OBSERVATION1_READY) != PMW3610_OBSERVATION1_READY) {
        dprintf("pmw3610_init: observation1=0x%02X\n", obs);
        pmw3610_spi_clk_off();
        return false;
    }

    pmw3610_reg_read(PMW3610_MOTION);
    pmw3610_reg_read(PMW3610_DELTA_X_L);
    pmw3610_reg_read(PMW3610_DELTA_Y_L);
    pmw3610_reg_read(PMW3610_DELTA_XY_H);

    pmw3610_reg_write_raw(PMW3610_PERFORMANCE, PMW3610_PERFORMANCE_INIT);
    pmw3610_reg_write_raw(PMW3610_RUN_DOWNSHIFT, PMW3610_RUN_DOWNSHIFT_INIT);
    pmw3610_reg_write_raw(PMW3610_REST1_RATE, PMW3610_REST1_RATE_INIT);
    pmw3610_reg_write_raw(PMW3610_REST1_DOWNSHIFT, PMW3610_REST1_DOWNSHIFT_INIT);
    pmw3610_spi_clk_off();

    return true;
}

bool pmw3610_init(void) {
    debug_enable = true;

    // AVR の setPinOutput は方向だけ変える。PORT が 0 のままだと NCS が
    // Low 駆動になり、センサ電源立ち上がり時に SPI モードに入れなくなる。
    setPinOutput(PMW3610_SCLK_PIN);
    writePinHigh(PMW3610_SCLK_PIN);
    setPinOutput(PMW3610_SDIO_PIN);
    writePinHigh(PMW3610_SDIO_PIN);
    setPinOutput(PMW3610_NCS_PIN);
    writePinHigh(PMW3610_NCS_PIN);

    wait_ms(PMW3610_INIT_DELAY_MS);
    for (uint8_t attempt = 0; attempt < PMW3610_INIT_RETRIES; attempt++) {
        if (attempt > 0) {
            wait_ms(PMW3610_INIT_RETRY_DELAY_MS);
        }
        if (pmw3610_init_once()) {
            return true;
        }
    }
    return false;
}
