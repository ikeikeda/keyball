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

#pragma once

#include <stdint.h>
#include "spi_master.h"

//////////////////////////////////////////////////////////////////////////////
// Configurations

#define PMW3610_NCS_PIN B6
#define PMW3610_SDIO_PIN B2
#define PMW3610_SCLK_PIN B1

//////////////////////////////////////////////////////////////////////////////
// Top level API

/// Drive NCS, SCLK and SDIO high without starting SPI.
/// Call this as early as possible so the sensor rail can rise with NCS high.
void pmw3610_prepare(void);

/// pmw3610_init initializes PMW3610DM-T2QU module.
/// Holds NCS high while the sensor rail settles, runs the datasheet
/// power-up sequence (including OBSERVATION1), and retries on failure.
bool pmw3610_init(void);

/// One power-up sequence, without the initial settle delay or retry loop.
/// NCS is left high. Used to retry after the TRRS rail has come up.
bool pmw3610_init_attempt(void);

typedef struct {
    int16_t x;
    int16_t y;
} pmw3610_motion_t;

/// pmw3610_motion_read gets a motion data by Motion register.
/// This requires to write a dummy data to pmw3610_Motion register
/// just before.
bool pmw3610_motion_read(pmw3610_motion_t *d);

// TODO: document
void pmw3610_cpi_set(uint8_t cpi);

//////////////////////////////////////////////////////////////////////////////
// Register operations

/// pmw3610_reg_write writes a value to a register.
void pmw3610_reg_write(uint8_t addr, uint8_t data);

/// pmw3610_reg_read reads a value from a register.
uint8_t pmw3610_reg_read(uint8_t addr);

typedef enum {
    PMW3610_PROD_ID         = 0x00,
    PMW3610_REV_ID          = 0x01,
    PMW3610_MOTION          = 0x02,
    PMW3610_DELTA_X_L       = 0x03,
    PMW3610_DELTA_Y_L       = 0x04,
    PMW3610_DELTA_XY_H      = 0x05,
    PMW3610_PERFORMANCE     = 0x11,
    PMW3610_RUN_DOWNSHIFT   = 0x1B,
    PMW3610_REST1_RATE      = 0x1C,
    PMW3610_REST1_DOWNSHIFT = 0x1D,
    PMW3610_OBSERVATION1    = 0x2D,
    PMW3610_POWER_UP_RESET  = 0x3A,
    PMW3610_SPI_CLK_ON_REQ  = 0x41,
    PMW3610_SPI_PAGE0       = 0x7F,
    PMW3610_RES_STEP        = 0x85,
    PMW3610_SPI_PAGE1       = 0xFF
} pmw3610_reg_t;

enum {
    pmw3610_MAXCPI = 0x77, // = 119: 12000 CPI
};

//////////////////////////////////////////////////////////////////////////////
// SPI operations

void pmw3610_spi_start(void);

void pmw3610_spi_stop(void);