// IWYU pragma: private
#pragma once
// ok no namespace fl
#ifndef _PIO_GEN_H
#define _PIO_GEN_H

#include "platforms/arm/rp/rpcommon/pio_asm.h"
// IWYU pragma: begin_keep
#include "fl/stl/noexcept.h"
#include "hardware/pio.h"

// IWYU pragma: end_keep
/*
 * This file contains code to manage the PIO program for clockless LEDs.
 *
 * A PIO program is "assembled" from compiler macros so that T1, T2, T3 can be
 * set from other code.
 * Otherwise, this is quite similar to what would be output by pioasm, with the
 * additional step of adding the program to a state machine integrated.
 */

#define CLOCKLESS_PIO_SIDESET_COUNT 0

#define CLOCKLESS_PIO_WRAP_TARGET 0
#define CLOCKLESS_PIO_WRAP 3

// we have 4 bits to store delay in instruction encoding with one sideset bit,
// but we can accept up to 16 because 1 is always subtracted first
#define CLOCKLESS_PIO_MAX_TIME_PERIOD (1 << (5 - CLOCKLESS_PIO_SIDESET_COUNT))

// A simple class to hold the PIO program and associated state machine and DMA
// channel for a clockless LED controller. This will let us easily delete the
// program and free the state machine and DMA channel when the controller is
// destroyed.
class PIOProgramInfo {
   public:
    pio_program_t* pio_program;
    PIO mPio = nullptr;
    uint mSm = 0;
    uint mPioOffset = 0;
    int dma_channel = -1;
    void* dma_buf = nullptr;
    size_t dma_buf_size = 0;
    float pio_clock_multiplier;
    int T1_mult, T2_mult, T3_mult;

    int startPin = 0;
    int numPins = 0;

    PIOProgramInfo(pio_instr *pio_instructions, u8 startPin, u8 numPins) {
        this->pio_program = new pio_program_t{
            .instructions = pio_instructions,
            .length = sizeof(pio_instructions) / sizeof(pio_instructions[0]),
            .origin = -1,
#if defined(PICO_SDK_VERSION_MAJOR) && PICO_SDK_VERSION_MAJOR >= 2
            // pico-sdk 2.x added these two fields to `pio_program`. They are
            // gated by *different* macros in the SDK header — getting the
            // gating wrong here is what regressed RP2040 builds in #2792 /
            // #2727.
            //
            //   * `pio_version` is unconditionally present in 2.x — guard on
            //     PICO_SDK_VERSION_MAJOR only.
            //   * `used_gpio_ranges` is per-chip, gated on PICO_PIO_VERSION > 0
            //     in rp2_common/hardware_pio/include/hardware/pio.h. RP2350
            //     defines `PICO_PIO_VERSION = 1` so the field exists; RP2040
            //     defines `PICO_PIO_VERSION = 0` so the field is genuinely
            //     absent even on the 2.x SDK.
            //
            // Zero-init preserves the previous implicit behavior while
            // silencing -Wmissing-field-initializers and documenting intent: no
            // specific PIO version pinned, no GPIO range pre-claimed. Revisit
            // if the SDK starts enforcing `used_gpio_ranges` at pio_add_program
            // time.
            .pio_version = 0,
#if defined(PICO_PIO_VERSION) && PICO_PIO_VERSION > 0
            .used_gpio_ranges = 0,
#endif
#endif
        };
        this->startPin = startPin;
        this->numPins = numPins;
    };

    ~PIOProgramInfo() {
        pio_sm_set_enabled(mPio, mSm, false);
        pio_sm_unclaim(mPio, mSm);
        pio_remove_program(mPio, pio_program, mPioOffset);
        dma_channel_abort(dma_channel);
        dma_channel_unclaim(dma_channel);
        if (dma_buf) {
            free(dma_buf);
        }
        if (pio_program) {
            delete pio_program->instructions;
            delete pio_program;
        }
    };
};

static inline pio_instr* get_clockless_pio_program(int T1, int T2,
                                                   int T3) FL_NO_EXCEPT {
    pio_instr* clockless_pio_instr = new pio_instr[4]{
        // wrap_target
        // out x, 1; read next bit to x
        (pio_instr)(PIO_INSTR_OUT | PIO_OUT_DST_X | PIO_OUT_CNT(1)),
        // set pins, 1 [T1 - 1]; set output high for T1
        (pio_instr)(PIO_INSTR_SET | PIO_SET_DST_PINS | PIO_SET_DATA(1) |
                    PIO_DELAY(T1 - 1, CLOCKLESS_PIO_SIDESET_COUNT)),
        // mov pins, x [T2 - 1]; set output to X for T2
        (pio_instr)(PIO_INSTR_MOV | PIO_MOV_DST_PINS | PIO_MOV_SRC_X |
                    PIO_DELAY(T2 - 1, CLOCKLESS_PIO_SIDESET_COUNT)),
        // set pins, 0 [T3 - 2] // set output low for T3 (minus two because
        // we'll also read next bit using one instruction during this time)
        (pio_instr)(PIO_INSTR_SET | PIO_SET_DST_PINS | PIO_SET_DATA(0) |
                    PIO_DELAY(T3 - 2, CLOCKLESS_PIO_SIDESET_COUNT)),
        // wrap
    };

    return clockless_pio_instr;
}

static inline pio_instr* get_clockless_parallel_pio_program(
    int T1, int T2, int T3) FL_NO_EXCEPT {
    pio_instr* clockless_pio_instr = new pio_instr[4]{
        // wrap_target
        // out x, 8 ; Read 8 bits from the OSR into X, autopulling from the
        // TX FIFO, blocking until data is ready.
        (pio_instr)(PIO_INSTR_OUT | PIO_OUT_DST_X | PIO_OUT_CNT(8)),
        // mov pins, !null [T1-1]  Write high for T1 cycles
        (pio_instr)(PIO_INSTR_MOV | PIO_MOV_DST_PINS | PIO_MOV_OP_INVERT |
                    PIO_MOV_SRC_NULL |
                    PIO_DELAY(T1 - 1, CLOCKLESS_PIO_SIDESET_COUNT)),
        // mov pins, x     [T2-1]  Write the data bits to the pins,
        // autopulls data from TX FIFO.
        (pio_instr)(PIO_INSTR_MOV | PIO_MOV_DST_PINS | PIO_MOV_SRC_X |
                    PIO_DELAY(T2 - 1, CLOCKLESS_PIO_SIDESET_COUNT)),
        // mov pins, null  [T3-2]  Write 0 bits to all pins, account for the
        // instruction at the top of the loop
        (pio_instr)(PIO_INSTR_MOV | PIO_MOV_DST_PINS | PIO_MOV_SRC_NULL |
                    PIO_DELAY(T3 - 2, CLOCKLESS_PIO_SIDESET_COUNT)),
        // wrap
    };

    return clockless_pio_instr;
}

static inline pio_sm_config clockless_pio_program_get_default_config(
    uint offset) FL_NO_EXCEPT {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + CLOCKLESS_PIO_WRAP_TARGET,
                       offset + CLOCKLESS_PIO_WRAP);
    sm_config_set_sideset(&c, CLOCKLESS_PIO_SIDESET_COUNT, false, false);
    return c;
}

#endif  // _PIO_GEN_H
