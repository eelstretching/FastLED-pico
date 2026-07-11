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

#define SECONDS_PER_NANOSECOND 1e-9f

// we have 4 bits to store delay in instruction encoding with one sideset bit,
// but we can accept up to 16 because 1 is always subtracted first
#define CLOCKLESS_PIO_MAX_DELAY (1 << (5 - CLOCKLESS_PIO_SIDESET_COUNT))

// Converts from nanonseconds (which we need for the LED timing) to the number of clock cycles at the current CPU frequency.
static inline int nanoseconds_to_cycles(float ns) FL_NO_EXCEPT {
    return (int)(ns * clock_get_hz(clk_sys) * SECONDS_PER_NANOSECOND);
}

// A simple class to hold the PIO program and associated state machine and DMA
// channel for a clockless LED controller. This will let us easily delete the
// program and free the state machine and DMA channel when the controller is
// destroyed.
class PIOProgramInfo {
   public:
    pio_program_t* pio_program = nullptr;
    PIO mPio = nullptr;
    uint mSm = 0;
    uint mPioOffset = 0;
    int dma_channel = -1;
    void* dma_buf = nullptr;
    size_t dma_buf_size = 0;
    dma_channel_transfer_size tsize;
    float pio_clock_multiplier;
    int T1_ns = 0, T2_ns = 0, T3_ns = 0;
    int T1_cyc = 0, T2_cyc = 0, T3_cyc = 0;

    int startPin = 0;
    int numPins = 0;

    PIOProgramInfo(int T1_ns, int T2_ns, int T3_ns, int waitTime, u8 startPin, u8 numPins) : T1_ns(T1_ns), T2_ns(T2_ns), T3_ns(T3_ns), startPin(startPin), numPins(numPins) {
        // convert from input timebase (nanoseconds) to the number of clock cycles at the current CPU frequency
        // convert from input timebase to one that the PIO program can handle
        T1_cyc = nanoseconds_to_cycles(T1_ns);
        T2_cyc = nanoseconds_to_cycles(T2_ns);
        T3_cyc = nanoseconds_to_cycles(T3_ns);
        
        int max_t = MAX(T3_cyc, MAX(T1_cyc, T2_cyc));

        if (max_t > CLOCKLESS_PIO_MAX_DELAY) {
            // We need to set a divider on the PIO clock to slow it down so that the longest delay fits in the instruction encoding.
            pio_clock_multiplier = (float) CLOCKLESS_PIO_MAX_DELAY / max_t;
            
            T1_cyc = pio_clock_multiplier * T1_cyc;
            T2_cyc = pio_clock_multiplier * T2_cyc;
            T3_cyc = pio_clock_multiplier * T3_cyc;
        } else {
            pio_clock_multiplier = 1.0f;
        }

        Serial1.printf("PIO clock multiplier: %f, T1_ns: %d T1_cyc: %d, T2_ns: %d T2_cyc: %d, T3_ns: %d T3_cyc: %d\n", 
            pio_clock_multiplier, T1_ns, T1_cyc, T2_ns, T2_cyc, T3_ns, T3_cyc);

        if (numPins == 1) {
            tsize = DMA_SIZE_32;
        } else {
            tsize = DMA_SIZE_8;
        }

    };

    ~PIOProgramInfo() {
        cleanup();
    };

    boolean init(std::pair<pio_instr*, u8> pio_instructions) FL_NO_EXCEPT {
        pio_program = new pio_program_t{
            .instructions = pio_instructions.first,
            .length = pio_instructions.second,
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

        // Find a free PIO and SM for our clockless program. This does all the heavy lifting.
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
            pio_program, &mPio, &mSm,
            &mPioOffset, startPin, numPins,
            true);

        if (!success) {
            // failed to claim a PIO and state machine for our program
            FASTLED_DBG(
                "Failed to claim a PIO and state machine for clockless "
                "program");
            return false;
        }

        // claim an unused DMA channel (there's 12 in total,, so this should
        // also usually work out fine)
        dma_channel = dma_claim_unused_channel(false);
        if (dma_channel == -1) {
            // Clean up the state machine? WE can still use it without DMA.
            return false;  // no free DMA channel
        }

        // setup PIO state machine
        pio_gpio_init(mPio, startPin);
        for (uint i = startPin; i < startPin + numPins; i++) {
            pio_gpio_init(mPio, i);
            gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_4MA);
        }

        pio_sm_set_consecutive_pindirs(mPio, mSm, startPin, numPins, true);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, mPioOffset + CLOCKLESS_PIO_WRAP_TARGET,
                           mPioOffset + CLOCKLESS_PIO_WRAP);
        sm_config_set_out_pins(&c, startPin, numPins);

        // Different transfer sizes require different shift.
        sm_config_set_out_shift(&c, false, true, tsize == DMA_SIZE_32 ? 32 : 8);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        float div = clock_get_hz(clk_sys) / (pio_clock_multiplier * CLOCKLESS_FREQUENCY);
        sm_config_set_clkdiv(&c, div);

        printf("clock: %d cycles: %d div: %f\n", clock_get_hz(clk_sys),
               T1_cyc+T2_cyc+T3_cyc, div);
        float cycle_time = 1000000000.0 / (clock_get_hz(clk_sys) / div);
        printf("state machine cycle is: %.1fns\n", cycle_time);
        printf("T1: %d %.1f T2: %d %.1f T3: %d %.1f\n", T1_cyc, T1_cyc * cycle_time, T2_cyc, T2_cyc * cycle_time, T3_cyc, T3_cyc * cycle_time);

        pio_sm_init(mPio, mSm, mPioOffset, &c);
        pio_sm_set_enabled(mPio, mSm, true);

        // setup DMA
        dma_channel_config channel_config = dma_channel_get_default_config(dma_channel);
        // Note that we're using a transfer size based on the number of pins. If
        // we have more than 1 pin, we use 8-bit transfers, otherwise we use
        // 32-bit transfers.
        channel_config_set_transfer_data_size(&channel_config, tsize);
        channel_config_set_dreq(&channel_config, pio_get_dreq(mPio, mSm, true));
        channel_config_set_read_increment(&channel_config, true);
        channel_config_set_write_increment(&channel_config, false);

        dma_channel_configure(dma_channel, 
                              &channel_config,
                              &mPio->txf[mSm],
                              nullptr,  // address set when making transfer
                              1,        // count set when making transfer
                              false);   // don't trigger now

        return true;
    };

    void resetSM() {
        // Reset the PIO state machine before starting new transfer to prevent
        // freeze
        // This clears any stale state from the previous transfer (RP2350 fix)
        pio_sm_set_enabled(mPio, mSm, false);
        pio_sm_clear_fifos(mPio, mSm);
        pio_sm_restart(mPio, mSm);
        pio_sm_exec(mPio, mSm, pio_encode_jmp(mPioOffset));  // Jump back to program start
        pio_sm_set_enabled(mPio, mSm, true);
    };

    void cleanup() {
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

    void print() {
        Serial1.printf("PIOProgramInfo for sm %d dma %d startPin %d numPins %d\n", mSm, dma_channel, startPin, numPins);
        Serial1.printf("PIO program length %d\n", pio_program->length);
        for(int i = 0; i < pio_program->length; i++) {
            Serial1.printf(" Instruction: %d: %0x\n", i, pio_program->instructions[i]);
        }
    };
};

static inline std::pair<pio_instr*,u8> get_clockless_pio_program(int T1, int T2, int T3) FL_NO_EXCEPT {
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
    return std::make_pair(clockless_pio_instr, 4);
}

static inline std::pair<pio_instr*,u8> get_clockless_parallel_pio_program(int T1, int T2, int T3) FL_NO_EXCEPT {

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
    return std::make_pair(clockless_pio_instr, 4);
}
#endif  // _PIO_GEN_H
