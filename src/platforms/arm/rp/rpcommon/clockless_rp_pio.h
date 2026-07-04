// IWYU pragma: private

#ifndef __INC_CLOCKLESS_RP_PIO_COMMON
#define __INC_CLOCKLESS_RP_PIO_COMMON

#include "fl/chipsets/timing_traits.h"
// IWYU pragma: begin_keep
#include "hardware/structs/sio.h"
// IWYU pragma: end_keep
#include "fastled_delay.h"
#include "platforms/arm/rp/is_rp.h"  // FL_IS_RP2040, FL_IS_RP2350

#if FASTLED_RP2040_CLOCKLESS_M0_FALLBACK || !FASTLED_RP2040_CLOCKLESS_PIO
#include "platforms/arm/common/m0clockless.h"
#endif

#if FASTLED_RP2040_CLOCKLESS_PIO
// IWYU pragma: begin_keep
#include "hardware/clocks.h"
// IWYU pragma: end_keep
// IWYU pragma: begin_keep
#include "hardware/dma.h"
// compiler throws a warning about comparison that is always true
// silence that so users don't see it
// IWYU pragma: end_keep
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
// IWYU pragma: begin_keep
#include "hardware/pio.h"
// IWYU pragma: end_keep
#pragma GCC diagnostic pop

#include "platforms/arm/rp/rpcommon/pio_gen.h"
#include "fl/stl/allocator.h"
#include "fl/stl/cstring.h"
#include "fl/stl/noexcept.h"
#endif

/*
 * This clockless implementation uses RP2040's PIO feature to perform
 * non-blocking transfers to LEDs with very little memory overhead.
 * (allocates one buffer of equal size to the data to be sent)
 *
 * The SDK-provided claims system is used so that resources can used without
 * interfering with other code that behaves well and uses claims.
 *
 * Resource usage is 4 instructions of program memory on the first PIO instance
 * with an unclaimed state machine, said unclaimed PIO state machine, and one
 * DMA channel per instance of ClocklessController.
 * Additionally, one interrupt handler for DMA_IRQ_0 (configurable as shared or
 * exclusive via FASTLED_RP2040_CLOCKLESS_IRQ_SHARED) is used regardless of how
 * many instances are created.
 *
 * The DMA handler is likely the only significant risk in terms of conflicts,
 * and users can adapt other code to use DMA_IRQ_1 and/or adopt shared handlers
 * to avoid this becoming an issue.
 */
namespace fl {
#define FL_CLOCKLESS_CONTROLLER_DEFINED 1

#if FASTLED_RP2040_CLOCKLESS_PIO
static CMinWait<0>* dma_chan_waits[NUM_DMA_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static inline void __isr clockless_dma_complete_handler() FL_NO_EXCEPT {
    for (u32 i = 0; i < NUM_DMA_CHANNELS; i++) {
        // if dma triggered for this channel and it's been used (has a CMinWait)
        if ((dma_hw->ints0 & (1 << i)) && dma_chan_waits[i]) {
            dma_hw->ints0 = (1 << i); // clear/ack IRQ
            dma_chan_waits[i]->mark(); // mark the wait
            return;
        }
    }
}
static bool clockless_isr_installed = false;
#endif

template <u8 DATA_PIN, typename TIMING, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 280>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {
    // Extract timing values from struct and convert from nanoseconds to clock cycles 
    // Formula: cycles = (nanoseconds * CPU_MHz + 500) / 1000
    // The +500 provides rounding to nearest integer
    static constexpr int T1 = (TIMING::T1 * (F_CPU / 1000000UL) + 500) / 1000;
    static constexpr int T2 = (TIMING::T2 * (F_CPU / 1000000UL) + 500) / 1000;
    static constexpr int T3 = (TIMING::T3 * (F_CPU / 1000000UL) + 500) / 1000;

#if FASTLED_RP2040_CLOCKLESS_PIO
    // Get a container for the PIO program and associated state machine and DMA
    // channel, which we'll fill in during init().
    PIOProgramInfo* ppi = new PIOProgramInfo(
        get_clockless_pio_program(T1, T2, T3), (u8)DATA_PIN, (u8)1);

    // increase wait time by time taken to send 4 words (to flush PIO TX buffer)
    CMinWait<WAIT_TIME + ( ((T1 + T2 + T3) * 32 * 4) / (CLOCKLESS_FREQUENCY / 1000000) )> mWait;

    // start a DMA transfer to the PIO state machine from addr (transfer count
    // 32 bit words)
    static void do_dma_transfer(int channel, const void *addr, uint count) FL_NO_EXCEPT {
        dma_channel_set_read_addr(channel, addr, false);
        dma_channel_set_trans_count(channel, count, true);
    }

    // writes bits to an in-memory buffer (to DMA from)
    // pico has enough memory to not really care about using a buffer for DMA
    template <int BITS> __attribute__((always_inline)) inline static int writeBitsToBuf(i32 *out_buf, u32 bitpos, u8 b) FL_NO_EXCEPT {
        // not really optimised and I haven't checked output assembly, but this
        // should take ~50 cycles worst case (and on average substantially fewer
        // -- LEDs without XTRA0 should never trigger the second half of the
        // function)

        // position of word that takes highest bits (first word used)
        int wordpos_1 = bitpos >> 5;  // bitpos / 32;

        // number of bits from the byte that fit into first word
        int bitcnt_1 = 32 - (bitpos & 0b11111); // bitpos % 32;
        // shift required to place byte within the word
        int bitshift_1 = bitcnt_1 - 8;
        // mask for output bits that are taken from input
        // int32_t bitmask_1 = 0xFF << bitshift_1;
        i32 bitmask_1 = ((1 << BITS) - 1) << (bitshift_1 - (BITS - 8));

        out_buf[wordpos_1] = (out_buf[wordpos_1] & ~bitmask_1) | ((b << bitshift_1) & bitmask_1);

        if (bitcnt_1 >= BITS) return BITS;  // fast case for entire byte fitting in word

        // number of bits from the byte to place into second word
        int bitcnt_2 = 8 - bitcnt_1;
        // shift required to place byte within the word
        int bitshift_2 = 32 - bitcnt_2;
        // mask for output bits that are taken from input
        // int32_t bitmask_2 = ((1 << bitcnt_2) - 1) << bitshift_2;
        i32 bitmask_2 = ((1 << (bitcnt_2 + (BITS - 8))) - 1)
                        << (bitshift_2 - (BITS - 8));  // fixed XTRA0

        out_buf[wordpos_1 + 1] = (out_buf[wordpos_1 + 1] & ~bitmask_2) | ((b << bitshift_2) & bitmask_2);

        return BITS;
    }
#else
    CMinWait<WAIT_TIME> mWait;
#endif
   public:
    virtual void init() FL_NO_EXCEPT {
#if FASTLED_RP2040_CLOCKLESS_PIO
        if (ppi->dma_channel != -1) return;  // maybe init was called twice somehow? not sure if possible
#endif

        // start by configuring pin as output for blocking fallback
        FastPin<DATA_PIN>::setOutput();

#if FASTLED_RP2040_CLOCKLESS_PIO
        // convert from input timebase to one that the PIO program can handle
        int max_t = T1 > T2 ? T1 : T2;
        max_t = T3 > max_t ? T3 : max_t;

        if (max_t > CLOCKLESS_PIO_MAX_TIME_PERIOD) {
            ppi->pio_clock_multiplier = (float)CLOCKLESS_PIO_MAX_TIME_PERIOD / max_t;
            ppi->T1_mult = ppi->pio_clock_multiplier * T1;
            ppi->T2_mult = ppi->pio_clock_multiplier * T2;
            ppi->T3_mult = ppi->pio_clock_multiplier * T3;
        } else {
            ppi->pio_clock_multiplier = 1.f;
            ppi->T1_mult = T1;
            ppi->T2_mult = T2;
            ppi->T3_mult = T3;
        }

        // This will find a free pio and state machine for our program (whether
        // serial or parallel) and load it for us.
        //
        // We use
        // pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range
        // variant) so we will get a PIO instance suitable for addressing gpios
        // >= 32 if needed and supported by the hardware.
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
            ppi->pio_program, &ppi->mPio, &ppi->mSm,
            &ppi->mPioOffset, ppi->startPin, ppi->numPins,
            true);

        if (!success) {
            // failed to claim a PIO and state machine for our program
            FASTLED_DBG(
                "Failed to claim a PIO and state machine for clockless "
                "program");
            return;
        }

        // claim an unused DMA channel (there's 12 in total,, so this should
        // also usually work out fine)
        ppi->dma_channel = dma_claim_unused_channel(false);
        if (ppi->dma_channel == -1) return;  // no free DMA channel

        // setup PIO state machine
        pio_gpio_init(ppi->mPio, ppi->startPin);
        pio_sm_set_consecutive_pindirs(ppi->mPio, ppi->mSm,
                                       ppi->startPin,
                                       ppi->numPins, true);

        pio_sm_config c = clockless_pio_program_get_default_config(ppi->mPioOffset);
        sm_config_set_set_pins(&c, ppi->startPin, ppi->numPins);
        sm_config_set_out_pins(&c, ppi->startPin, ppi->numPins);
        sm_config_set_out_shift(&c, false, true, 32);

        // uncommenting this makes the FIFO 8 words long,
        // which seems like it won't actually benefit us
        // sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

        float div = clock_get_hz(clk_sys) / (ppi->pio_clock_multiplier * CLOCKLESS_FREQUENCY);
        sm_config_set_clkdiv(&c, div);

        pio_sm_init(ppi->mPio, ppi->mSm, ppi->mPioOffset, &c);
        pio_sm_set_enabled(ppi->mPio, ppi->mSm, true);

        // setup DMA
        dma_channel_config channel_config = dma_channel_get_default_config(ppi->dma_channel);
        channel_config_set_dreq(&channel_config, pio_get_dreq(ppi->mPio, ppi->mSm, true));
        dma_channel_configure(ppi->dma_channel, 
                              &channel_config,
                              &ppi->mPio->txf[ppi->mSm],
                              nullptr,  // address set when making transfer
                              1,        // count set when making transfer
                              false);   // don't trigger now

        // setup DMA complete interrupt handler to update mWait time after
        // transfer

        // store a pointer to mWait of this instance to a global array for the
        // interrupt handler kinda dirty hack here to cast to CMinWait<0>*, but
        // only mark is used, which isn't affected by the template var WAIT
        dma_chan_waits[ppi->dma_channel] = (CMinWait<0>*)&mWait;

        if (!clockless_isr_installed) {
#if FASTLED_RP2040_CLOCKLESS_IRQ_SHARED
            irq_add_shared_handler(DMA_IRQ_0, clockless_dma_complete_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
#else
            irq_set_exclusive_handler(DMA_IRQ_0, clockless_dma_complete_handler);
#endif
            irq_set_enabled(DMA_IRQ_0, true);
            clockless_isr_installed = true;
        }
        dma_channel_set_irq0_enabled(ppi->dma_channel, true);
#endif  // FASTLED_RP2040_CLOCKLESS_PIO
    }

    virtual u16 getMaxRefreshRate() const { return 400; }

    virtual void showPixels(PixelController<RGB_ORDER>& pixels) FL_NO_EXCEPT {
#if FASTLED_RP2040_CLOCKLESS_PIO
        if (ppi->dma_channel ==
            -1) {  // setup failed, so fall back to a blocking implementation
#if FASTLED_RP2040_CLOCKLESS_M0_FALLBACK
            showRGBBlocking(pixels);
#endif
            return;
        }

        // wait for past transfer to finish
        // call when previous pixels are done will run without blocking,
        // call when previous pixels are still being transmitted should block until complete

        // a potential improvement here would be to prepare data for the output before waiting,
        // but that would require a smarter DMA buffer system
        // (currently, the gap between LEDs is greater than 50us due to the time taken)
        if (dma_channel_is_busy(ppi->dma_channel)) {
            dma_channel_wait_for_finish_blocking(ppi->dma_channel);
        }
        mWait.wait();

        // Reset the PIO state machine before starting new transfer to prevent freeze 
        //This clears any stale state from the previous transfer (RP2350 fix)
        pio_sm_set_enabled(ppi->mPio, ppi->mSm, false);
        pio_sm_clear_fifos(ppi->mPio, ppi->mSm);
        pio_sm_restart(ppi->mPio, ppi->mSm);
        pio_sm_exec(ppi->mPio, ppi->mSm, pio_encode_jmp(ppi->mPioOffset));  // Jump back to program start
        pio_sm_set_enabled(ppi->mPio, ppi->mSm, true);

        showRGBInternal(pixels);
#else
        mWait.wait();
        showRGBBlocking(pixels);
        mWait.mark();
#endif
    }

#if FASTLED_RP2040_CLOCKLESS_PIO
    void showRGBInternal(PixelController<RGB_ORDER> pixels) FL_NO_EXCEPT {
        // Detect RGBW mode using pattern from ESP32 drivers
        const Rgbw rgbw = this->getRgbw();
        const bool is_rgbw = rgbw.active();
        const int bytes_per_pixel = is_rgbw ? 4 : 3;

        // Calculate buffer size based on pixel format (RGB = 3 bytes, RGBW = 4
        // bytes)
        size_t req_buf_size = (pixels.mLen * bytes_per_pixel * (8 + XTRA0) + 31) / 32;

        // (re)allocate DMA buffer if not large enough to hold req_buf_size 32-bit words
        //pico has enough memory to not really care about using a buffer for DMA
        // just give up on failure
        if (ppi->dma_buf_size < req_buf_size) {
            if (ppi->dma_buf != nullptr)
                fl::free(ppi->dma_buf);

            ppi->dma_buf = fl::malloc(req_buf_size * 4);
            if (ppi->dma_buf == nullptr) {
                ppi->dma_buf_size = 0;
                return;
            }
            ppi->dma_buf_size = req_buf_size;

            // fill with zeroes to ensure XTRA0s are really zero without needing
            // extra work
            fl::memset(ppi->dma_buf, 0, ppi->dma_buf_size * 4);
        }

        u32 bitpos = 0;

        // Separate loops for RGB vs RGBW to match ESP32 pattern
        if (is_rgbw) {
            // RGBW mode: process 4 bytes per pixel
            pixels.preStepFirstByteDithering();

            while(pixels.has(1)) {
                pixels.stepDithering();

                // Load all 4 channels (R, G, B, W) with proper color adjustment
                // and RGBW conversion
                u8 b0, b1, b2, b3;
                pixels.loadAndScaleRGBW(rgbw, &b0, &b1, &b2, &b3);

                // Write all 4 bytes to buffer
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b0);
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b1);
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b2);
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b3);

                pixels.advanceData();
            };
        } else {
            // RGB mode: process 3 bytes per pixel (original logic)
            pixels.preStepFirstByteDithering();
            u8 b = pixels.loadAndScale0();

            while(pixels.has(1)) {
                pixels.stepDithering();

                // Write first byte, read next byte
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b);
                b = pixels.loadAndScale1();

                // Write second byte, read 3rd byte
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b);
                b = pixels.loadAndScale2();

                // Write third byte, read 1st byte of next pixel
                bitpos += writeBitsToBuf<8 + XTRA0>((i32*)(ppi->dma_buf), bitpos, b);
                b = pixels.advanceAndLoadAndScale0();
            };
        }

        do_dma_transfer(ppi->dma_channel, ppi->dma_buf,
                        ppi->dma_buf_size);
    }
#endif  // FASTLED_RP2040_CLOCKLESS_PIO

#if FASTLED_RP2040_CLOCKLESS_M0_FALLBACK
    void showRGBBlocking(PixelController<RGB_ORDER> pixels) FL_NO_EXCEPT {
        struct M0ClocklessData data;
        data.d[0] = pixels.d[0];
        data.d[1] = pixels.d[1];
        data.d[2] = pixels.d[2];
        data.s[0] = pixels.mColorAdjustment.premixed[0];
        data.s[1] = pixels.mColorAdjustment.premixed[1];
        data.s[2] = pixels.mColorAdjustment.premixed[2];
        data.e[0] = pixels.e[0];
        data.e[1] = pixels.e[1];
        data.e[2] = pixels.e[2];
        data.adj = pixels.mAdvance;

        typedef FastPin<DATA_PIN> pin;
        volatile u32* portBase = &sio_hw->gpio_out;
        const int portSetOff = (u32)&sio_hw->gpio_set - (u32)&sio_hw->gpio_out;
        const int portClrOff = (u32)&sio_hw->gpio_clr - (u32)&sio_hw->gpio_out;

        cli();
        showLedData<portSetOff, portClrOff, TIMING, RGB_ORDER, WAIT_TIME>(
            portBase, pin::mask(), pixels.mData, pixels.mLen, &data);
        sei();
    }
#endif
};
}  // namespace fl
#endif  // __INC_CLOCKLESS_RP_PIO_COMMON
