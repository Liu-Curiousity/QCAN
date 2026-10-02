/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016, 2019 Hubert Denkmair
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stm32f072xb.h>

// ARM's
// "Application Note 321 ARM Cortex-M Programming Guide to Memory Barrier Instructions"
// (from https://developer.arm.com/documentation/dai0321/latest) says that
// the ISBs are actually necessary on Cortex-M0 to avoid a 2-instruction
// delay in the effect of enabling and disabling interrupts.
// That probably doesn't matter here, but it's hard to say what the compiler
// will put in those 2 instructions so it's safer to leave it. The DSB isn't
// necessary on Cortex-M0, but it's architecturally required so we'll
// include it to be safe.
//
// The "memory" and "cc" clobbers tell GCC to avoid moving memory loads or
// stores across the instructions. This is important when an interrupt and the
// code calling disable_irq/enable_irq share memory. The fact that these are
// non-inlined functions probably forces GCC to flush everything to memory
// anyways, but trying to outsmart the compiler is a bad strategy (you never
// know when somebody will turn on LTO or something).

/** @brief Extract the Least Significant Bit from @p value. */
#define LSB_GET(value) ((value) & -(value))
/**
 * @brief Extract a bitfield element from @p value corresponding to
 *	  the field mask @p mask.
 */
#define FIELD_GET(mask, value) (((value) & (mask)) / LSB_GET(mask))
/**
 * @brief Prepare a bitfield element using @p value with @p mask representing
 *	  its field position and width. The result should be combined
 *	  with other fields using a logical OR.
 */
#define FIELD_PREP(mask, value) (((value) * LSB_GET(mask)) & (mask))

/* Then we append an extra argument to fool the gcc preprocessor into
 * accepting it as a varargs macro.
 *                         arg1   arg2  arg3
 *   ENABLED:   Z_IS_ENABLED3(_YYYY,    1,    0)
 *   DISABLED   Z_IS_ENABLED3(_XXXX 1,  0)
 */
#define Z_IS_ENABLED2(one_or_two_args) Z_IS_ENABLED3(one_or_two_args 1, 0)

/* And our second argument is thus now cooked to be 1 in the case
 * where the value is defined to 1, and 0 if not:
 */
#define Z_IS_ENABLED3(ignore_this, val, ...) val

#define IS_ENABLED(config_macro) Z_IS_ENABLED2(_XXXX ## config_macro)

static inline bool is_irq_enabled(void)
{
	return (__get_PRIMASK() & 1u) == 0u; // interrupts not prevented
}

static inline bool disable_irq(void)
{
	bool was_enabled = is_irq_enabled();
	__disable_irq();
	__ISB();
	return was_enabled;
}

static inline void enable_irq(void)
{
	__enable_irq();
	__ISB();
}

static inline void restore_irq(bool was_enabled)
{
	if (was_enabled) {
		enable_irq();
	}
}

#ifdef __cplusplus
}
#endif