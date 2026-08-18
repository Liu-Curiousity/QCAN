/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "board.h"
#include "can_common.h"
#include "config.h"
#include "gpio_.h"

/*=========================是否支持软件设置终端电阻=========================*/
#ifdef CONFIG_TERMINATION
static int term_state = 0;

enum gs_can_termination_state get_term(can_data_t *channel)
{
	const uint8_t nr = can_channel_get_nr(channel);

	if (term_state & (1 << nr)) {
		return GS_CAN_TERMINATION_STATE_ON;
	} else {
		return GS_CAN_TERMINATION_STATE_OFF;
	}
}

void set_term(can_data_t *channel, enum gs_can_termination_state state)
{
	const uint8_t nr = can_channel_get_nr(channel);

	if (state == GS_CAN_TERMINATION_STATE_ON) {
		term_state |= 1 << nr;
	} else {
		term_state &= ~(1 << nr);
	}

	board_termination_set(channel, state);
}

static inline void gpio_init_term(void)
{
	HAL_GPIO_WritePin(TERM_GPIO_Port, TERM_Pin, GPIO_INIT_STATE(TERM_Active_High));

	GPIO_InitTypeDef GPIO_InitStruct = {
		.Pin = TERM_Pin,
		.Mode = TERM_Mode,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_LOW,
		.Alternate = 0
	};
	HAL_GPIO_Init(TERM_GPIO_Port, &GPIO_InitStruct);
}

#else

static inline void gpio_init_term(void)
{
}

#endif
