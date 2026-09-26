/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Hubert Denkmair
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

#include "usbd_def.h"

#define USB_CAN_CONFIG_DESC_SIZ	 (50U + CDC_CONFIG_DESC_SIZE)
#define USBD_GS_CAN_VENDOR_CODE	 0x20

#define COMPOSITE_DEVICE_NUM 3

typedef struct {
    USBD_ClassTypeDef *pClass[COMPOSITE_DEVICE_NUM];
} USBD_COMPOSITE_HandleTypeDef __attribute__ ((aligned (4)));

extern USBD_ClassTypeDef USBD_COMPOSITE;

#if defined(STM32F4)
// RX FIFO is defined in words, so divide bytes by 4
// RX FIFO size chosen according to reference manual RM0368 which suggests
// using (largest packet size / 4) + 1
# define USB_RX_FIFO_SIZE ((256U / 4U) + 1U)
#endif
