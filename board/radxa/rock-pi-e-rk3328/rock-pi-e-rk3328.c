// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2015 Rockchip Electronics Co., Ltd
 * (C) Copyright 2023 Radxa Ltd.
 */

#include <common.h>
#include <adc.h>

#define ADC_CHANNEL				1
#define KEY_DOWN_MIN_VAL        320
#define KEY_DOWN_MAX_VAL        400

int rockchip_dnl_key_pressed(void)
{
	unsigned int val;

	if (adc_channel_single_shot("adc@ff280000", ADC_CHANNEL, &val)) {
		printf("%s read adc key val failed\n", __func__);
		return false;
	}

	printf("Recovery ADC %i read as %i\n", ADC_CHANNEL, val);
	return (val >= KEY_DOWN_MIN_VAL && val <= KEY_DOWN_MAX_VAL);
}
