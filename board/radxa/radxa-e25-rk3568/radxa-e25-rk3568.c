// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019 Rockchip Electronics Co., Ltd
 * (C) Copyright 2023 Collabora Ltd
 * (C) Copyright 2024 Radxa Ltd.
 */

#include <common.h>
#include <adc.h>

#define ADC_CHANNEL             0
#define KEY_DOWN_MIN_VAL        0
#define KEY_DOWN_MAX_VAL        80

int rockchip_dnl_key_pressed(void)
{
	unsigned int val;

	if (adc_channel_single_shot("saradc@fe720000", ADC_CHANNEL, &val)) {
		printf("%s read adc key val failed\n", __func__);
		return false;
	}

	printf("Recovery ADC %i read as %i\n", ADC_CHANNEL, val);
	return (val >= KEY_DOWN_MIN_VAL && val <= KEY_DOWN_MAX_VAL);
}
