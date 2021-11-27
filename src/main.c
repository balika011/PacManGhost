/*
 * Copyright (c) 2021 balika011
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_ADC1);

	rcc_periph_clock_enable(RCC_TIM3);
}

static void systick_setup(void)
{
	systick_set_reload(72000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	systick_interrupt_enable();
}

volatile uint32_t system_millis;
void sys_tick_handler(void)
{
	system_millis++;
}

static void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis)
		;
}
static void adc_setup(void)
{
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

	adc_power_off(ADC1);

	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

static void timer_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,  GPIO_TIM3_CH1 | GPIO_TIM3_CH2);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH3 | GPIO_TIM3_CH4);

	TIM3_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	TIM3_ARR = 65535;
	TIM3_PSC = 0;
	TIM3_EGR = TIM_EGR_UG;

	TIM3_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

	TIM3_CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;

	TIM3_CCR1 = 0;

	TIM3_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	TIM3_CCER |= TIM_CCER_CC2P | TIM_CCER_CC2E;
	TIM3_CCR2 = 0;

	TIM3_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
	TIM3_CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;
	TIM3_CCR3 = 0;

	TIM3_CCMR2 |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
	TIM3_CCER |= TIM_CCER_CC4P | TIM_CCER_CC4E;
	TIM3_CCR4 = 0;

	TIM3_CR1 |= TIM_CR1_ARPE;

	TIM3_CR1 |= TIM_CR1_CEN;
}

static void switch_setup()
{
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	gpio_set(GPIOA, GPIO0);
}

static uint16_t read_adc(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_direct(ADC1);
	while (!adc_eoc(ADC1))
		;
	return adc_read_regular(ADC1);
}

static int is_party_mode()
{
	return gpio_get(GPIOA, GPIO0);
}

typedef struct
{
	double r;
	double g;
	double b;
} rgb;

typedef struct
{
	double h;
	double s;
	double v;
} hsv;

static rgb hsv2rgb(hsv in)
{
	double hh, p, q, t, ff;
	long i;
	rgb out;

	hh = in.h;
	if (hh >= 360.0)
		hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch (i)
	{
	case 0:
		out.r = in.v;
		out.g = t;
		out.b = p;
		break;
	case 1:
		out.r = q;
		out.g = in.v;
		out.b = p;
		break;
	case 2:
		out.r = p;
		out.g = in.v;
		out.b = t;
		break;

	case 3:
		out.r = p;
		out.g = q;
		out.b = in.v;
		break;
	case 4:
		out.r = t;
		out.g = p;
		out.b = in.v;
		break;
	case 5:
	default:
		out.r = in.v;
		out.g = p;
		out.b = q;
		break;
	}
	return out;
}

static uint16_t read_potentiometer()
{
	return read_adc(ADC_CHANNEL9);
}

int main(void)
{
	clock_setup();
	systick_setup();
	timer_setup();
	switch_setup();
	adc_setup();

	if (is_party_mode())
	{
		while (1)
		{
			uint16_t adc = read_potentiometer();
			for (double hue = 0.f; hue < 360.f; hue += 1.f)
			{
				hsv in;
				in.h = hue;
				in.s = 1.f;
				in.v = 1.f;
				rgb out = hsv2rgb(in);

				TIM3_CCR1 = 65535 * out.r; // red
				TIM3_CCR2 = 65535 * out.g; // green
				TIM3_CCR3 = 65535 * out.b; // blue

				/* 0 - 4096 */
				if ((int)hue % 10 == 0)
					adc = read_potentiometer();
				msleep((adc / 100) + 1);
			}
		}
	}
	else
	{
		while (1)
		{
			uint16_t adc = read_potentiometer();
			hsv in;
			in.h = (360.f * adc) / 4096.f;
			in.s = 1.f;
			in.v = 1.f;
			rgb out = hsv2rgb(in);

			TIM3_CCR1 = 65535 * out.r; // red
			TIM3_CCR2 = 65535 * out.g; // green
			TIM3_CCR3 = 65535 * out.b; // blue
			msleep(10);
		}
	}

	return 0;
}
