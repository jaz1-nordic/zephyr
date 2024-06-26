/*
 * Copyright (c) 2024, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf_egpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/ipc/ipc_service.h>

#include <nrfe_gpio.h>

#ifdef CONFIG_MULTITHREADING
K_SEM_DEFINE(bound_sem, 0, 1);
#else
volatile uint32_t bound_sem = 1;
#endif

static void ep_bound(void *priv)
{
#ifdef CONFIG_MULTITHREADING
	k_sem_give(&bound_sem);
#else
	bound_sem = 0;
#endif
}

static void ep_recv(const void *data, size_t len, void *priv)
{
}

static struct ipc_ept_cfg ep_cfg = {
	.cb = {
		.bound = ep_bound,
		.received = ep_recv,
	},
};

struct ipc_ept ep;

struct gpio_nrfe_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
};

struct gpio_nrfe_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint8_t port_num;
};

static inline const struct gpio_nrfe_cfg *get_port_cfg(const struct device *port)
{
	return port->config;
}

static int gpio_nrfx_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	nrfe_gpio_data_packet_t msg = {.opcode = NRFE_GPIO_PIN_CONFIGURE,
				       .pin = pin,
				       .port = get_port_cfg(port)->port_num,
				       .flags = flags};

	int ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int gpio_nrfx_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{

	const uint32_t set_mask = value & mask;
	const uint32_t clear_mask = (~set_mask) & mask;

	nrfe_gpio_data_packet_t msg = {
		.opcode = NRFE_GPIO_PIN_SET, .pin = set_mask, .port = get_port_cfg(port)->port_num};

	int ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	msg.opcode = NRFE_GPIO_PIN_CLEAR;
	msg.pin = clear_mask;

	ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int gpio_nrfx_port_set_bits_raw(const struct device *port, gpio_port_pins_t mask)
{
	nrfe_gpio_data_packet_t msg = {
		.opcode = NRFE_GPIO_PIN_SET, .pin = mask, .port = get_port_cfg(port)->port_num};

	int ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int gpio_nrfx_port_clear_bits_raw(const struct device *port, gpio_port_pins_t mask)
{
	nrfe_gpio_data_packet_t msg = {
		.opcode = NRFE_GPIO_PIN_CLEAR, .pin = mask, .port = get_port_cfg(port)->port_num};

	int ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int gpio_nrfx_port_toggle_bits(const struct device *port, gpio_port_pins_t mask)
{
	nrfe_gpio_data_packet_t msg = {
		.opcode = NRFE_GPIO_PIN_TOGGLE, .pin = mask, .port = get_port_cfg(port)->port_num};

	int ret = ipc_service_send(&ep, (void *)(&msg), sizeof(nrfe_gpio_data_packet_t));

	if (ret < 0) {
		return ret;
	}

	return ret;
}

static int gpio_nrfe_init(const struct device *port)
{
	const struct device *ipc0_instance = DEVICE_DT_GET(DT_NODELABEL(ipc0));

	int ret = ipc_service_open_instance(ipc0_instance);

	if ((ret < 0) && (ret != -EALREADY)) {
		return ret;
	}

	ret = ipc_service_register_endpoint(ipc0_instance, &ep, &ep_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Wait for endpoint to be bound */
#ifdef CONFIG_MULTITHREADING
	k_sem_take(&bound_sem, K_FOREVER);
#else
	while (bound_sem != 0) {
	};
#endif

	return 0;
}

static const struct gpio_driver_api gpio_nrfx_drv_api_funcs = {
	.pin_configure = gpio_nrfx_pin_configure,
	.port_set_masked_raw = gpio_nrfx_port_set_masked_raw,
	.port_set_bits_raw = gpio_nrfx_port_set_bits_raw,
	.port_clear_bits_raw = gpio_nrfx_port_clear_bits_raw,
	.port_toggle_bits = gpio_nrfx_port_toggle_bits,
};

/* needs to be done after MBOX driver init, which is at
 * POST_KERNEL:KERNEL_INIT_PRIORITY_DEFAULT.
 */
#define CONFIG_EGPIO_INIT_PRIORITY CONFIG_KERNEL_INIT_PRIORITY_DEVICE

/* Device instantiation is done with node labels because 'port_num' is
 * the peripheral number by SoC numbering. We therefore cannot use
 * DT_INST APIs here without wider changes.
 */
#define EGPIO_NRF_DEVICE(id)                                                                       \
	static const struct gpio_nrfe_cfg gpio_nrfx_p##id##_cfg = {                                \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(id),              \
			},                                                                         \
		.port_num = DT_INST_PROP(id, port),                                                \
	};                                                                                         \
                                                                                                   \
	static struct gpio_nrfe_data gpio_nrfx_p##id##_data;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, gpio_nrfe_init, NULL, &gpio_nrfx_p##id##_data,                   \
			      &gpio_nrfx_p##id##_cfg, POST_KERNEL, CONFIG_EGPIO_INIT_PRIORITY,     \
			      &gpio_nrfx_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(EGPIO_NRF_DEVICE)
