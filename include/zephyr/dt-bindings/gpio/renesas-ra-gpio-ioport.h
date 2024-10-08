/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RENESAS_RA_GPIO_IOPORT_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RENESAS_RA_GPIO_IOPORT_H_

#define RENESAS_GPIO_DS_POS                   (8)
#define RENESAS_GPIO_DS_MSK                   (0x3U << RENESAS_GPIO_DS_POS)
/*  GPIO Drive strength */
#define RENESAS_GPIO_DS_LOW                   (0x0 << RENESAS_GPIO_DRIVE_POS)
#define RENESAS_GPIO_DS_MIDDLE                (0x1 << RENESAS_GPIO_DRIVE_POS)
#define RENESAS_GPIO_DS_HIGH_SPEED_HIGH_DRIVE (0x2 << RENESAS_GPIO_DRIVE_POS)
#define RENESAS_GPIO_DS_HIGH_DRIVE            (0x3 << RENESAS_GPIO_DRIVE_POS)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RENESAS_RA_GPIO_IOPORT_H_ */
