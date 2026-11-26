/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * Copyright 2021-2026 Analog Devices Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef RPMSG_PLATFORM_H_
#define RPMSG_PLATFORM_H_

#include <stdint.h>
#include "rpmsg_default_config.h"

/*
 * Linux requires the ALIGN to 0x1000(4KB) instead of 0x80
 */
#ifndef VRING_ALIGN
#define VRING_ALIGN (0x1000U)
#endif

/* contains pool of descriptors and two circular buffers */
#ifndef VRING_SIZE
#define VRING_SIZE (0x8000UL)
#endif

/* define shared memory space for VRINGS per one channel */
#define RL_VRING_OVERHEAD (2UL * VRING_SIZE)

/* Maximum Number of ISR Count. It is determined by the VQ_ID bit field size. */
#ifndef RL_PLATFORM_MAX_ISR_COUNT
#define RL_PLATFORM_MAX_ISR_COUNT (32U)
#endif

#define RL_GET_VQ_ID(link_id, queue_id) (((queue_id)&0x1U) | (((link_id) << 1U) & 0xFFFFFFFEU))
#define RL_GET_LINK_ID(id)              (((id)&0xFFFFFFFEU) >> 1U)
#define RL_GET_Q_ID(id)                 ((id)&0x1U)

/* Link IDs to use on ARM core */
#define RL_PLATFORM_ARM_SHARC0_LINK_ID (0U)
#define RL_PLATFORM_ARM_SHARC1_LINK_ID (1U)

/* Link IDs to use on a SHARC core */
#define RL_PLATFORM_SHARC_ARM_LINK_ID (0U)
#define RL_PLATFORM_SHARC_SHARC_LINK_ID (1U)

#define RL_PLATFORM_HIGHEST_LINK_ID (1U)

/* platform interrupt related functions */
int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data);
int32_t platform_deinit_interrupt(uint32_t vector_id);
int32_t platform_interrupt_enable(uint32_t vector_id);
int32_t platform_interrupt_disable(uint32_t vector_id);
int32_t platform_in_isr(void);
void platform_notify(uint32_t vector_id);

/* platform low-level time-delay (busy loop) */
void platform_time_delay(uint32_t num_msec);

/* returns 1MHz clock tick */
uint32_t platform_us_clock_tick(void);

/* platform memory functions */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags);
void platform_cache_all_flush_invalidate(void);
void platform_cache_disable(void);
#if defined(RL_USE_DCACHE) && (RL_USE_DCACHE == 1)
void platform_cache_invalidate(void *data, uint32_t len);
void platform_cache_flush(void *data, uint32_t len);
#endif /* defined(RL_USE_DCACHE) && (RL_USE_DCACHE == 1) */
uintptr_t platform_vatopa(void *addr);
void *platform_patova(uintptr_t addr);

/* platform init/deinit */
int32_t platform_init(void);
int32_t platform_deinit(void);

#endif /* RPMSG_PLATFORM_H_ */
