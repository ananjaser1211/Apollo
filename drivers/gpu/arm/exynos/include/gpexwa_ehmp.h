// SPDX-License-Identifier: GPL-2.0

/*
 * (C) COPYRIGHT 2021 Samsung Electronics Inc. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#ifndef _GPEXWA_EHMP_H_
#define _GPEXWA_EHMP_H_

#if IS_ENABLED(CONFIG_MALI_EXYNOS_EHMP)
#include <linux/types.h>

void gpexwa_ehmp_set(void);
void gpexwa_ehmp_unset(void);
bool gpexwa_ehmp_skip_ifpo_power_down(void);
int gpexwa_ehmp_init(void);
void gpexwa_ehmp_term(void);
#else
#define gpexwa_ehmp_set(...) (void)0
#define gpexwa_ehmp_unset(...) (void)0
#define gpexwa_ehmp_skip_ifpo_power_down(...) 0
#define gpexwa_ehmp_init(...)
#define gpexwa_ehmp_term(...) (void)0
#endif

#endif /* _GPEXWA_EHMP_H_ */
