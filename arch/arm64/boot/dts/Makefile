# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_MACH_EXYNOS9810_CROWNLTE_EUR_OPEN) := exynos/exynos9810-crownlte_eur_open_26.dtb
dtb-$(CONFIG_MACH_EXYNOS9810_STARLTE_EUR_OPEN) := exynos/exynos9810-starlte_eur_open_26.dtb
dtb-$(CONFIG_MACH_EXYNOS9810_STAR2LTE_EUR_OPEN) := exynos/exynos9810-star2lte_eur_open_26.dtb

dtb-$(CONFIG_MACH_EXYNOS9810_CROWNLTE_KOR) := exynos/exynos9810-crownlte_kor_26.dtb
dtb-$(CONFIG_MACH_EXYNOS9810_STARLTE_KOR) := exynos/exynos9810-starlte_kor_26.dtb
dtb-$(CONFIG_MACH_EXYNOS9810_STAR2LTE_KOR) := exynos/exynos9810-star2lte_kor_26.dtb

targets += dtbs
DTB_LIST  := $(dtb-y) $(dtbo-y)
always    := $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb*
