// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/platform/bus.h>
#include <soc/aml-s905d2/s905d2-gpio.h>
#include <soc/aml-s905d2/s905d2-hiu.h>
#include <soc/aml-s905d2/s905d2-hw.h>

#include <limits.h>
#include "astro.h"
#include "astro-gpios.h"

//clang-format off

static const pbus_mmio_t audio_mmios[] = {
    {
        .base = S905D2_EE_AUDIO_BASE,
        .length = S905D2_EE_AUDIO_LENGTH
    },
};

static const pbus_bti_t tdm_btis[] = {
    {
        .iommu_index = 0,
        .bti_id = BTI_AUDIO_OUT,
    },
};

static pbus_dev_t aml_tdm_dev = {
    .name = "AstroAudio",
    .vid = PDEV_VID_AMLOGIC,
    .pid = PDEV_PID_AMLOGIC_S905D2,
    .did = PDEV_DID_AMLOGIC_TDM,
    .mmio_list = audio_mmios,
    .mmio_count = countof(audio_mmios),
    .bti_list = tdm_btis,
    .bti_count = countof(tdm_btis),
};

static const zx_bind_inst_t root_match[] = {
    BI_MATCH(),
};
static const zx_bind_inst_t i2c_match[] = {
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_I2C),
    BI_ABORT_IF(NE, BIND_I2C_BUS_ID, ASTRO_I2C_3),
    BI_MATCH_IF(EQ, BIND_I2C_ADDRESS, I2C_AUDIO_CODEC_ADDR),
};
static const zx_bind_inst_t fault_gpio_match[] = {
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_GPIO),
    BI_MATCH_IF(EQ, BIND_GPIO_PIN, GPIO_AUDIO_SOC_FAULT_L),
};
static const zx_bind_inst_t enable_gpio_match[] = {
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_GPIO),
    BI_MATCH_IF(EQ, BIND_GPIO_PIN, GPIO_SOC_AUDIO_EN),
};
static const device_component_part_t i2c_component[] = {
    { countof(root_match), root_match },
    { countof(i2c_match), i2c_match },
};
static const device_component_part_t fault_gpio_component[] = {
    { countof(root_match), root_match },
    { countof(fault_gpio_match), fault_gpio_match },
};
static const device_component_part_t enable_gpio_component[] = {
    { countof(root_match), root_match },
    { countof(enable_gpio_match), enable_gpio_match },
};
static const device_component_t components[] = {
    { countof(i2c_component), i2c_component },
    { countof(fault_gpio_component), fault_gpio_component },
    { countof(enable_gpio_component), enable_gpio_component },
};

//PDM input configurations
static const pbus_mmio_t pdm_mmios[] = {
    {
        .base = S905D2_EE_PDM_BASE,
        .length = S905D2_EE_PDM_LENGTH
    },
    {
        .base = S905D2_EE_AUDIO_BASE,
        .length = S905D2_EE_AUDIO_LENGTH
    },
};

static const pbus_bti_t pdm_btis[] = {
    {
        .iommu_index = 0,
        .bti_id = BTI_AUDIO_IN,
    },
};

static const pbus_dev_t aml_pdm_dev = {
    .name = "gauss-audio-in",
    .vid = PDEV_VID_AMLOGIC,
    .pid = PDEV_PID_AMLOGIC_S905D2,
    .did = PDEV_DID_ASTRO_PDM,
    .mmio_list = pdm_mmios,
    .mmio_count = countof(pdm_mmios),
    .bti_list = pdm_btis,
    .bti_count = countof(pdm_btis),
};

//clang-format on

zx_status_t astro_tdm_init(aml_bus_t* bus) {

    aml_hiu_dev_t hiu;
    zx_status_t status = s905d2_hiu_init(&hiu);
    if (status != ZX_OK) {
        zxlogf(ERROR, "hiu_init: failed: %d\n", status);
        return status;
    }

    aml_pll_dev_t hifi_pll;
    s905d2_pll_init(&hiu, &hifi_pll, HIFI_PLL);
    status = s905d2_pll_set_rate(&hifi_pll, 1536000000);
    if (status != ZX_OK) {
        zxlogf(ERROR,"Invalid rate selected for hifipll\n");
        return status;
    }

    s905d2_pll_ena(&hifi_pll);

    // TDM pin assignments
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(1), S905D2_GPIOA_1_TDMB_SCLK_FN);
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(2), S905D2_GPIOA_2_TDMB_FS_FN);
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(3), S905D2_GPIOA_3_TDMB_D0_FN);
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(6), S905D2_GPIOA_6_TDMB_DIN3_FN);

    // PDM pin assignments
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(7), S905D2_GPIOA_7_PDM_DCLK_FN);
    gpio_impl_set_alt_function(&bus->gpio, S905D2_GPIOA(8), S905D2_GPIOA_8_PDM_DIN0_FN);

    gpio_impl_config_out(&bus->gpio, S905D2_GPIOA(5), 1);

    status = pbus_composite_device_add(&bus->pbus, &aml_tdm_dev, components, countof(components),
                                       UINT32_MAX);
    if (status != ZX_OK) {
        zxlogf(ERROR, "astro_tdm_init: pbus_device_add failed: %d\n", status);
        return status;
    }

    status = pbus_device_add(&bus->pbus, &aml_pdm_dev);
    if (status != ZX_OK) {
        zxlogf(ERROR, "astro_tdm_init: pbus_device_add failed: %d\n", status);
        return status;
    }

    return ZX_OK;
}
