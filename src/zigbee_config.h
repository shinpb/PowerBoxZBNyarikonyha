#pragma once
// Zigbee stack / platform configuration builders (used by zigbee_manager.cpp only).
// The values come from Config.Zigbee (config.h): channel masks, storage partition, max children.
#include "sdkconfig.h"   // CONFIG_SOC_IEEE802154_SUPPORTED
#include "config.h"
#include "esp_zigbee.h"

// ZED: end device (kept for a possible battery variant, D1: the device runs as a router)
#define ESP_ZIGBEE_ZED_CONFIG()                         \
    {                                                   \
        .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,  \
        .install_code_policy = false,                   \
        .zed_config = {                                 \
            .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,     \
            .keep_alive = 4000,                         \
        },                                              \
    }

// ZR: router
#define ESP_ZIGBEE_ZR_CONFIG()                                         \
    {                                                                  \
        .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,                     \
        .install_code_policy = false,                                  \
        .zczr_config = { .max_children = Config.Zigbee.MaxChildren },  \
    }

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = Config.Zigbee.StoragePartition,    \
        .radio_config = {                                            \
            .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,              \
        },                                                           \
    }
#else
#warning "The example is not for IEEE 802.15.4-disabled SoC usage, please refer to esp_zigbee_gateway for RCP configuration"
#endif

// The device role: ESP_ZIGBEE_ZR_CONFIG() or ESP_ZIGBEE_ZED_CONFIG()
#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ZR_CONFIG(),         \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    };
