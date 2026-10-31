#pragma once
#include <stdint.h>

// Diagnostics on the board endpoint (EP 0x01), see PLAN.md 4.8:
//  - standard Binary Input cluster (0x000F): PresentValue = 1 when any sensor/bus fault is active,
//    StatusFlags FAULT bit follows it
//  - manufacturer-specific cluster ZB_DIAG_CLUSTER_ID, attribute ZB_DIAG_ATTR_FAULT_MASK (bitmap32):
//    health_zb_mask() (sensor bits 0..15, bus bits 16..23)
static constexpr uint16_t ZB_DIAG_CLUSTER_ID      = 0xFC11;
static constexpr uint16_t ZB_DIAG_ATTR_FAULT_MASK = 0x0000;

void init_zb_stack();
void button_task(void*);

//ZB Global flag
extern volatile bool is_zigbee_connected;



#define ESP_ZIGBEE_PRIMARY_CHANNEL_MASK   (1U << CONFIG_ZB_EXAMPLE_PRIMARY_CHANNEL)
#define ESP_ZIGBEE_SECONDARY_CHANNEL_MASK CONFIG_ZB_EXAMPLE_SECONDARY_CHANNEL_MASK

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "nvs"

#define ESP_ZIGBEE_ZED_CONFIG()                         \
    {                                                   \
        .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,  \
        .install_code_policy = false,                   \
        .zed_config = {                                 \
            .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,     \
            .keep_alive = 4000,                         \
        },                                              \
    }

#define ESP_ZIGBEE_ZR_CONFIG()                          \
{                                                   \
    .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,      \
    .install_code_policy = false,                   \
    .zczr_config = { .max_children = 10 },          \
}

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define ESP_ZIGBEE_PLATFORM_CONFIG()                                 \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
        .radio_config = {                                            \
            .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,              \
        },                                                           \
    }
#else
#warning "The example is not for IEEE 802.15.4-disabled SoC usage, please refer to esp_zigbee_gateway for RCP configuration"
#endif

//        .device_config = ESP_ZIGBEE_ZED_CONFIG(),
#define ESP_ZIGBEE_DEFAULT_CONFIG()                      \
    {                                                    \
        .device_config = ESP_ZIGBEE_ZR_CONFIG(),         \
        .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
    };
