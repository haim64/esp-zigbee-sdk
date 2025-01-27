/*
 * Solara device
 */

#include "esp_zigbee_core.h"
#include "light_driver.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define HA_SOLARA_LIGHT_ENDPOINT           21    
#define HA_SOLARA_SWITCH_ENDPOINT          22    
#define HA_SOLARA_LOCK_ENDPOINT            23    
#define HA_SOLARA_ONOFF_ENDPOINT           24    
#define HA_SOLARA_SHADE_ENDPOINT           25    
#define HA_SOLARA_TEMP_ENDPOINT            26   

#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */
#define ESP_ZB_SECONDARY_CHANNEL_MASK   (1l << 13)  /* Zigbee primary channel mask use in the example */

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""SOLSOLSOL"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07""SOLSOLA" /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
