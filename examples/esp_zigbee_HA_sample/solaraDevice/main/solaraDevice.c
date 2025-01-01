/*
 * Solara device c file
 */
#include "solaraDevice.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "string.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static float zb_s16_to_temperature(int16_t value)
{
    return 1.0 * value / 100;
}

typedef struct zbstring_s {
    uint8_t len;
    char data[];
} ESP_ZB_PACKED_STRUCT
zbstring_t;

static const char *TAG = "SOLARA_DEVICE";

/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{
    light_driver_init(LIGHT_DEFAULT_OFF);
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t* attribute)
{
    /* Basic cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Manufacturer is \"%s\"", string);
            free(string);
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING &&
            attribute->data.value) {
            zbstring_t *zbstr = (zbstring_t *)attribute->data.value;
            char *string = (char*)malloc(zbstr->len + 1);
            memcpy(string, zbstr->data, zbstr->len);
            string[zbstr->len] = '\0';
            ESP_LOGI(TAG, "Peer Model is \"%s\"", string);
            free(string);
        }
    }

    /* Temperature Measurement cluster attributes */
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t min_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Min Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(min_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            int16_t max_value = attribute->data.value ? *(int16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Max Measured Value is %.2f degrees Celsius", zb_s16_to_temperature(max_value));
        }
        if (attribute->id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_ID &&
            attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
            uint16_t tolerance = attribute->data.value ? *(uint16_t *)attribute->data.value : 0;
            ESP_LOGI(TAG, "Tolerance is %.2f degrees Celsius", 1.0 * tolerance / 100);
        }
    }
}


static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d) , data type(%d) ", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size, message->attribute.data.type);
    if (message->info.dst_endpoint == HA_SOLARA_SHADE_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                light_driver_set_power(light_state);
            }
        }
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
            {
                int volume = message->attribute.data.value  ? *(int *)message->attribute.data.value : -1;
                ESP_LOGI(TAG, "Level sets to %d", volume);
            }             
        }
    }
    return ret;
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                    variable->status, message->info.cluster,
                    variable->attribute.id, variable->attribute.data.type,
                    variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}


static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
	    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void* pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

 
	
/*	
    esp_zb_ep_list_add_ep(endpoint_list, switch_cluster_list, 1, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID);
    esp_zb_ep_list_add_ep(endpoint_list, light_cluster_list, 2, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID);
    esp_zb_ep_list_add_ep(endpoint_list, door_lock_cluster_list, 3, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_DOOR_LOCK_DEVICE_ID);
    esp_zb_ep_list_add_ep(endpoint_list, shade_cluster_list, 4, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SHADE_DEVICE_ID);
    esp_zb_ep_list_add_ep(endpoint_list, temperature_sensor_cluster_list, 5, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID);
*/	
    esp_zb_ep_list_t* endpoint_list = esp_zb_ep_list_create();


	esp_zb_endpoint_config_t endpoint_config = {
	.endpoint = HA_SOLARA_SWITCH_ENDPOINT,
	.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
	.app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID, 
	.app_device_version = 0};

	endpoint_config.endpoint = HA_SOLARA_SHADE_ENDPOINT;
	endpoint_config.app_device_id = ESP_ZB_HA_SHADE_DEVICE_ID;
    //esp_zb_shade_cfg_t shade_config = ESP_ZB_DEFAULT_SHADE_CONFIG();

    /* set the on-off light device config */
    uint8_t test_attr, test_attr2;

    test_attr = 0;
    test_attr2 = 4;

#if ALT_CODE    

    /* basic cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr2);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);
    /* group cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);
    /* scenes cluster create with standard cluster + customized */
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);
#endif

    // shade attr list
    esp_zb_shade_config_cluster_cfg_t shade_config_cfg;
    shade_config_cfg.closed_limit = ESP_ZB_ZCL_SHADE_CONFIG_CLOSED_LIMIT_DEFAULT_VALUE;
    shade_config_cfg.mode = ESP_ZB_ZCL_SHADE_CONFIG_MODE_DEFAULT_VALUE;
    shade_config_cfg.status = ESP_ZB_ZCL_SHADE_CONFIG_STATUS_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_shade_cluster = esp_zb_shade_config_cluster_create(&shade_config_cfg);

    esp_zb_level_cluster_cfg_t level_cfg;
    level_cfg.current_level = 50;
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_level_cluster_create(&level_cfg);

    /* on-off cluster create with standard cluster config*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);


    /* group cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);

    /* scenes cluster create with standard cluster + customized */
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);

    esp_zb_cluster_list_t *shade_cluster_list = esp_zb_zcl_cluster_list_create();
    
    //esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(shade_config.basic_cfg));
    esp_zb_attribute_list_t *basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);


// basic
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr2));
    ESP_ERROR_CHECK(esp_zb_cluster_update_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(shade_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
// identify
    //ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(shade_cluster_list, esp_zb_identify_cluster_create(&(shade_config.identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(shade_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(shade_cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
// groups 
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_groups_cluster(shade_cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
// scenes
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(shade_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    // onoff
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(shade_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    // level
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(shade_cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
// shade
    //ESP_ERROR_CHECK(esp_zb_cluster_list_add_shade_config_cluster(shade_cluster_list, esp_zb_shade_clusters_create(&(shade_config.shade_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_shade_config_cluster(shade_cluster_list, esp_zb_shade_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    
    /* Add temperature measurement cluster for attribute reporting */
    //ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(shade_cluster_list, esp_zb_temperature_meas_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));


	esp_zb_ep_list_add_ep(endpoint_list, shade_cluster_list, endpoint_config);
/*
    endpoint_config.endpoint = HA_SOLARA_SWITCH_ENDPOINT;
    esp_zb_on_off_switch_cfg_t switch_config = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_cluster_list_t* switch_cluster_list = esp_zb_on_off_switch_clusters_create(&switch_config);
	esp_zb_ep_list_add_ep(endpoint_list, switch_cluster_list, endpoint_config);
	
    endpoint_config.endpoint = HA_SOLARA_LIGHT_ENDPOINT;
    esp_zb_on_off_light_cfg_t light_config = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_cluster_list_t* light_cluster_list = esp_zb_on_off_light_clusters_create(&light_config);
	endpoint_config.app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID;
	esp_zb_ep_list_add_ep(endpoint_list, light_cluster_list, endpoint_config);
	    
    endpoint_config.endpoint = HA_SOLARA_LOCK_ENDPOINT;
	endpoint_config.app_device_id = ESP_ZB_HA_DOOR_LOCK_DEVICE_ID;
    esp_zb_door_lock_cfg_t door_lock_config = ESP_ZB_DEFAULT_DOOR_LOCK_CONFIG();
    esp_zb_cluster_list_t* door_lock_cluster_list = esp_zb_door_lock_clusters_create(&door_lock_config);
	esp_zb_ep_list_add_ep(endpoint_list, door_lock_cluster_list, endpoint_config);

	endpoint_config.endpoint = HA_SOLARA_TEMP_ENDPOINT;
	endpoint_config.app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID;
    esp_zb_temperature_sensor_cfg_t temperature_sensor_config = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    esp_zb_cluster_list_t* temperature_sensor_cluster_list = esp_zb_temperature_sensor_clusters_create(&temperature_sensor_config);
	esp_zb_ep_list_add_ep(endpoint_list, temperature_sensor_cluster_list, endpoint_config);
*/	

/** 
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };
    ESP_LOGI(TAG, "Set names %s %s ", ESP_MANUFACTURER_NAME, ESP_MODEL_IDENTIFIER);
    esp_zcl_utility_add_ep_basic_manufacturer_info(endpoint_list, ESP_ZB_HA_SHADE_DEVICE_ID, &info);
**/

    /*
    esp_zcl_utility_add_ep_basic_manufacturer_info(endpoint_list, HA_SOLARA_SWITCH_ENDPOINT &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(endpoint_list, HA_SOLARA_LIGHT_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(endpoint_list, HA_SOLARA_LOCK_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(endpoint_list, HA_SOLARA_TEMP_ENDPOINT, &info);
	*/

    esp_zb_device_register(endpoint_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
	esp_zb_set_secondary_network_channel_set(ESP_ZB_SECONDARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop(); 
}


void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}