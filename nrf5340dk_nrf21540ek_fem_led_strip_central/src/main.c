#include <stdio.h>
#include <string.h>
#include <math.h>

#include <zephyr/zephyr.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <bluetooth/services/lbs.h>
#include <dk_buttons_and_leds.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ble_central);

#define STRIP_NODE              DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS        DT_PROP(DT_ALIAS(led_strip), chain_length)
#define DELAY_TIME K_MSEC(20)
#define DEFAULT_SPEED 20
#define SPEED_INCREMENT 50
#define NUM_LED_STATES 4

static void start_scan();

static struct bt_conn *default_conn;
static struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

struct led_rgb pixels[STRIP_NUM_PIXELS];
static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static bool led_strip_on = true;
static int led_state = 1;
static int speed = DEFAULT_SPEED;

uint8_t GAMMA_8BIT[256] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
      2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
      6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11,
      11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
      19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
      29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
      40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
      71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
      90, 91, 93, 94, 95, 96, 98, 99, 100, 102, 103, 104, 106, 107, 109, 110,
      111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 128, 129, 131, 132, 134,
      135, 137, 138, 140, 142, 143, 145, 146, 148, 150, 151, 153, 155, 157, 158, 160,
      162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 179, 181, 183, 185, 187, 189,
      191, 193, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220,
      222, 224, 227, 229, 231, 233, 235, 237, 239, 241, 244, 246, 248, 250, 252, 255};

void set_rgb(uint32_t index, uint8_t r, uint8_t g, uint8_t b) {
    struct led_rgb color;
    color.r = GAMMA_8BIT[r];
    color.g = GAMMA_8BIT[g];
    color.b = GAMMA_8BIT[b];
    memcpy(&pixels[index], &color, sizeof(struct led_rgb));
}

void pixels_set_hsv(uint32_t index, float h, float s, float v)
{
    float i = floor(h * 6.0f);
    float f = h * 6.0f - i;
    v *= 255.0f;
    uint8_t p = v * (1.0f - s);
    uint8_t q = v * (1.0f - f * s);
    uint8_t t = v * (1.0f - (1.0f - f) * s);

    switch ((int)i % 6) {
      case 0: set_rgb(index, v, t, p); break;
      case 1: set_rgb(index, q, v, p); break;
      case 2: set_rgb(index, p, v, t); break;
      case 3: set_rgb(index, p, q, v); break;
      case 4: set_rgb(index, t, p, v); break;
      case 5: set_rgb(index, v, p, q); break;
    }
}

static uint8_t notify_func(struct bt_conn *conn,
               struct bt_gatt_subscribe_params *params,
               const void *payload, uint16_t length)
{
    if (!payload) {
        params->value_handle = 0U;
        LOG_INF("[UNSUBSCRIBED]\n");
        return BT_GATT_ITER_STOP;
    }

    led_state++;

    if (led_state > NUM_LED_STATES) {
        led_state = 0;
    }

    if (led_state == 0) {
        led_strip_on = false; 
    } else {
        led_strip_on = true; 
    }
    
    if (led_state == 1) { 
        speed = DEFAULT_SPEED;
    } else {
        speed += SPEED_INCREMENT;
    }
   
    LOG_INF("led_strip_on = %d\n", led_strip_on);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
                 const struct bt_gatt_attr *attr,
                 struct bt_gatt_discover_params *params)
{
    int err;

    if (!attr) {
        printk("Discover complete\n");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_INF("[ATTRIBUTE] handle %u\n", attr->handle);

    if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_LBS)) {
        memcpy(&uuid, BT_UUID_LBS_BUTTON, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)\n", err);
        }
    } else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_LBS_BUTTON)) {
        memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.start_handle = attr->handle + 2;
        discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)\n", err);
        }
    } else {
        subscribe_params.notify = notify_func;
        subscribe_params.value = BT_GATT_CCC_NOTIFY;
        subscribe_params.ccc_handle = attr->handle;

        err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err && err != -EALREADY) {
            LOG_ERR("Subscribe failed (err %d)\n", err);
        } else {
            LOG_INF("[SUBSCRIBED] %u\n",  attr->handle);
        }

        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_STOP;
}

static bool eir_found(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    LOG_INF("[AD]: %u data_len %u\n", data->type, data->data_len);

    if ( data->type == BT_DATA_NAME_COMPLETE) {
        struct bt_le_conn_param *param;
        uint8_t name[data->data_len+1];
        memcpy(name, data->data, data->data_len);
            name[data->data_len] = 0;

        if (strcmp(name, "Nordic_LBS")) {
            return true;
        }
            
        int err = bt_le_scan_stop();
        if (err) {
            LOG_ERR("Stop LE scan failed (err %d)\n", err);
            return true;
        }

        param = BT_LE_CONN_PARAM_DEFAULT;
        err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                    param, &default_conn);
        if (err) {
            LOG_ERR("Create conn failed (err %d)\n", err);
            start_scan();
        }
            
        LOG_INF("Found:  %s\n", name);

        return false;
    }
    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
             struct net_buf_simple *ad)
{
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    LOG_INF("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
           dev, type, ad->len, rssi);

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
        type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_data_parse(ad, eir_found, (void *)addr);
    }
}

static void start_scan()
{
    int err;

    /* Use active scanning and disable duplicate filtering to handle any
     * devices that might update their advertising data at runtime. */
    struct bt_le_scan_param scan_param = {
        .type       = BT_LE_SCAN_TYPE_ACTIVE,
        .options    = BT_LE_SCAN_OPT_NONE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)\n", err);
        return;
    }

    LOG_INF("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)\n", addr, conn_err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    LOG_INF("Connected: %s\n", addr);
    dk_set_led_on(DK_LED1);

    if (conn == default_conn) {
        LOG_INF("discovering services\n");

        memcpy(&uuid, BT_UUID_LBS, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.func = discover_func;
        discover_params.start_handle = 0x0001;
        discover_params.end_handle = 0xffff;
        discover_params.type = BT_GATT_DISCOVER_PRIMARY;

        err = bt_gatt_discover(default_conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed(err %d)\n", err);
            return;
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason 0x%02x)\n", addr, reason);

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;
    dk_set_led_off(DK_LED1);

    start_scan();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void main()
{
    int err;

    err = dk_leds_init();

    if (err) {
        printk("LEDs init failed (err %d)\n", err);
        return;
    }

    if (device_is_ready(strip)) {
        LOG_INF("Found LED strip device %s", strip->name);
    } else {
        LOG_ERR("LED strip device %s is not ready", strip->name);
        return;
    }

    err = bt_enable(NULL);

    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }

    LOG_INF("Bluetooth initialized\n");
    bt_conn_cb_register(&conn_callbacks);
    start_scan();

    int rc;
    bool pixels_off = true;
    float offset = 0.0f;
    memset(&pixels, 0x00, sizeof(pixels));

    while (1) {
        if (led_strip_on) {
            LOG_INF("LED strip ON");
            offset += (float)speed / 2000.0f;

            for (int i = 0; i < STRIP_NUM_PIXELS; i = i + 3) {
                float hue = (float)i / (STRIP_NUM_PIXELS/3);
                pixels_set_hsv(i, hue + offset, 1.0f, 1.0f);
            }

            rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

            if (rc) {
                    LOG_ERR("couldn't update strip: %d", rc);
            }
            pixels_off = false;
        } else {
            if (!pixels_off) {
                memset(&pixels, 0x00, sizeof(pixels));
                rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
                if (rc) {
                    LOG_ERR("couldn't update strip: %d", rc);
                }
                pixels_off = true;
            }
        }
        k_sleep(DELAY_TIME);
    }
}

