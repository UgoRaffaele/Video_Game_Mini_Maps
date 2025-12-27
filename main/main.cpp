#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_memory_utils.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "driver/i2c_master.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "freertos/FreeRTOS.h"

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <cmath>
#include <stdlib.h>

#include "gps_locator.h"

#include "images/maps_bg.h"
#include "images/car_icon.h"
#include "images/north_pointer.h"
#include "images/no_satellite.h"

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / M_PI)

// declarations
static const char *TAG_GPS = "gps_i2c";

LV_IMG_DECLARE(maps_bg);
LV_IMG_DECLARE(car_icon);
LV_IMG_DECLARE(north_pointer);
LV_IMG_DECLARE(no_satellite);


// screens
lv_obj_t *main_scr;

// global elements
lv_obj_t *no_satellite_bg;
lv_obj_t *map_container;

lv_obj_t *car_icon_img;
lv_obj_t *north_pointer_img;

// CONTROL VARIABLE INIT

#define MIN_MOVE_DISTANCE 2.0 // distance in meters to trigger icon rotation

float current_latitude        = 0.0;
float current_longitude       = 0.0;
float new_latitude            = 0.0;
float new_longitude           = 0.0;
int current_angle             = 0;
int new_angle                 = 0;
volatile bool data_ready      = false; // new incoming data
bool init_anim_complete       = false; // needle sweep completed - tbc
bool location_initialized     = false; // has the initial GPS location been set

// general color palettes
const lv_color_t PALETTE_BLACK        = LV_COLOR_MAKE(0, 0, 0);
const lv_color_t PALETTE_BLUE         = LV_COLOR_MAKE(31, 104, 135); // fuel arc main
const lv_color_t PALETTE_BLUE_NEON    = LV_COLOR_MAKE(83, 252, 254); // fuel arc indicator
const lv_color_t PALETTE_DARK_GREY    = LV_COLOR_MAKE(24, 24, 24); // highlight button background
const lv_color_t PALETTE_RED          = LV_COLOR_MAKE(130, 35, 53); // redline
const lv_color_t PALETTE_GREEN        = LV_COLOR_MAKE(123, 207, 21); // buttons and text
const lv_color_t PALETTE_GREY         = LV_COLOR_MAKE(120, 120, 120); // button background
const lv_color_t PALETTE_WHITE        = LV_COLOR_MAKE(255, 255, 255);

// NFSU2 pickable colors
const lv_color_t PALETTE_NFS_WHITE    = LV_COLOR_MAKE(255, 255, 255);
const lv_color_t PALETTE_NFS_BLUE     = LV_COLOR_MAKE(52, 154, 227);
const lv_color_t PALETTE_NFS_CYAN     = LV_COLOR_MAKE(34, 199, 239);
const lv_color_t PALETTE_NFS_GREEN    = LV_COLOR_MAKE(93, 239, 39);
const lv_color_t PALETTE_NFS_CITRUS   = LV_COLOR_MAKE(221, 221, 37);
const lv_color_t PALETTE_NFS_LIME     = LV_COLOR_MAKE(148, 248, 38);
const lv_color_t PALETTE_NFS_ORANGE   = LV_COLOR_MAKE(244, 153, 37);
const lv_color_t PALETTE_NFS_RED      = LV_COLOR_MAKE(255, 42, 22);
const lv_color_t PALETTE_NFS_PURPLE   = LV_COLOR_MAKE(136, 86, 255);
const lv_color_t PALETTE_NFS_GREY     = LV_COLOR_MAKE(175, 181, 191);
const lv_color_t PALETTE_NFS_BLUE2    = LV_COLOR_MAKE(27, 173, 252);
const lv_color_t PALETTE_NFS_YELLOW   = LV_COLOR_MAKE(229, 223, 33);

// get bearing angle between two coordinates
double angle_from_coordinate(double lat1, double long1, double lat2, double long2) {
    double lat1_rad = DEG_TO_RAD(lat1);
    double lat2_rad = DEG_TO_RAD(lat2);
    double dlon_rad = DEG_TO_RAD(long2 - long1);

    double y = sin(dlon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad)*sin(lat2_rad) - sin(lat1_rad)*cos(lat2_rad)*cos(dlon_rad);
    double bearing_rad = atan2(y, x);

    double bearing_deg = fmod(RAD_TO_DEG(bearing_rad) + 360.0, 360.0);

    int bearing_lvgl = (int)round(bearing_deg * 10.0);

    return bearing_lvgl;
}

// calculate distance between two coordinates in meters
double distance_between(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth radius in meters
    double dLat = DEG_TO_RAD(lat2 - lat1);
    double dLon = DEG_TO_RAD(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(DEG_TO_RAD(lat1)) * cos(DEG_TO_RAD(lat2)) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// normalize angle to shortest rotation direction
float normalize_angle(float from, float to) {
    float diff = to - from;
    while (diff > 1800) diff -= 3600;
    while (diff < -1800) diff += 3600;
    return from + diff;
}

static void anim_set_r_cb(void * obj, int32_t v) {
    lv_img_set_angle((lv_obj_t *)obj, v);
}

void update_values(void) {
    if (location_initialized) {
        GPSLocator::move_location((double)new_latitude, (double)new_longitude);
    } else {            
        GPSLocator::show_initial_location((double)new_latitude, (double)new_longitude);
        location_initialized = true;
    }

    // TODO - animations in / out and lost connection handling
    lv_obj_set_style_opa(no_satellite_bg, LV_OPA_0, 0);
    lv_obj_set_style_opa(map_container, LV_OPA_COVER, 0);
    lv_obj_set_style_opa(car_icon_img, LV_OPA_COVER, 0);

    double dist = distance_between(current_latitude, current_longitude, new_latitude, new_longitude);
  
    if (dist > MIN_MOVE_DISTANCE) {
        new_angle = angle_from_coordinate(current_latitude, current_longitude, new_latitude, new_longitude);
        float anim_target_angle = normalize_angle(current_angle, new_angle);

        lv_anim_t aa;
        lv_anim_init(&aa);
        lv_anim_set_var(&aa, car_icon_img);
        lv_anim_set_time(&aa, STEP_ANIMATION_DURATION);
        lv_anim_set_exec_cb(&aa, anim_set_r_cb);

        lv_anim_set_values(&aa, current_angle, anim_target_angle);
        lv_anim_start(&aa);

        current_angle = anim_target_angle;
    }

    current_latitude = new_latitude;
    current_longitude = new_longitude;
}

void make_screen(void) {
    main_scr = lv_obj_create(NULL);
    lv_obj_set_size(main_scr, 466, 466);
    lv_obj_set_style_bg_color(main_scr, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(main_scr, 0, 0);
    lv_obj_set_style_border_width(main_scr, 0, 0);

    lv_obj_t *maps_bg_img = lv_img_create(main_scr);
    lv_image_set_src(maps_bg_img, &maps_bg);
    lv_obj_align(maps_bg_img, LV_ALIGN_CENTER, 0, 0);

    map_container = lv_obj_create(main_scr);
    lv_obj_set_size(map_container, 466, 466);
    lv_obj_align(map_container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_opa(map_container, LV_OPA_0, 0);
    lv_obj_set_style_bg_color(map_container, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(map_container, 0, 0);
    lv_obj_set_style_border_width(map_container, 0, 0);
    lv_obj_set_style_outline_color(map_container, PALETTE_GREY, 0);

    no_satellite_bg = lv_img_create(main_scr);
    lv_image_set_src(no_satellite_bg, &no_satellite);
    lv_obj_set_size(no_satellite_bg, 466, 466);
    lv_obj_align(no_satellite_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_opa(no_satellite_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(no_satellite_bg, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(no_satellite_bg, 0, 0);
    lv_obj_set_style_border_width(no_satellite_bg, 0, 0);
}

void make_ui(void) {
    car_icon_img = lv_img_create(main_scr);
    lv_image_set_src(car_icon_img, &car_icon);
    lv_obj_set_style_opa(car_icon_img, LV_OPA_0, 0);
    lv_obj_align(car_icon_img, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_transform_pivot_x(car_icon_img, 24, 0);
    lv_obj_set_style_transform_pivot_y(car_icon_img, 21, 0);

    north_pointer_img = lv_img_create(main_scr);
    lv_image_set_src(north_pointer_img, &north_pointer);
    lv_obj_align(north_pointer_img, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_transform_pivot_x(north_pointer_img, 43, 0);
    lv_obj_set_style_transform_pivot_y(north_pointer_img, (200 + 30), 0);
}

void mount_sd(void) {
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        printf("Failed to mount SD card, error: %s\n", esp_err_to_name(err));
    }
}

// decouple movement updates from GPS UART task
void lvgl_timer(lv_timer_t * timer) {
    if (data_ready) {
        data_ready = false;
        update_values();
    }
}

static bool nmea_to_decimal(const char *coord, char hemi, double *out) {
    if (!coord || !*coord) return false;
    double val = atof(coord);
    int deg = (int)(val / 100.0);
    double minutes = val - (deg * 100.0);
    double dec = deg + (minutes / 60.0);
    if (hemi == 'S' || hemi == 'W') {
        dec = -dec;
    }
    *out = dec;
    return true;
}

static bool parse_rmc_line(const char *line, double *lat, double *lon) {
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) {
        return false;
    }

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *save = NULL;
    char *token = strtok_r(buf, ",", &save);
    int field = 0;
    const char *status = NULL;
    const char *lat_str = NULL;
    const char *lon_str = NULL;
    char lat_hemi = '\0';
    char lon_hemi = '\0';

    while (token) {
        switch (field) {
            case 2: status = token; break;
            case 3: lat_str = token; break;
            case 4: lat_hemi = token[0]; break;
            case 5: lon_str = token; break;
            case 6: lon_hemi = token[0]; break;
            default: break;
        }
        token = strtok_r(NULL, ",", &save);
        field++;
    }

    if (!status || status[0] != 'A') {
        ESP_LOGW(TAG_GPS, "RMC no fix (status=%s)", status ? status : "null");
        return false;
    }

    double tmp_lat = 0.0;
    double tmp_lon = 0.0;
    if (!nmea_to_decimal(lat_str, lat_hemi, &tmp_lat)) {
        ESP_LOGW(TAG_GPS, "Invalid latitude in RMC");
        return false;
    }
    if (!nmea_to_decimal(lon_str, lon_hemi, &tmp_lon)) {
        ESP_LOGW(TAG_GPS, "Invalid longitude in RMC");
        return false;
    }

    *lat = tmp_lat;
    *lon = tmp_lon;
    return true;
}

#define I2C_MASTER_SCL_IO GPIO_NUM_14
#define I2C_MASTER_SDA_IO GPIO_NUM_15
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LC76G_DEVICE_ADDRESS 0x50
#define LC76G_DEVICE_ADDRESS_R 0x54

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t lc76g_dev_w = NULL;
static i2c_master_dev_handle_t lc76g_dev_r = NULL;

static void gps_i2c_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg_w = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LC76G_DEVICE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg_w, &lc76g_dev_w));

    i2c_device_config_t dev_cfg_r = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LC76G_DEVICE_ADDRESS_R,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg_r, &lc76g_dev_r));
    ESP_LOGI(TAG_GPS, "I2C init ok (SCL=GPIO14, SDA=GPIO15, 100kHz)");
}

static esp_err_t lc76g_i2c_write(uint8_t device_addr, const uint8_t *data, size_t data_len) {
    if (device_addr == LC76G_DEVICE_ADDRESS && lc76g_dev_w) {
        return i2c_master_transmit(lc76g_dev_w, data, data_len, pdMS_TO_TICKS(1000));
    }
    if (device_addr == LC76G_DEVICE_ADDRESS_R && lc76g_dev_r) {
        return i2c_master_transmit(lc76g_dev_r, data, data_len, pdMS_TO_TICKS(1000));
    }
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t lc76g_i2c_read(uint8_t device_addr, uint8_t *data, size_t data_len) {
    if (device_addr == LC76G_DEVICE_ADDRESS_R && lc76g_dev_r) {
        return i2c_master_receive(lc76g_dev_r, data, data_len, pdMS_TO_TICKS(1000));
    }
    if (device_addr == LC76G_DEVICE_ADDRESS && lc76g_dev_w) {
        return i2c_master_receive(lc76g_dev_w, data, data_len, pdMS_TO_TICKS(1000));
    }
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t lc76g_init_read(uint8_t *len_buf, size_t len_buf_size) {
    uint8_t data[] = { 0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00 };
    esp_err_t ret = lc76g_i2c_write(LC76G_DEVICE_ADDRESS, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_GPS, "LC76G init write failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = lc76g_i2c_read(LC76G_DEVICE_ADDRESS_R, len_buf, len_buf_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG_GPS, "LC76G len read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void gps_i2c_task(void *arg) {
    uint8_t read_len_buf[4] = { 0 };
    char line[128];
    size_t idx = 0;

    while (true) {
        if (lc76g_init_read(read_len_buf, sizeof(read_len_buf)) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint32_t data_len = (uint32_t)read_len_buf[0] |
                            ((uint32_t)read_len_buf[1] << 8) |
                            ((uint32_t)read_len_buf[2] << 16) |
                            ((uint32_t)read_len_buf[3] << 24);

        if (data_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint8_t header[] = { 0x00, 0x20, 0x51, 0xAA };
        uint8_t data_to_send[sizeof(header) + sizeof(read_len_buf)];
        memcpy(data_to_send, header, sizeof(header));
        memcpy(data_to_send + sizeof(header), read_len_buf, sizeof(read_len_buf));
        vTaskDelay(pdMS_TO_TICKS(100));

        if (lc76g_i2c_write(LC76G_DEVICE_ADDRESS, data_to_send, sizeof(data_to_send)) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint8_t *nmea_buf = (uint8_t *)malloc(data_len);
        if (!nmea_buf) {
            ESP_LOGW(TAG_GPS, "NMEA buffer alloc failed (len=%u)", (unsigned)data_len);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        if (lc76g_i2c_read(LC76G_DEVICE_ADDRESS_R, nmea_buf, data_len) != ESP_OK) {
            ESP_LOGW(TAG_GPS, "NMEA read failed");
            free(nmea_buf);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        for (uint32_t i = 0; i < data_len; i++) {
            char c = (char)nmea_buf[i];
            if (c == '\n' || c == '\r') {
                if (idx == 0) {
                    continue;
                }
                line[idx] = '\0';
                idx = 0;

                double lat = 0.0;
                double lon = 0.0;
                if (parse_rmc_line(line, &lat, &lon)) {
                    new_latitude = (float)lat;
                    new_longitude = (float)lon;
                    data_ready = true;
                    ESP_LOGI(TAG_GPS, "Fix: lat=%.6f lon=%.6f", lat, lon);
                }
                continue;
            }

            if (idx < sizeof(line) - 1) {
                line[idx++] = c;
            } else {
                idx = 0;
            }
        }

        free(nmea_buf);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

extern "C" void app_main(void) {
    mount_sd();

    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = true
        }
    };
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();
    bsp_display_brightness_set(100);

    gps_i2c_init();
    xTaskCreatePinnedToCore(gps_i2c_task, "gps_i2c_task", 4096, NULL, 5, NULL, 1);

    bsp_display_lock(0);

    make_screen();

    if (!GPSLocator::init(map_container)) {
        printf("Failed to initialize map\n");
        return;
    }

    make_ui();

    lv_screen_load(main_scr);
    
    bsp_display_unlock();

    lv_timer_t * timer = lv_timer_create(lvgl_timer, 10,  NULL);
    (void)timer;
}
