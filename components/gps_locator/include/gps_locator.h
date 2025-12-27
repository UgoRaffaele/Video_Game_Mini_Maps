#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAP_TILES_TILE_SIZE 256
#define MAP_TILES_GRID_COLS 3
#define MAP_TILES_GRID_ROWS 3
#define MAP_TILES_BYTES_PER_PIXEL 2
#define MAP_TILES_COLOR_FORMAT LV_COLOR_FORMAT_RGB565
#define MAP_HEIGHT 466
#define MAP_WIDTH 466
#define MAP_ZOOM 18
#define BASE_PATH "/sdcard"
#define TILE_FOLDER "tiles1"
#define STEP_ANIMATION_DURATION 500

class GPSLocator {
public:
    static lv_obj_t** tile_components;

    // Initialize the simple map system
    static bool init(lv_obj_t* parent_screen);
    
    // Display map at specific coordinates
    static void show_initial_location(double latitude, double longitude);

    // Update location (for moving GPS)
    static void move_location(double latitude, double longitude);

    static bool fetch_images_from_sd(int index, int tile_x, int tile_y);

    // Cleanup
    static void cleanup();

private:
    static lv_obj_t* map_container;
    static lv_obj_t* map_group;
    static int tile_count;
    static int top_left_tile_x;
    static int top_left_tile_y;
    static int new_top_left_tile_x;
    static int new_top_left_tile_y;
    static int marker_offset_x;
    static int marker_offset_y;
    static int new_marker_offset_x;
    static int new_marker_offset_y;
    static bool initialized;
    static bool is_loading;
    static int tile_mapping[MAP_TILES_GRID_ROWS * MAP_TILES_GRID_COLS];

    static void create_tile_components();
    static bool load_tile_images();
    static void set_fallback_tile(int col);
    static void get_tile_coordinates(double latitude, double longitude, double &tile_x, double &tile_y);
    static void get_marker_offsets(double &tile_x, double &tile_y, int &offset_x, int &offset_y);
    
    static void deinit();
    static void center_map_on_gps();
    static void animate_map_center();
    static void shift_right();
    static void shift_left();
    static void shift_down();
    static void shift_up();
    static bool is_location_within_center(int new_latitude, int new_longitude);
    static void show_loading_popup();
    static void hide_loading_popup();
};

#ifdef __cplusplus
}
#endif