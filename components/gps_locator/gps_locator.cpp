#include "gps_locator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "libs/lz4/lz4.h"

#include "images/area_locked_tile.h"

LV_IMG_DECLARE(area_locked_tile);

lv_obj_t* GPSLocator::map_container = nullptr;
lv_obj_t* GPSLocator::map_group = nullptr;
lv_obj_t** GPSLocator::tile_components = nullptr;
int GPSLocator::top_left_tile_x = 0;
int GPSLocator::top_left_tile_y = 0;
int GPSLocator::new_top_left_tile_x = 0;
int GPSLocator::new_top_left_tile_y = 0;
int GPSLocator::new_marker_offset_x = 0;
int GPSLocator::new_marker_offset_y = 0;
int GPSLocator::marker_offset_x = 0;
int GPSLocator::marker_offset_y = 0;
int GPSLocator::tile_count = 0;
bool GPSLocator::initialized = false;
bool GPSLocator::is_loading = false;

struct TileSlot {
    uint8_t* buf;          // Pixel buffer
    lv_image_dsc_t img;    // LVGL image descriptor
};

TileSlot* tiles = nullptr;

bool GPSLocator::init(lv_obj_t* parent_screen) {
    if (is_loading) return true;

    tile_count = MAP_TILES_GRID_COLS * MAP_TILES_GRID_ROWS;

    tiles = (TileSlot*)calloc(tile_count, sizeof(TileSlot));
    if (!tiles) {
        printf("Failed to allocate tile slots\n");
        return false;
    }

    // Create map container
    map_container = lv_obj_create(parent_screen);
    lv_obj_set_size(map_container, MAP_WIDTH, MAP_HEIGHT);  
    lv_obj_center(map_container);
    lv_obj_set_style_pad_all(map_container, 0, 0);
    lv_obj_set_style_border_width(map_container, 0, 0);
    lv_obj_set_style_radius(map_container, 0, 0);
    lv_obj_set_style_bg_color(map_container, lv_color_make(0,0,0), 0);
    lv_obj_remove_flag(map_container, LV_OBJ_FLAG_SCROLLABLE);

    create_tile_components();

    initialized = true;
    return true;
}

void GPSLocator::create_tile_components() {
    if (!tile_components) {
        tile_components = (lv_obj_t**)calloc(tile_count, sizeof(lv_obj_t*));
        if (!tile_components) {
            printf("Failed to allocate tile_components array\n");
            return;
        }
    }

    if (!map_group) {
        map_group = lv_obj_create(map_container);
        lv_obj_set_size(map_group, MAP_TILES_TILE_SIZE * MAP_TILES_GRID_COLS, MAP_TILES_TILE_SIZE * MAP_TILES_GRID_ROWS);
        lv_obj_set_style_pad_all(map_group, 0, 0);
        lv_obj_set_style_border_width(map_group, 0, 0);
        lv_obj_set_pos(map_group, 0, 0);
        
        // Create grid of image widgets for tiles
        for (int i = 0; i < tile_count; i++) {
            tile_components[i] = lv_image_create(map_group);

            int row = i / MAP_TILES_GRID_COLS;
            int col = i % MAP_TILES_GRID_COLS;
            
            lv_obj_set_pos(tile_components[i], col * MAP_TILES_TILE_SIZE, row * MAP_TILES_TILE_SIZE);
            lv_obj_set_size(tile_components[i], MAP_TILES_TILE_SIZE, MAP_TILES_TILE_SIZE);

            // Set default background while tiles load
            lv_obj_set_style_bg_opa(tile_components[i], LV_OPA_COVER, 0);
        }
    }
}

bool GPSLocator::load_tile_images() {
    if (is_loading) {
        printf("GPSLocator: Already loading\n");
        return false;
    }
    is_loading = true;

    for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
        for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
            int index = row * MAP_TILES_GRID_COLS + col;
            int tile_x = top_left_tile_x + col;
            int tile_y = top_left_tile_y + row;

            // Load tile from SD card
            bool loaded = fetch_images_from_sd(index, tile_x, tile_y);
            
            if (loaded) {
                // Get the tile image data and set it
                lv_image_set_src(tile_components[index], &tiles[index].img);
            } else {
                tiles[col].img = area_locked_tile;          // Copy descriptor
                tiles[col].buf = nullptr;                   // No writable buffer
                lv_image_set_src(tile_components[index], &area_locked_tile);
            }
        }
    }

    is_loading = false;

    printf("GPSLocator: Tile images loaded\n");

    return true;
}

bool GPSLocator::fetch_images_from_sd(int index, int tile_x, int tile_y) {
    TileSlot& slot = tiles[index];
    
    char path[256];
    snprintf(path, sizeof(path), "%s/%s/%d/%d/%d.bin", 
             BASE_PATH, TILE_FOLDER, MAP_ZOOM, tile_x, tile_y);
    
    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("Tile not found: %s", path);
        return false;
    }
    
    uint8_t magic = 0;
    uint8_t color_format = 0;
    uint16_t flags = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    uint16_t stride = 0;
    uint16_t reserved = 0;

    if (fread(&magic, 1, sizeof(magic), f) != sizeof(magic) ||
        fread(&color_format, 1, sizeof(color_format), f) != sizeof(color_format) ||
        fread(&flags, 1, sizeof(flags), f) != sizeof(flags) ||
        fread(&width, 1, sizeof(width), f) != sizeof(width) ||
        fread(&height, 1, sizeof(height), f) != sizeof(height) ||
        fread(&stride, 1, sizeof(stride), f) != sizeof(stride) ||
        fread(&reserved, 1, sizeof(reserved), f) != sizeof(reserved)) {
        printf("Tile %d: failed to read header\n", index);
        fclose(f);
        return false;
    }

    if (magic != 0x19) {
        printf("Tile %d: invalid magic 0x%02X\n", index, magic);
        fclose(f);
        return false;
    }
    if (color_format != 0x12) {
        printf("Tile %d: unexpected color format 0x%02X\n", index, color_format);
        fclose(f);
        return false;
    }
    if ((flags & 0x02) == 0) {
        printf("Tile %d: missing LZ4 flag (flags=0x%04X)\n", index, flags);
        fclose(f);
        return false;
    }
    if (width != MAP_TILES_TILE_SIZE || height != MAP_TILES_TILE_SIZE) {
        printf("Tile %d: unexpected size %ux%u\n", index, width, height);
        fclose(f);
        return false;
    }

    // Skip 12-byte header
    if (fseek(f, 12, SEEK_SET) != 0) {
        printf("Tile %d: failed to seek header\n", index);
        fclose(f);
        return false;
    }

    uint32_t compressed_size = 0;
    if (fread(&compressed_size, 1, sizeof(compressed_size), f) != sizeof(compressed_size)) {
        printf("Tile %d: failed to read compressed size\n", index);
        fclose(f);
        return false;
    }
    if (compressed_size == 0) {
        printf("Tile %d: compressed size is 0\n", index);
        fclose(f);
        return false;
    }
    
    size_t img_size = MAP_TILES_TILE_SIZE * MAP_TILES_TILE_SIZE * MAP_TILES_BYTES_PER_PIXEL;

    if (!slot.buf) {
        slot.buf = (uint8_t*)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!slot.buf) {
            printf("Tile %d: allocation failed\n", index);
            fclose(f);
            return false;
        }
    }
    
    uint8_t *compressed_buf = (uint8_t*)heap_caps_malloc(compressed_size, MALLOC_CAP_8BIT);
    if (!compressed_buf) {
        printf("Tile %d: compressed buffer allocation failed (%zu)\n", index, compressed_size);
        fclose(f);
        return false;
    }

    size_t bytes_read = fread(compressed_buf, 1, compressed_size, f);
    fclose(f);
    if (bytes_read != compressed_size) {
        printf("Tile %d: incomplete compressed read %zu/%u\n", index, bytes_read, compressed_size);
        free(compressed_buf);
        return false;
    }

    int decompressed = LZ4_decompress_safe((const char*)compressed_buf,
                                           (char*)slot.buf,
                                           (int)compressed_size,
                                           (int)img_size);
    free(compressed_buf);
    if (decompressed < 0 || (size_t)decompressed != img_size) {
        printf("Tile %d: LZ4 decompress failed (%d)\n", index, decompressed);
        return false;
    }
    
    // Setup image descriptor
    slot.img.header.w = MAP_TILES_TILE_SIZE;
    slot.img.header.h = MAP_TILES_TILE_SIZE;
    slot.img.header.cf = MAP_TILES_COLOR_FORMAT;
    slot.img.header.stride = MAP_TILES_TILE_SIZE * MAP_TILES_BYTES_PER_PIXEL;
    slot.img.data = slot.buf;
    slot.img.data_size = img_size;

    return true;
}

void GPSLocator::get_tile_coordinates(double latitude, double longitude, double &tile_x, double &tile_y) {
    tile_x = ((longitude + 180.0) / 360.0 * (1 << MAP_ZOOM));
    double lat_rad = latitude * M_PI / 180.0;
    tile_y = (1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * (1 << MAP_ZOOM);
}

void GPSLocator::get_marker_offsets(double &tile_x, double &tile_y, int &offset_x, int &offset_y) {
    offset_x = (int)((tile_x - (int)tile_x) * MAP_TILES_TILE_SIZE);
    offset_y = (int)((tile_y - (int)tile_y) * MAP_TILES_TILE_SIZE);
}

void GPSLocator::center_map_on_gps() {
    printf("GPSLocator: Centering on GPS coordinates\n");

    lv_obj_align(map_group, LV_ALIGN_TOP_LEFT, 0 - marker_offset_x, 0 - marker_offset_y);
}

static void anim_set_x_cb(void * obj, int32_t v) {
    lv_obj_set_x((lv_obj_t *)obj, v);
}

static void anim_set_y_cb(void * obj, int32_t v) {
    lv_obj_set_y((lv_obj_t *)obj, v);
}

void GPSLocator::animate_map_center() {
    printf("Animate from (%d, %d) to (%d, %d)\n", marker_offset_x, marker_offset_y, new_marker_offset_x, new_marker_offset_y);

    // X animation
    lv_anim_t ax;
    lv_anim_init(&ax);
    lv_anim_set_var(&ax, map_group);
    lv_anim_set_time(&ax, STEP_ANIMATION_DURATION);
    lv_anim_set_exec_cb(&ax, anim_set_x_cb);
    lv_anim_set_values(&ax, 0 - marker_offset_x, 0 - new_marker_offset_x);
    lv_anim_start(&ax);

    // Y animation
    lv_anim_t ay;
    lv_anim_init(&ay);
    lv_anim_set_var(&ay, map_group);
    lv_anim_set_time(&ay, STEP_ANIMATION_DURATION);
    lv_anim_set_exec_cb(&ay, anim_set_y_cb);
    lv_anim_set_values(&ay, 0 - marker_offset_y, 0 - new_marker_offset_y);
    lv_anim_start(&ay);

    marker_offset_x = new_marker_offset_x;
    marker_offset_y = new_marker_offset_y;
}

void GPSLocator::show_initial_location(double latitude, double longitude) {
    if (!initialized) return;

    printf("GPSLocator: Showing location at %.6f, %.6f\n", latitude, longitude);

    double tile_x, tile_y;
    get_tile_coordinates(latitude, longitude, tile_x, tile_y);
    get_marker_offsets(tile_x, tile_y, marker_offset_x, marker_offset_y);

    // store integer tile coordinates
    // shift to top-left tile of grid
    top_left_tile_x = (int)tile_x - (int)(MAP_TILES_GRID_COLS / 2);
    top_left_tile_y = (int)tile_y - (int)(MAP_TILES_GRID_ROWS / 2);

    // Load tile images and move map to center
    load_tile_images();
    center_map_on_gps();
}

void GPSLocator::set_fallback_tile(int col) {
   tiles[col].img = area_locked_tile;          // Copy descriptor
   tiles[col].buf = nullptr;                   // No writable buffer
   lv_image_set_src(GPSLocator::tile_components[col], &area_locked_tile);
}

void GPSLocator::shift_up() {
    // Shift tile mapping up by one row
    
    for (int row = MAP_TILES_GRID_ROWS - 1; row >= 0; row--) {
        if (row > 0) {
            for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
                int from_index = (row - 1) * MAP_TILES_GRID_COLS + col;
                int to_index = row * MAP_TILES_GRID_COLS + col;
                
                std::swap(tiles[to_index], tiles[from_index]);
            }
        } else {

            // load new top row asynchronously
            xTaskCreate([](void* param) {
                struct TileUpdate { int col; bool loaded; };
                for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
                    int tile_x = GPSLocator::top_left_tile_x + col;
                    bool loaded = GPSLocator::fetch_images_from_sd(col, tile_x, GPSLocator::new_top_left_tile_y);
                    TileUpdate* update = new TileUpdate{col, loaded};
                    lv_async_call([](void* p) {
                        auto* upd = static_cast<TileUpdate*>(p);
                        if (upd->loaded) {
                            lv_image_set_src(tile_components[upd->col], &tiles[upd->col].img);
                        } else {
                            set_fallback_tile(upd->col);
                        }
                        delete upd;
                    }, update);
                }
                vTaskDelete(NULL);
            }, "tile_load_task", 4096, NULL, 5, NULL);
        }
    }
}

void GPSLocator::shift_down() {
    // Shift tile mapping down by one row

    for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
        if (row < MAP_TILES_GRID_ROWS - 1) {
            for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
                int from_index = row * MAP_TILES_GRID_COLS + col;
                int to_index = (row + 1) * MAP_TILES_GRID_COLS + col;
                
                std::swap(tiles[to_index], tiles[from_index]);
            }
        } else {

            // load new bottom row asynchronously
            xTaskCreate([](void* param) {
                struct TileUpdate { int col; bool loaded; };
                for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
                    int tile_x = GPSLocator::top_left_tile_x + col;
                    int tile_y = GPSLocator::new_top_left_tile_y + (MAP_TILES_GRID_ROWS - 1);
                    int index = (MAP_TILES_GRID_ROWS - 1) * MAP_TILES_GRID_COLS + col;
                    bool loaded = GPSLocator::fetch_images_from_sd(index, tile_x, tile_y);
                    TileUpdate* update = new TileUpdate{index, loaded};
                    lv_async_call([](void* p) {
                        auto* upd = static_cast<TileUpdate*>(p);
                        if (upd->loaded) {
                            lv_image_set_src(tile_components[upd->col], &tiles[upd->col].img);
                        } else {
                            set_fallback_tile(upd->col);
                        } 
                        delete upd;
                    }, update);
                }
                vTaskDelete(NULL);
            }, "tile_load_down", 4096, NULL, 5, NULL);
        }
    }
}

void GPSLocator::shift_left() {
    // Shift tile mapping left by one column
    for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
        for (int col = MAP_TILES_GRID_COLS - 1; col > 0; col--) {
            int to_index   = row * MAP_TILES_GRID_COLS + col;
            int from_index = row * MAP_TILES_GRID_COLS + (col - 1);

            std::swap(tiles[to_index], tiles[from_index]);
        }
    }

    // Now reload column 0 asynchronously
    xTaskCreate([](void* param) {
        struct TileUpdate { int index; bool loaded; };
        int left_col = 0;
        for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
            int index = row * MAP_TILES_GRID_COLS + left_col;
            int tile_x = GPSLocator::new_top_left_tile_x;
            int tile_y = GPSLocator::top_left_tile_y + row;
            bool loaded = GPSLocator::fetch_images_from_sd(index, tile_x, tile_y);
            TileUpdate* update = new TileUpdate{index, loaded};
            lv_async_call([](void* p) {
                auto* upd = static_cast<TileUpdate*>(p);
                if (upd->loaded) {
                    lv_image_set_src(tile_components[upd->index], &tiles[upd->index].img);
                } else {
                    set_fallback_tile(upd->index);
                }
                delete upd;
            }, update);
        }
        vTaskDelete(NULL);
    }, "tile_load_left", 4096, NULL, 5, NULL);
}

void GPSLocator::shift_right() {
    // Shift columns LEFT
    for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
        for (int col = 0; col < MAP_TILES_GRID_COLS - 1; col++) {

            int to_index   = row * MAP_TILES_GRID_COLS + col;
            int from_index = row * MAP_TILES_GRID_COLS + (col + 1);

            std::swap(tiles[to_index], tiles[from_index]);
        }
    }

    // Now reload rightmost column asynchronously
    xTaskCreate([](void* param) {
        struct TileUpdate { int index; bool loaded; };
        int right_col = MAP_TILES_GRID_COLS - 1;
        for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
            int index = row * MAP_TILES_GRID_COLS + right_col;
            int tile_x = GPSLocator::new_top_left_tile_x + right_col;
            int tile_y = GPSLocator::top_left_tile_y + row;
            bool loaded = GPSLocator::fetch_images_from_sd(index, tile_x, tile_y);
            TileUpdate* update = new TileUpdate{index, loaded};
            lv_async_call([](void* p) {
                auto* upd = static_cast<TileUpdate*>(p);
                if (upd->loaded) {
                    lv_image_set_src(tile_components[upd->index], &tiles[upd->index].img);
                } else {
                    set_fallback_tile(upd->index);
                }
                delete upd;
            }, update);
        }
        vTaskDelete(NULL);
    }, "tile_load_right", 4096, NULL, 5, NULL);
}

void GPSLocator::move_location(double latitude, double longitude) {
    if (!initialized) return;

    double new_tile_x, new_tile_y;
    get_tile_coordinates(latitude, longitude, new_tile_x, new_tile_y);

    new_top_left_tile_x = (int)new_tile_x - 1;
    new_top_left_tile_y = (int)new_tile_y - 1;

    if (top_left_tile_x != new_top_left_tile_x || top_left_tile_y != new_top_left_tile_y) {
        // Still within current center tile, just update offsets
        if (new_top_left_tile_x < top_left_tile_x) {
           shift_left();
           marker_offset_x = marker_offset_x + MAP_TILES_TILE_SIZE;
        } else if (new_top_left_tile_x > top_left_tile_x) {
            shift_right();
            marker_offset_x = marker_offset_x - MAP_TILES_TILE_SIZE;
        }
        if (new_top_left_tile_y < top_left_tile_y) {
            shift_up();
            marker_offset_y = marker_offset_y + MAP_TILES_TILE_SIZE;
        } else if (new_top_left_tile_y > top_left_tile_y) {
            shift_down();
            marker_offset_y = marker_offset_y - MAP_TILES_TILE_SIZE;
        }

        top_left_tile_x = new_top_left_tile_x;
        top_left_tile_y = new_top_left_tile_y;
    }
    
    get_marker_offsets(new_tile_x, new_tile_y, new_marker_offset_x, new_marker_offset_y);

    animate_map_center();
}
