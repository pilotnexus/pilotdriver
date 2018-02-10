#ifndef __pilot_BMP_H__
#define __pilot_BMP_H__

#include <linux/kernel.h> /* needed for uint8_t, size_t, etc. */

typedef struct {
  int file_size;    /* the size of the bitmap file */
  int pixel_offset; /* the offset to the pixel data */
} pilot_bmp_header_t;

typedef struct {
  int size;        /* the size in bytes of the dib header */
  int image_width; /* the width of of the image in pixel */
  int image_height; /* the height of the image in pixel */
} pilot_bmp_dib_header_t;

typedef struct {
  int color0;
  int color1;
} pilot_bmp_color_table_t;

int pilot_bmp_try_parse_header(const uint8_t *data, size_t count, pilot_bmp_header_t *header);

int pilot_bmp_try_parse_dib_header(const uint8_t *data, size_t count, pilot_bmp_dib_header_t *header);

int pilot_bmp_try_parse_color_table(const uint8_t *data, size_t count, const pilot_bmp_dib_header_t *header, pilot_bmp_color_table_t *color_table);

int pilot_bmp_fill_display_buffer(uint8_t *display_buffer,
                                 size_t display_buffer_count,
                                 const char __user *data,
                                 int data_count,
                                 const pilot_bmp_header_t *header,
                                 const pilot_bmp_dib_header_t *dib_header,
                                 const pilot_bmp_color_table_t *color_table);

#endif
