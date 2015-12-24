#include "bmp.h"
#include "common.h"

/* gets an int from the buffer (least-significant byte first) */
#define GET_INT(p) ( (p[3] << 3*8) | (p[2] << 2*8) | (p[1] << 1*8) | (p[0] << 0*8) )

/* gets an uint16_t from the buffer (least-significant byte first) */
#define GET_UINT16(p) ( (p[1] << 8) | p[0] )

#define BMP_HEADER_SIZE 14

typedef enum {
  bmp_header_size_os21    = 12,
  bmp_header_size_os22    = 64,
  bmp_header_size_infov1  = 40,
  bmp_header_size_infov2  = 52,
  bmp_header_size_infov3  = 56,
  bmp_header_size_infov4  = 108,
  bmp_header_size_infov5  = 124
} bmp_header_size_t;

int pilot_bmp_try_parse_header(const uint8_t *data, size_t count, pilot_bmp_header_t *header)
{
  int ret = 0;

  if (header != NULL && count >= BMP_HEADER_SIZE)
  {
    if (data[0] == 'B' && data[1] == 'M')
    {
      header->file_size = GET_INT((data+2));
      header->pixel_offset = GET_INT((data+10));

      /* sanity check the header */
      if (header->file_size == count && header->pixel_offset > 0 && header->pixel_offset < count)
        ret = 1;
    }
  }

  return ret;
}

int pilot_bmp_try_parse_dib_header(const uint8_t *data, size_t count, pilot_bmp_dib_header_t *header)
{
  int ret = 0;

  if (header != NULL && count >= BMP_HEADER_SIZE + 4)
  {
    /* the dib header starts directly after the bmp header, it's first member is it's size */
    header->size = GET_INT((data+BMP_HEADER_SIZE));

    /* sanity check the header size */
    if (header->size <= count)
    {
      /* the header size specifies it's type */
      switch (header->size)
      {
        case bmp_header_size_os21:
          header->image_width = GET_UINT16((data+18));
          header->image_height = GET_UINT16((data+20));
          ret = 1;
          break;

        case bmp_header_size_infov1:
        case bmp_header_size_infov2:
        case bmp_header_size_infov3:
        case bmp_header_size_infov4:
        case bmp_header_size_infov5:
          header->image_width = GET_INT((data+18));
          header->image_height = GET_INT((data+22));
          ret = 1;
          break;

        default: ret = 0; break;
      }
    }
  }

  return ret;
}

int pilot_bmp_try_parse_color_table(const uint8_t *data, size_t count, const pilot_bmp_dib_header_t *dib_header, pilot_bmp_color_table_t *color_table)
{
  int color_table_offset;
  int ret = 0;

  /* get the num of colors in the color table */
  int num_colors = GET_INT((data+46));

  /* if the number of colors is exactly 2 or the default (0) is used */
  if (dib_header != NULL && (num_colors == 0 || num_colors == 2))
  {
    color_table_offset = BMP_HEADER_SIZE + dib_header->size;

    /* the data is large enough to contain the color table */
    if (color_table_offset + 8 <= count)
    {
      color_table->color0 = GET_INT((data+color_table_offset));
      color_table->color1 = GET_INT((data+color_table_offset+4));
      ret = 1;
    }
  }

  return ret;
}

/* calculate the row byte count (it must be a multiple of 4) */
static int pilot_bmp_row_byte_count(int image_width)
{
  int byte_count = image_width / 8;
  int remainder = byte_count % 4;
  int padding = 0;

  if (remainder != 0)
    padding = (4 - remainder);

  return byte_count + padding;
}

int pilot_bmp_fill_display_buffer(uint8_t *display_buffer,
                                 size_t display_buffer_count,
                                 const char __user *data,
                                 int data_count,
                                 const pilot_bmp_header_t *header,
                                 const pilot_bmp_dib_header_t *dib_header,
                                 const pilot_bmp_color_table_t *color_table)
{
  int ret = 0;
  int row, col, row_count, col_count, row_bmp, target_index, source_index, byte_per_row;

  /* get the number of rows */
  row_count = dib_header->image_height;
  col_count = dib_header->image_width / 8;
  byte_per_row = pilot_bmp_row_byte_count(dib_header->image_width);

  /* switch the order of the rows (they are bottom up in the bmp) */
  for (row = 0; row < row_count; row++)
  {
    /* foreach display buffer row, calculate the corresponding bitmap row */
    row_bmp = row_count - row - 1;
    for (col = 0; col < col_count; col++)
    {
      target_index = (row * col_count) + col;
      source_index = (row_bmp * byte_per_row) + col + header->pixel_offset;

      /* if color0 != 0, invert bits! */
      display_buffer[target_index] = color_table->color0 ? ~data[source_index] : data[source_index];
    }
    ret+=col_count;
  }
  //LOG_INFO("pilot_bmp_fill_display_buffer() row_count: %i, col_count: %i, data_count: %i", row_count, col_count, data_count);

  return ret;
}
