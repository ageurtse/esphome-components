#include "axs15231_display.h"
#include "axs15231_defines.h"

#include "esphome/core/log.h"
#include "esphome/components/display/display_color_utils.h"

#ifdef USE_ESP_IDF

namespace esphome {
namespace axs15231 {

namespace {
  constexpr static const char *const TAG = "axs15231.display";

  typedef struct
  {
      uint8_t cmd;
      uint8_t data[36];
      uint8_t len;
  } lcd_cmd_t;

  const static lcd_cmd_t AXS_QSPI_INIT_NEW[] = {
    {AXS_LCD_DISPOFF, {0x00}, 0x40},
    {AXS_LCD_SLPIN,   {0x00}, 0x10},
    {AXS_LCD_SLPOUT,  {0x00}, 0x20},
    {AXS_LCD_DISPON,  {0x00}, 0x00},
  };

  // store a 16 bit value in a buffer, big endian.
  static inline void put16_be(uint8_t *buf, uint16_t value) {
    buf[0] = value >> 8;
    buf[1] = value;
  }
}  // anonymous namespace

void AXS15231Display::update() {
 if (this->prossing_update_) {
    this->need_update_ = true;
    return;
  }
  this->prossing_update_ = true;
  do {
    this->need_update_ = false;
    this->do_update_();
  } while (this->need_update_);
  this->prossing_update_ = false;
  this->display_();
}

void ILI9XXXDisplay::display_() {
  // check if something was displayed
  if ((this->x_high_ < this->x_low_) || (this->y_high_ < this->y_low_)) {
    return;
  }

  // we will only update the changed rows to the display
  size_t const w = this->x_high_ - this->x_low_ + 1;
  size_t const h = this->y_high_ - this->y_low_ + 1;

  size_t mhz = this->data_rate_ / 1000000;
  // estimate time for a single write
  size_t sw_time = this->width_ * h * 16 / mhz + this->width_ * h * 2 / SPI_MAX_BLOCK_SIZE * SPI_SETUP_US * 2;
  // estimate time for multiple writes
  size_t mw_time = (w * h * 16) / mhz + w * h * 2 / ILI9XXX_TRANSFER_BUFFER_SIZE * SPI_SETUP_US;
  ESP_LOGV(TAG,
           "Start display(xlow:%d, ylow:%d, xhigh:%d, yhigh:%d, width:%d, "
           "height:%zu, mode=%d, 18bit=%d, sw_time=%zuus, mw_time=%zuus)",
           this->x_low_, this->y_low_, this->x_high_, this->y_high_, w, h, this->buffer_color_mode_,
           this->is_18bitdisplay_, sw_time, mw_time);
  auto now = millis();
  if (this->buffer_color_mode_ == BITS_16 && !this->is_18bitdisplay_ && sw_time < mw_time) {
    // 16 bit mode maps directly to display format
    ESP_LOGV(TAG, "Doing single write of %zu bytes", this->width_ * h * 2);
    set_addr_window_(0, this->y_low_, this->width_ - 1, this->y_high_);
    this->write_array(this->buffer_ + this->y_low_ * this->width_ * 2, h * this->width_ * 2);
  } else {
    ESP_LOGV(TAG, "Doing multiple write");
    uint8_t transfer_buffer[ILI9XXX_TRANSFER_BUFFER_SIZE];
    size_t rem = h * w;  // remaining number of pixels to write
    set_addr_window_(this->x_low_, this->y_low_, this->x_high_, this->y_high_);
    size_t idx = 0;    // index into transfer_buffer
    size_t pixel = 0;  // pixel number offset
    size_t pos = this->y_low_ * this->width_ + this->x_low_;
    while (rem-- != 0) {
      uint16_t color_val;
      switch (this->buffer_color_mode_) {
        case BITS_8:
          color_val = display::ColorUtil::color_to_565(display::ColorUtil::rgb332_to_color(this->buffer_[pos++]));
          break;
        case BITS_8_INDEXED:
          color_val = display::ColorUtil::color_to_565(
              display::ColorUtil::index8_to_color_palette888(this->buffer_[pos++], this->palette_));
          break;
        default:  // case BITS_16:
          color_val = (this->buffer_[pos * 2] << 8) + this->buffer_[pos * 2 + 1];
          pos++;
          break;
      }
      if (this->is_18bitdisplay_) {
        transfer_buffer[idx++] = (uint8_t) ((color_val & 0xF800) >> 8);  // Blue
        transfer_buffer[idx++] = (uint8_t) ((color_val & 0x7E0) >> 3);   // Green
        transfer_buffer[idx++] = (uint8_t) (color_val << 3);             // Red
      } else {
        put16_be(transfer_buffer + idx, color_val);
        idx += 2;
      }
      if (idx == sizeof(transfer_buffer)) {
        this->write_array(transfer_buffer, idx);
        idx = 0;
        App.feed_wdt();
      }
      // end of line? Skip to the next.
      if (++pixel == w) {
        pixel = 0;
        pos += this->width_ - w;
      }
    }
    // flush any balance.
    if (idx != 0) {
      this->write_array(transfer_buffer, idx);
    }
  }
  this->end_data_();
  ESP_LOGV(TAG, "Data write took %dms", (unsigned) (millis() - now));
  // invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}
float AXS15231Display::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

void AXS15231Display::setup() {
  ESP_LOGCONFIG(TAG, "setting up axs15231");

  ESP_LOGI(TAG, "setup pins");
  this->setup_pins_();
  ESP_LOGI(TAG, "setup lcd");
  this->init_lcd_();

  ESP_LOGI(TAG, "set madctl");
  this->set_madctl_();
  this->invalidate_();

  ESP_LOGI(TAG, "init internal buffer");
  this->init_internal_(this->get_buffer_length_());
  if (this->buffer_ == nullptr) {
    this->mark_failed();
  }

  ESP_LOGI(TAG, "set brightness");
  this->write_command_(AXS_LCD_WRDISBV, &this->brightness_, 1);

  this->setup_complete_ = true;
  ESP_LOGCONFIG(TAG, "axs15231 setup complete");
}

bool AXS15231Display::can_proceed() {
  return this->setup_complete_;
}

void AXS15231Display::dump_config() {
  ESP_LOGCONFIG("", "AXS15231 Display");
  ESP_LOGCONFIG(TAG, "  Height: %u", this->height_);
  ESP_LOGCONFIG(TAG, "  Width: %u", this->width_);
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  SPI Data rate: %dMHz", (unsigned) (this->data_rate_ / 1000000));
#ifdef USE_POWER_SUPPLY
  ESP_LOGCONFIG(TAG, "  Power Supply Configured: yes");
#endif
}

void AXS15231Display::fill(Color color) {
  uint16_t new_color = 0;
  this->x_low_ = 0;
  this->y_low_ = 0;
  this->x_high_ = this->get_width_internal() - 1;
  this->y_high_ = this->get_height_internal() - 1;

  new_color = display::ColorUtil::color_to_565(color);
  if (((uint8_t) (new_color >> 8)) == ((uint8_t) new_color)) {
    // Upper and lower is equal can use quicker memset operation
    memset(this->buffer_, (uint8_t) new_color, this->get_buffer_length_());
  } else {
    // Slower set of both buffers
    for (uint32_t i = 0; i < this->get_buffer_length_(); i = i + 2) {
      this->buffer_[i] = (uint8_t) (new_color >> 8);
      this->buffer_[i + 1] = (uint8_t) new_color;
    }
  }
}

display::DisplayType AXS15231Display::get_display_type() {
  return display::DisplayType::DISPLAY_TYPE_COLOR;
}

uint32_t AXS15231Display::get_buffer_length_() {
  return this->get_width_internal() * this->get_height_internal() * 2;
}

int AXS15231Display::get_width_internal() {
  return this->width_;
}

int AXS15231Display::get_height_internal() {
  return this->height_;
}

void AXS15231Display::set_reset_pin(GPIOPin *reset_pin) {
  this->reset_pin_ = reset_pin;
}

void AXS15231Display::set_backlight_pin(GPIOPin *backlight_pin) {
  this->backlight_pin_ = backlight_pin;
}

void AXS15231Display::set_width(uint16_t width) {
  this->width_ = width;
}

void AXS15231Display::set_dimensions(uint16_t width, uint16_t height) {
  this->width_ = width;
  this->height_ = height;
}

void AXS15231Display::set_mirror_x(bool mirror_x) {
  this->mirror_x_ = mirror_x;
}

void AXS15231Display::set_mirror_y(bool mirror_y) {
  this->mirror_y_ = mirror_y;
}

void AXS15231Display::set_swap_xy(bool swap_xy) {
  this->swap_xy_ = swap_xy;
}

void AXS15231Display::set_brightness(uint8_t brightness) {
  this->brightness_ = brightness;

  if (this->setup_complete_) {
    this->write_command_(AXS_LCD_WRDISBV, &this->brightness_, 1);
  }
}

void AXS15231Display::set_offsets(int16_t offset_x, int16_t offset_y) {
  this->offset_x_ = offset_x;
  this->offset_y_ = offset_y;
}

void AXS15231Display::setup_pins_() {
  if (this->backlight_pin_ != nullptr) {
    this->backlight_pin_->setup();
    this->backlight_pin_->digital_write(true);
  }

  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
  }

  this->spi_setup();

  this->reset_();
}

void AXS15231Display::set_madctl_() {
// custom x/y transform and color order
  uint8_t mad = MADCTL_RGB;
  if (this->swap_xy_)
    mad |= MADCTL_MV;
  if (this->mirror_x_)
    mad |= MADCTL_MX;
  if (this->mirror_y_)
    mad |= MADCTL_MY;
  this->command(ILI9XXX_MADCTL);
  this->data(mad);
  esph_log_d(TAG, "Wrote MADCTL 0x%02X", mad);
}

void AXS15231Display::init_lcd_() {
  const lcd_cmd_t *lcd_init = AXS_QSPI_INIT_NEW;
  for (int i = 0; i < sizeof(AXS_QSPI_INIT_NEW) / sizeof(lcd_cmd_t); ++i) {
    this->write_command_(lcd_init[i].cmd, (uint8_t *)lcd_init[i].data, lcd_init[i].len & 0x3f);
    if (lcd_init[i].len & 0x80)
        delay(150);
    if (lcd_init[i].len & 0x40)
        delay(20);
  }
}

void AXS15231Display::reset_() {
  if (this->reset_pin_  == nullptr) {
    return;
  }

  this->reset_pin_->digital_write(true);
  delay(20);
  this->reset_pin_->digital_write(false);
  delay(20);
  this->reset_pin_->digital_write(true);
  delay(20);
}

void AXS15231Display::write_command_(uint8_t cmd, const uint8_t *bytes, size_t len) {
  this->enable();
  this->write_cmd_addr_data(8, 0x02, 24, cmd << 8, bytes, len);
  this->disable();
}

void AXS15231Display::write_command_(uint8_t cmd, uint8_t data) { this->write_command_(cmd, &data, 1); }

void AXS15231Display::write_command_(uint8_t cmd) { this->write_command_(cmd, &cmd, 0); }

void AXS15231Display::set_addr_window_(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  x1 += this->offset_x_;
  x2 += this->offset_x_;
  y1 += this->offset_y_;
  y2 += this->offset_y_;
  this->command(ILI9XXX_CASET);
  this->data(x1 >> 8);
  this->data(x1 & 0xFF);
  this->data(x2 >> 8);
  this->data(x2 & 0xFF);
  this->command(ILI9XXX_PASET);  // Page address set
  this->data(y1 >> 8);
  this->data(y1 & 0xFF);
  this->data(y2 >> 8);
  this->data(y2 & 0xFF);
  this->command(ILI9XXX_RAMWR);  // Write to RAM
  this->start_data_();
}

void AXS15231Display::display_() {
  // check if something was displayed
  if ((this->x_high_ < this->x_low_) || (this->y_high_ < this->y_low_)) {
    return;
  }

  // we will only update the changed rows to the display
  size_t const w = this->x_high_ - this->x_low_ + 1;
  size_t const h = this->y_high_ - this->y_low_ + 1;

  size_t mhz = this->data_rate_ / 1000000;
  // estimate time for a single write
  size_t sw_time = this->width_ * h * 16 / mhz + this->width_ * h * 2 / SPI_MAX_BLOCK_SIZE * SPI_SETUP_US * 2;
  // estimate time for multiple writes
  size_t mw_time = (w * h * 16) / mhz + w * h * 2 / ILI9XXX_TRANSFER_BUFFER_SIZE * SPI_SETUP_US;
  ESP_LOGV(TAG,
           "Start display(xlow:%d, ylow:%d, xhigh:%d, yhigh:%d, width:%d, "
           "height:%zu, mode=%d, 18bit=%d, sw_time=%zuus, mw_time=%zuus)",
           this->x_low_, this->y_low_, this->x_high_, this->y_high_, w, h, this->buffer_color_mode_,
           this->is_18bitdisplay_, sw_time, mw_time);
  auto now = millis();
  if (this->buffer_color_mode_ == BITS_16 && !this->is_18bitdisplay_ && sw_time < mw_time) {
    // 16 bit mode maps directly to display format
    ESP_LOGV(TAG, "Doing single write of %zu bytes", this->width_ * h * 2);
    set_addr_window_(0, this->y_low_, this->width_ - 1, this->y_high_);
    this->write_array(this->buffer_ + this->y_low_ * this->width_ * 2, h * this->width_ * 2);
  } else {
    ESP_LOGV(TAG, "Doing multiple write");
    uint8_t transfer_buffer[ILI9XXX_TRANSFER_BUFFER_SIZE];
    size_t rem = h * w;  // remaining number of pixels to write
    set_addr_window_(this->x_low_, this->y_low_, this->x_high_, this->y_high_);
    size_t idx = 0;    // index into transfer_buffer
    size_t pixel = 0;  // pixel number offset
    size_t pos = this->y_low_ * this->width_ + this->x_low_;
    while (rem-- != 0) {
      uint16_t color_val;
      switch (this->buffer_color_mode_) {
        case BITS_8:
          color_val = display::ColorUtil::color_to_565(display::ColorUtil::rgb332_to_color(this->buffer_[pos++]));
          break;
        case BITS_8_INDEXED:
          color_val = display::ColorUtil::color_to_565(
              display::ColorUtil::index8_to_color_palette888(this->buffer_[pos++], this->palette_));
          break;
        default:  // case BITS_16:
          color_val = (this->buffer_[pos * 2] << 8) + this->buffer_[pos * 2 + 1];
          pos++;
          break;
      }
      if (this->is_18bitdisplay_) {
        transfer_buffer[idx++] = (uint8_t) ((color_val & 0xF800) >> 8);  // Blue
        transfer_buffer[idx++] = (uint8_t) ((color_val & 0x7E0) >> 3);   // Green
        transfer_buffer[idx++] = (uint8_t) (color_val << 3);             // Red
      } else {
        put16_be(transfer_buffer + idx, color_val);
        idx += 2;
      }
      if (idx == sizeof(transfer_buffer)) {
        this->write_array(transfer_buffer, idx);
        idx = 0;
        App.feed_wdt();
      }
      // end of line? Skip to the next.
      if (++pixel == w) {
        pixel = 0;
        pos += this->width_ - w;
      }
    }
    // flush any balance.
    if (idx != 0) {
      this->write_array(transfer_buffer, idx);
    }
  }
  this->end_data_();
  ESP_LOGV(TAG, "Data write took %dms", (unsigned) (millis() - now));
  // invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}


void AXS15231Display::invalidate_() {
  // invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}

void AXS15231Display::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0) {
    return;
  }
  if (!this->check_buffer_())
    return;
  uint32_t pos = (y * width_) + x;
  uint16_t new_color;
  bool updated = false;
  switch (this->buffer_color_mode_) {
    case BITS_8_INDEXED:
      new_color = display::ColorUtil::color_to_index8_palette888(color, this->palette_);
      break;
    case BITS_16:
      pos = pos * 2;
      new_color = display::ColorUtil::color_to_565(color, display::ColorOrder::COLOR_ORDER_RGB);
      if (this->buffer_[pos] != (uint8_t) (new_color >> 8)) {
        this->buffer_[pos] = (uint8_t) (new_color >> 8);
        updated = true;
      }
      pos = pos + 1;
      new_color = new_color & 0xFF;
      break;
    default:
      new_color = display::ColorUtil::color_to_332(color, display::ColorOrder::COLOR_ORDER_RGB);
      break;
  }

  if (this->buffer_[pos] != new_color) {
    this->buffer_[pos] = new_color;
    updated = true;
  }
  if (updated) {
    // low and high watermark may speed up drawing from buffer
    if (x < this->x_low_)
      this->x_low_ = x;
    if (y < this->y_low_)
      this->y_low_ = y;
    if (x > this->x_high_)
      this->x_high_ = x;
    if (y > this->y_high_)
      this->y_high_ = y;
  }
}

void ILI9XXXDisplay::update() {
  if (this->prossing_update_) {
    this->need_update_ = true;
    return;
  }
  this->prossing_update_ = true;
  do {
    this->need_update_ = false;
    this->do_update_();
  } while (this->need_update_);
  this->prossing_update_ = false;
  this->display_();
}


#endif
}  // namespace axs15231
}  // namespace esphome
