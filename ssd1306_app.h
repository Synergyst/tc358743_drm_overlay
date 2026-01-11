#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { LOG_DEBUG=0, LOG_INFO=1, LOG_WARN=2, LOG_ERROR=3 } log_level_t;
typedef void (*log_fn_t)(void* user, log_level_t lvl, const char* msg);

typedef struct { log_fn_t fn; void* user; } logger_t;

typedef enum { CHARGE_ICON_PLUG=0, CHARGE_ICON_PLUS=1, CHARGE_ICON_CHECK=2, CHARGE_ICON_BOLT_H=3 } ChargeIconStyle;

typedef uint32_t field_mask_t;

typedef struct {
  // host-controlled stop flag; loop exits when stop_flag != NULL and *stop_flag != 0
  volatile int* stop_flag;

  // OLED basic settings
  uint8_t i2c_node_address;     // /dev/i2c-<n>
  char oled_type[10];           // "128x64" "128x32" "64x48" or "" to load persisted resolution
  int orientation;              // -1, 0, 180
  int inverted;                 // -1, 0, 1
  int display;                  // -1, 0, 1

  // modes (choose one)
  int animate;
  unsigned frame_delay_ms;

  int test_frame;

  int power_meter;
  int ina_bus;
  int ina_addr;
  unsigned ina_period_ms;

  char fields_arg[256];         // optional; parsed into fields_mask if non-empty
  field_mask_t fields_mask;

  int show_batt_icon;
  int invert_charge;
  int batt_x_offset;            // -1 center
  int batt_width_px;            // 14..24
  int flash_icon;
  ChargeIconStyle icon_style;

  int video_monitor;
  char video_devs[256];
  unsigned video_poll_ms;
  int vid_icon_x_offset;

  // GPS/NMEA
  char nmea_arg[192];           // "dev[,baud[,timeout]]"
  char tz_arg[64];              // timezone string

  // Logging: TCP server inside this module (broadcast)
  int log_tcp_enable;           // default 1
  char log_bind_ip[64];         // default "0.0.0.0"
  int log_port;                 // default 9090

  // optional additional logger callback (still non-console; default NULL)
  logger_t logger;
} ssd1306_app_cfg_t;

void ssd1306_app_cfg_init(ssd1306_app_cfg_t* cfg);

// runs configured mode; blocks for loop modes until stop_flag is set
int ssd1306_app_run(const ssd1306_app_cfg_t* cfg);

#ifdef __cplusplus
}
#endif
