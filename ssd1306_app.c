#define _GNU_SOURCE
#include "ssd1306_app.h"

#include <stdarg.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>

#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>

#include "ssd1306.h"
#include "linux_i2c.h"
#include "image.h"
#include "nmea_miniparser.h"

// Persistent resolution file used by the library
static const char* RES_FILE = "/tmp/.ssd1306_oled_type";

#define SHOULD_STOP(cfg) ((cfg) && (cfg)->stop_flag && *((cfg)->stop_flag))

// ---------------- TCP log server (broadcast) ----------------
// - binds cfg->log_bind_ip:cfg->log_port (defaults 0.0.0.0:9090)
// - accepts clients and broadcasts log lines
// - keeps small ring buffer so new clients get recent lines

#define LOG_RING_LINES 200
#define LOG_LINE_MAX   512
#define LOG_MAX_CLIENTS 16

typedef struct {
  int enabled;
  int port;
  char bind_ip[64];

  int srv_fd;
  pthread_t thr;
  pthread_mutex_t mu;

  int clients[LOG_MAX_CLIENTS];
  int client_count;

  // ring buffer of recent lines
  char ring[LOG_RING_LINES][LOG_LINE_MAX];
  int ring_head; // next write
  int ring_size; // number valid

  volatile int stop;
} tcp_log_t;

static void tcp_log_broadcast_locked(tcp_log_t* t, const char* line) {
  // best-effort send; drop dead clients
  for (int i = 0; i < t->client_count; ) {
    int fd = t->clients[i];
    ssize_t n = send(fd, line, strlen(line), MSG_NOSIGNAL);
    if (n < 0) {
      close(fd);
      t->clients[i] = t->clients[t->client_count - 1];
      t->client_count--;
      continue;
    }
    i++;
  }
}

static void tcp_log_push(tcp_log_t* t, const char* msg) {
  if (!t || !t->enabled) return;
  char line[LOG_LINE_MAX];
  // one line per message, always newline terminated
  snprintf(line, sizeof(line), "%s\n", msg);

  pthread_mutex_lock(&t->mu);
  // ring
  snprintf(t->ring[t->ring_head], LOG_LINE_MAX, "%s", line);
  t->ring_head = (t->ring_head + 1) % LOG_RING_LINES;
  if (t->ring_size < LOG_RING_LINES) t->ring_size++;

  tcp_log_broadcast_locked(t, line);
  pthread_mutex_unlock(&t->mu);
}

static void* tcp_log_thread(void* arg) {
  tcp_log_t* t = (tcp_log_t*)arg;
  if (!t) return NULL;

  struct sockaddr_in addr; memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons((uint16_t)t->port);
  if (inet_pton(AF_INET, t->bind_ip, &addr.sin_addr) != 1) {
    // invalid bind IP -> fallback
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  }

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return NULL;

  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    close(fd);
    return NULL;
  }
  if (listen(fd, 8) != 0) {
    close(fd);
    return NULL;
  }

  t->srv_fd = fd;

  while (!t->stop) {
    struct timeval tv = {0, 250000}; // 250ms
    fd_set rfds; FD_ZERO(&rfds); FD_SET(fd, &rfds);
    int r = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (r <= 0) continue;
    if (!FD_ISSET(fd, &rfds)) continue;

    struct sockaddr_in caddr; socklen_t clen = sizeof(caddr);
    int cfd = accept(fd, (struct sockaddr*)&caddr, &clen);
    if (cfd < 0) continue;

    // socket options: keep small latency; ignore Nagle
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    pthread_mutex_lock(&t->mu);

    if (t->client_count >= LOG_MAX_CLIENTS) {
      close(cfd);
      pthread_mutex_unlock(&t->mu);
      continue;
    }

    // send ring buffer to new client (oldest -> newest)
    int start = (t->ring_head - t->ring_size);
    if (start < 0) start += LOG_RING_LINES;
    for (int i = 0; i < t->ring_size; ++i) {
      int idx = (start + i) % LOG_RING_LINES;
      send(cfd, t->ring[idx], strlen(t->ring[idx]), MSG_NOSIGNAL);
    }

    t->clients[t->client_count++] = cfd;
    pthread_mutex_unlock(&t->mu);
  }

  close(fd);
  t->srv_fd = -1;
  return NULL;
}

static void tcp_log_init(tcp_log_t* t, const ssd1306_app_cfg_t* cfg) {
  memset(t, 0, sizeof(*t));
  pthread_mutex_init(&t->mu, NULL);
  t->srv_fd = -1;

  t->enabled = (cfg && cfg->log_tcp_enable) ? 1 : 0;
  t->port = (cfg && cfg->log_port > 0) ? cfg->log_port : 9090;
  snprintf(t->bind_ip, sizeof(t->bind_ip), "%s", (cfg && cfg->log_bind_ip[0]) ? cfg->log_bind_ip : "0.0.0.0");
}

static void tcp_log_start(tcp_log_t* t) {
  if (!t || !t->enabled) return;
  t->stop = 0;
  pthread_create(&t->thr, NULL, tcp_log_thread, t);
}

static void tcp_log_stop(tcp_log_t* t) {
  if (!t || !t->enabled) return;
  t->stop = 1;
  pthread_join(t->thr, NULL);

  pthread_mutex_lock(&t->mu);
  for (int i = 0; i < t->client_count; ++i) close(t->clients[i]);
  t->client_count = 0;
  pthread_mutex_unlock(&t->mu);

  pthread_mutex_destroy(&t->mu);
}

static inline void LOGF(tcp_log_t* t, const logger_t* extra, log_level_t lvl, const char* fmt, ...) {
  char b[1024];
  va_list ap; va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);

  // tcp log always (if enabled)
  if (t && t->enabled) {
    const char* p = (lvl==LOG_ERROR)?"ERR":(lvl==LOG_WARN)?"WRN":(lvl==LOG_INFO)?"INF":"DBG";
    char line[1100];
    snprintf(line, sizeof(line), "[%s] %s", p, b);
    tcp_log_push(t, line);
  }

  // optional extra callback (still no console here)
  if (extra && extra->fn) extra->fn(extra->user, lvl, b);
}

// ---------------- Resolution helper ----------------
static int read_saved_resolution(int* out_cols, int* out_lines) {
  if (!out_cols || !out_lines) return -1;
  *out_cols = 0;
  *out_lines = 0;
  FILE* fp = fopen(RES_FILE, "r");
  if (!fp) return -1;
  int cols = 0, lines = 0;
  if (fscanf(fp, "%dx%d", &cols, &lines) != 2 &&
      fscanf(fp, "%ux%u", (unsigned*)&cols, (unsigned*)&lines) != 2) {
    fclose(fp);
    return -1;
  }
  fclose(fp);
  if (cols <= 0 || lines <= 0) return -1;
  *out_cols = cols;
  *out_lines = lines;
  return 0;
}

// ---------------- INA219 (Power Meter) ----------------
#define INA219_REG_CONFIG          0x00
#define INA219_REG_SHUNT_VOLTAGE   0x01
#define INA219_REG_BUS_VOLTAGE     0x02
#define INA219_REG_POWER           0x03
#define INA219_REG_CURRENT         0x04
#define INA219_REG_CALIBRATION     0x05

#define INA219_RANGE_16V           0x00
#define INA219_GAIN_80MV           0x01
#define INA219_ADC_12BIT_32S       0x0D
#define INA219_MODE_SANDB_CONT     0x07

typedef struct {
  int      fd;
  uint8_t  addr;
  double   current_lsb;  // mA per bit
  double   power_lsb;    // W per bit
  uint16_t cal_value;
} ina219_t;

static int ina219_open(ina219_t* dev, int bus, uint8_t addr) {
  if (!dev) return -1;
  memset(dev, 0, sizeof(*dev));
  dev->addr = addr;
  char filename[32];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
  dev->fd = open(filename, O_RDWR);
  if (dev->fd < 0) return -1;
  if (ioctl(dev->fd, I2C_SLAVE, addr) < 0) {
    close(dev->fd);
    dev->fd = -1;
    return -1;
  }
  return 0;
}

static void ina219_close(ina219_t* dev) {
  if (dev && dev->fd >= 0) { close(dev->fd); dev->fd = -1; }
}

static int ina219_write16(ina219_t* dev, uint8_t reg, uint16_t val) {
  uint8_t buf[3];
  buf[0] = reg;
  buf[1] = (uint8_t)((val >> 8) & 0xFF);
  buf[2] = (uint8_t)(val & 0xFF);
  return (write(dev->fd, buf, 3) == 3) ? 0 : -1;
}

static int ina219_read16(ina219_t* dev, uint8_t reg, uint16_t* out) {
  if (!out) return -1;
  uint8_t r = reg;
  if (write(dev->fd, &r, 1) != 1) return -1;
  uint8_t buf[2];
  if (read(dev->fd, buf, 2) != 2) return -1;
  *out = (uint16_t)((buf[0] << 8) | buf[1]);
  return 0;
}

static int ina219_calibrate_16V_5A(ina219_t* dev) {
  if (!dev) return -1;
  dev->cal_value   = 26868;
  dev->current_lsb = 0.1524;
  dev->power_lsb   = 0.003048;

  if (ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value) != 0) return -1;

  uint16_t config = 0;
  config |= (uint16_t)(INA219_RANGE_16V & 0x01) << 13;
  config |= (uint16_t)(INA219_GAIN_80MV & 0x03) << 11;
  config |= (uint16_t)(INA219_ADC_12BIT_32S & 0x0F) << 7;
  config |= (uint16_t)(INA219_ADC_12BIT_32S & 0x0F) << 3;
  config |= (uint16_t)(INA219_MODE_SANDB_CONT & 0x07);

  return (ina219_write16(dev, INA219_REG_CONFIG, config) == 0) ? 0 : -1;
}

static double ina219_get_shunt_voltage_V(ina219_t* dev) {
  uint16_t raw = 0;
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  if (ina219_read16(dev, INA219_REG_SHUNT_VOLTAGE, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * 1e-5;
}

static double ina219_get_bus_voltage_V(ina219_t* dev) {
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_BUS_VOLTAGE, &raw) != 0) return 0.0;
  raw >>= 3;
  return (double)raw * 0.004;
}

static double ina219_get_current_mA(ina219_t* dev) {
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_CURRENT, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * dev->current_lsb;
}

static double ina219_get_power_W(ina219_t* dev) {
  ina219_write16(dev, INA219_REG_CALIBRATION, dev->cal_value);
  uint16_t raw = 0;
  if (ina219_read16(dev, INA219_REG_POWER, &raw) != 0) return 0.0;
  int16_t sraw = (raw > 32767) ? (int16_t)(raw - 65535) : (int16_t)raw;
  return (double)sraw * dev->power_lsb;
}

// ---------------- SSD1306 helpers (page-level blit) ----------------
static uint8_t ssd1306_blit_full_pages(const uint8_t* pages, int w, int h) {
  if (!pages || w <= 0 || h <= 0 || (h % 8) != 0) return 1;
  int page_count = h / 8;
  uint8_t tx[1 + 128];
  tx[0] = SSD1306_DATA_CONTROL_BYTE;
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  for (int p = 0; p < page_count; ++p) {
    if (ssd1306_oled_set_XY(0, (uint8_t)p) != 0) return 1;
    memcpy(&tx[1], &pages[p * w], (size_t)w);
    if (_i2c_write(tx, (int16_t)(1 + w)) != 0) return 1;
  }
  return 0;
}

static uint8_t ssd1306_blit_from_01(const uint8_t* img01, int w, int h) {
  if (!img01 || w <= 0 || h <= 0 || (h % 8) != 0 || w > 128) return 1;
  static uint8_t pagebuf[128 * 64 / 8];
  for (int p = 0; p < h / 8; ++p) {
    for (int x = 0; x < w; ++x) {
      uint8_t b = 0;
      for (int bit = 0; bit < 8; ++bit) {
        int y = p * 8 + bit;
        if (img01[y * w + x]) b |= (uint8_t)(1u << bit);
      }
      pagebuf[p * w + x] = b;
    }
  }
  return ssd1306_blit_full_pages(pagebuf, w, h);
}

static uint8_t oled_write_page_row(uint8_t x, uint8_t page, const uint8_t* bytes, uint8_t w) {
  if (!bytes || w == 0) return 1;
  if (w > 128) return 1;
  if (ssd1306_oled_set_XY(x, page) != 0) return 1;
  uint8_t tx[1 + 128];
  tx[0] = SSD1306_DATA_CONTROL_BYTE;
  memcpy(&tx[1], bytes, w);
  return _i2c_write(tx, (int16_t)(1 + w));
}

// ---------------- Animation demo ----------------
static void build_row_vertical_bar(uint8_t* row, int cols, int bar_x) {
  memset(row, 0x00, (size_t)cols);
  if (bar_x >= 0 && bar_x < cols) row[bar_x] = 0xFF;
}
static void build_row_horizontal_line(uint8_t* row, int cols, int page, int y) {
  memset(row, 0x00, (size_t)cols);
  int target_page = y >> 3;
  if (page == target_page) {
    uint8_t mask = (uint8_t)(1u << (y & 7));
    memset(row, mask, (size_t)cols);
  }
}
static void build_row_diagonal_pixel(uint8_t* row, int cols, int page, int x, int y) {
  memset(row, 0x00, (size_t)cols);
  int target_page = y >> 3;
  if (page == target_page && x >= 0 && x < cols) {
    uint8_t mask = (uint8_t)(1u << (y & 7));
    row[x] = mask;
  }
}

static void run_animation_loop(const ssd1306_app_cfg_t* cfg, int cols, int lines, unsigned frame_delay_ms) {
  if (cols <= 0 || lines <= 0) return;
  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  ssd1306_oled_clear_screen();
  const int pages = lines / 8;
  uint8_t row_buf[128];
  if (cols > (int)sizeof(row_buf)) cols = (int)sizeof(row_buf);
  uint32_t t = 0;
  while (!SHOULD_STOP(cfg)) {
    int span = (cols > lines ? cols : lines);
    int mode = (t / (uint32_t)span) % 3;
    int bar_x = (int)(t % (uint32_t)cols);
    int y     = (int)(t % (uint32_t)lines);
    int x     = (int)(t % (uint32_t)cols);
    for (int p = 0; p < pages; ++p) {
      switch (mode) {
        case 0: build_row_vertical_bar(row_buf, cols, bar_x); break;
        case 1: build_row_horizontal_line(row_buf, cols, p, y); break;
        case 2:
        default: build_row_diagonal_pixel(row_buf, cols, p, x, y); break;
      }
      if (oled_write_page_row(0, (uint8_t)p, row_buf, (uint8_t)cols) != 0) return;
    }
    t++;
    if (frame_delay_ms == 0) frame_delay_ms = 1;
    usleep(frame_delay_ms * 1000);
  }
}

// ---------------- Power Meter UI with differential updates ----------------
#define METER_FONT_SIZE SSD1306_FONT_SMALL
static inline int meter_char_width(void) { return (METER_FONT_SIZE == SSD1306_FONT_SMALL) ? 6 : 8; }

static void meter_draw_run(uint8_t row_page, int x_chars, const char* s, int run_len) {
  if (run_len <= 0) return;
  int cw = meter_char_width();
  int x_px = x_chars * cw;
  ssd1306_oled_set_XY((uint8_t)x_px, row_page);
  char buf[64];
  while (run_len > 0) {
    int n = (run_len < (int)sizeof(buf) - 1) ? run_len : (int)sizeof(buf) - 1;
    memcpy(buf, s, (size_t)n);
    buf[n] = '\0';
    ssd1306_oled_write_line(METER_FONT_SIZE, buf);
    s += n;
    x_chars += n;
    x_px = x_chars * cw;
    if (run_len > n) ssd1306_oled_set_XY((uint8_t)x_px, row_page);
    run_len -= n;
  }
}

static void meter_diff_and_draw(uint8_t row_page, int x_chars, char* prev, const char* now) {
  int old_len = (int)strlen(prev);
  int new_len = (int)strlen(now);
  int max_len = (old_len > new_len) ? old_len : new_len;
  int i = 0;
  while (i < max_len) {
    char new_ch = (i < new_len) ? now[i] : ' ';
    char old_ch = (i < old_len) ? prev[i] : ' ';
    if (new_ch == old_ch) { i++; continue; }
    int run_start = i;
    int run_len = 1;
    i++;
    while (i < max_len) {
      char new_ch_i = (i < new_len) ? now[i] : ' ';
      char old_ch_i = (i < old_len) ? prev[i] : ' ';
      if (new_ch_i != old_ch_i) { run_len++; i++; }
      else break;
    }
    int seg1_len = 0;
    if (run_start < new_len) {
      seg1_len = new_len - run_start;
      if (seg1_len > run_len) seg1_len = run_len;
      meter_draw_run(row_page, x_chars + run_start, &now[run_start], seg1_len);
    }
    int seg2_len = run_len - seg1_len;
    if (seg2_len > 0) {
      int offset = 0;
      char spaces[64];
      memset(spaces, ' ', sizeof(spaces));
      while (seg2_len > 0) {
        int chunk = (seg2_len > (int)sizeof(spaces)) ? (int)sizeof(spaces) : seg2_len;
        meter_draw_run(row_page, x_chars + run_start + seg1_len + offset, spaces, chunk);
        seg2_len -= chunk;
        offset += chunk;
      }
    }
  }
  strncpy(prev, now, (size_t)max_len);
  prev[new_len] = '\0';
}

// ---------------- Dynamic fields and battery + video icons ----------------
typedef uint32_t field_mask_t;
#define FLD_PSU        (1u << 0)
#define FLD_LOAD       (1u << 1)
#define FLD_CURR       (1u << 2)
#define FLD_POWER      (1u << 3)
#define FLD_PCT        (1u << 4)
#define FLD_NONE       (1u << 5)
#define FLD_VID        (1u << 6)
#define FLD_FMT        (1u << 7)
#define FLD_UTC_TIME   (1u << 8)
#define FLD_UTC_DATE   (1u << 9)
#define FLD_LOCAL_TIME (1u << 10)
#define FLD_LOCAL_DATE (1u << 11)
#define FLD_LAT        (1u << 12)
#define FLD_LON        (1u << 13)
#define FLD_SPEED      (1u << 14)
#define FLD_ALT        (1u << 15)
#define FLD_SATS       (1u << 16)
#define FLD_COURSE     (1u << 17)
#define FLD_HDOP       (1u << 18)

static inline field_mask_t default_field_mask(void) {
  return (FLD_PSU | FLD_LOAD | FLD_CURR | FLD_POWER | FLD_PCT | FLD_VID | FLD_FMT);
}

static field_mask_t parse_enabled_fields(const char* s) {
  if (!s || !*s) return default_field_mask();
  field_mask_t mask = 0;
  int saw_none = 0;
  char buf[256];
  strncpy(buf, s, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* tok = strtok(buf, ",");
  while (tok) {
    while (*tok == ' ' || *tok == '\t') tok++;
    if (!strcasecmp(tok, "none")) { saw_none = 1; mask = FLD_NONE; break; }
    if (!strcasecmp(tok, "psu"))         mask |= FLD_PSU;
    else if (!strcasecmp(tok, "load"))   mask |= FLD_LOAD;
    else if (!strcasecmp(tok, "curr"))   mask |= FLD_CURR;
    else if (!strcasecmp(tok, "power"))  mask |= FLD_POWER;
    else if (!strcasecmp(tok, "pct"))    mask |= FLD_PCT;
    else if (!strcasecmp(tok, "vid"))    mask |= FLD_VID;
    else if (!strcasecmp(tok, "fmt"))    mask |= FLD_FMT;
    else if (!strcasecmp(tok, "utc_time") || !strcasecmp(tok, "utc"))    mask |= FLD_UTC_TIME;
    else if (!strcasecmp(tok, "utc_date"))                                mask |= FLD_UTC_DATE;
    else if (!strcasecmp(tok, "local_time") || !strcasecmp(tok, "tz"))   mask |= FLD_LOCAL_TIME;
    else if (!strcasecmp(tok, "local_date") || !strcasecmp(tok, "date")) mask |= FLD_LOCAL_DATE;
    else if (!strcasecmp(tok, "lat"))                                     mask |= FLD_LAT;
    else if (!strcasecmp(tok, "lon"))                                     mask |= FLD_LON;
    else if (!strcasecmp(tok, "speed"))                                   mask |= FLD_SPEED;
    else if (!strcasecmp(tok, "alt"))                                     mask |= FLD_ALT;
    else if (!strcasecmp(tok, "sats") || !strcasecmp(tok, "satellites"))  mask |= FLD_SATS;
    else if (!strcasecmp(tok, "course"))                                  mask |= FLD_COURSE;
    else if (!strcasecmp(tok, "hdop"))                                    mask |= FLD_HDOP;
    tok = strtok(NULL, ",");
  }
  if (saw_none) return FLD_NONE;
  return (mask == 0) ? default_field_mask() : mask;
}

#define MAX_VDEVS 8
typedef struct { int count; char devs[MAX_VDEVS][1024]; } video_list_t;

static void trim_leading(char** s) { while (**s == ' ' || **s == '\t') (*s)++; }

static void build_video_list(const char* list, video_list_t* out) {
  if (!out) return;
  memset(out, 0, sizeof(*out));
  if (!list || !*list) { snprintf(out->devs[0], sizeof(out->devs[0]), "/dev/video0"); out->count = 1; return; }
  char buf[1024];
  strncpy(buf, list, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* tok = strtok(buf, ",");
  while (tok && out->count < MAX_VDEVS) {
    trim_leading(&tok);
    if (strncmp(tok, "/dev/video", 10) != 0) snprintf(out->devs[out->count], sizeof(out->devs[out->count]), "/dev/video%s", tok);
    else snprintf(out->devs[out->count], sizeof(out->devs[out->count]), "%s", tok);
    out->count++;
    tok = strtok(NULL, ",");
  }
  if (out->count == 0) { snprintf(out->devs[0], sizeof(out->devs[0]), "/dev/video0"); out->count = 1; }
}

static void meter_draw_static_labels_dyn_for_vlist(const video_list_t* vlist,
                                                   field_mask_t enabled_mask,
                                                   uint8_t max_rows,
                                                   const char* tz_label_opt) {
  if (!(enabled_mask & FLD_NONE)) {
    const char* lab_psu   = "SUPP(V):";
    const char* lab_load  = "LOAD(V):";
    const char* lab_curr  = "CUR(mA):";
    const char* lab_power = "POWR(W):";
    const char* lab_pct   = "BATT(%):";
    const char* lab_utc   = "UTC:";
    char lab_local[16]; lab_local[0] = 0;
    if (tz_label_opt && *tz_label_opt) snprintf(lab_local, sizeof(lab_local), "%.12s:", tz_label_opt);
    else snprintf(lab_local, sizeof(lab_local), "LOCAL:");
    const char* lab_speed = "SPD(kn):";
    const char* lab_alt   = "ALT(m):";
    const char* lab_sats  = "SATS:";
    const char* lab_course= "CRS(deg):";
    const char* lab_hdop  = "HDOP:";

    uint8_t row = 0;
    if ((enabled_mask & FLD_PSU)        && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_psu);   row++; }
    if ((enabled_mask & FLD_LOAD)       && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_load);  row++; }
    if ((enabled_mask & FLD_CURR)       && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_curr);  row++; }
    if ((enabled_mask & FLD_POWER)      && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_power); row++; }
    if ((enabled_mask & FLD_UTC_TIME)   && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_utc);   row++; }
    if ((enabled_mask & FLD_LOCAL_TIME) && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (lab_local[0]?lab_local:(char*)"LOCAL:")); row++; }
    if ((enabled_mask & FLD_UTC_DATE)   && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"UDATE:");  row++; }
    if ((enabled_mask & FLD_LOCAL_DATE) && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"LDATE:");  row++; }
    if ((enabled_mask & FLD_LAT)        && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"LAT:");    row++; }
    if ((enabled_mask & FLD_LON)        && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"LON:");    row++; }
    if ((enabled_mask & FLD_SPEED)      && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_speed); row++; }
    if ((enabled_mask & FLD_ALT)        && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_alt);   row++; }
    if ((enabled_mask & FLD_SATS)       && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_sats);  row++; }
    if ((enabled_mask & FLD_COURSE)     && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_course);row++; }
    if ((enabled_mask & FLD_HDOP)       && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_hdop);  row++; }

    if (vlist && vlist->count > 1) {
      for (int i = 0; i < vlist->count && row < max_rows; ++i) {
        char lab[16];
        snprintf(lab, sizeof(lab), "VID%d:", i);
        ssd1306_oled_set_XY(0, row);
        ssd1306_oled_write_line(METER_FONT_SIZE, lab);
        row++;
        if (row >= max_rows) break;
        snprintf(lab, sizeof(lab), "FMT%d:", i);
        ssd1306_oled_set_XY(0, row);
        ssd1306_oled_write_line(METER_FONT_SIZE, lab);
        row++;
      }
    } else {
      if ((enabled_mask & FLD_VID)      && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"VID:");   row++; }
      if ((enabled_mask & FLD_FMT)      && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)"FMT:");   row++; }
    }
    if ((enabled_mask & FLD_PCT)        && row < max_rows) { ssd1306_oled_set_XY(0, row); ssd1306_oled_write_line(METER_FONT_SIZE, (char*)lab_pct);   row++; }
  }
}

typedef struct {
  char psu[24], load[24], curr[24], power[24], pct[24];
  char vid[24], fmt[24];
  char utc_time[24], utc_date[24], local_time[24], local_date[24], lat[24], lon[24];
  char speed[24], alt[24], sats[24], course[24], hdop[24];
  int  initialized;
} meter_prev_t;

static uint8_t meter_update_values_dyn(meter_prev_t* st,
                                       const video_list_t* vlist,
                                       field_mask_t enabled_mask,
                                       uint8_t max_rows,
                                       int start_chars,
                                       double busV, double shuntV,
                                       double current_mA, double power_W,
                                       double percent,
                                       const char vid_texts[][24],
                                       const char fmt_texts[][24],
                                       int vid_count,
                                       const char* utc_time,
                                       const char* utc_date,
                                       const char* local_time,
                                       const char* local_date,
                                       const char* lat_str,
                                       const char* lon_str,
                                       const char* speed_kn_str,
                                       const char* alt_m_str,
                                       const char* sats_str,
                                       const char* course_deg_str,
                                       const char* hdop_str,
                                       const char* tz_label_opt) {
  if (!st) return 0;
  char now_psu[24], now_load[24], now_curr[24], now_power[24], now_pct[24];
  char now_vid[24], now_fmt[24];
  snprintf(now_psu,   sizeof(now_psu),   "%1.3f", busV + shuntV);
  snprintf(now_load,  sizeof(now_load),  "%1.3f", busV);
  snprintf(now_curr,  sizeof(now_curr),  "%d", (int)current_mA);
  snprintf(now_power, sizeof(now_power), "%2.3f", power_W);
  if (percent >= 90.0) snprintf(now_pct, sizeof(now_pct), "%3.1f", percent);
  else                 snprintf(now_pct, sizeof(now_pct), "%3.2f", percent);
  snprintf(now_vid,   sizeof(now_vid),   "%s", (vid_count > 0 && vid_texts) ? vid_texts[0] : "");
  snprintf(now_fmt,   sizeof(now_fmt),   "%s", (vid_count > 0 && fmt_texts) ? fmt_texts[0] : "");
  if (!st->initialized) {
    meter_draw_static_labels_dyn_for_vlist(vlist, enabled_mask, max_rows, tz_label_opt);
    st->psu[0] = st->load[0] = st->curr[0] = st->power[0] = st->pct[0] = st->vid[0] = st->fmt[0] = '\0';
    st->utc_time[0] = st->utc_date[0] = st->local_time[0] = st->local_date[0] = st->lat[0] = st->lon[0] = '\0';
    st->speed[0] = st->alt[0] = st->sats[0] = st->course[0] = st->hdop[0] = '\0';
    st->initialized = 1;
  }

  const int vid_label_chars_single = (int)strlen("VID:");
  const int fmt_label_chars_single = (int)strlen("FMT:");
  uint8_t row = 0;

  if ((enabled_mask & FLD_PSU)        && row < max_rows) { meter_diff_and_draw(row, start_chars, st->psu,       now_psu);   row++; }
  if ((enabled_mask & FLD_LOAD)       && row < max_rows) { meter_diff_and_draw(row, start_chars, st->load,      now_load);  row++; }
  if ((enabled_mask & FLD_CURR)       && row < max_rows) { meter_diff_and_draw(row, start_chars, st->curr,      now_curr);  row++; }
  if ((enabled_mask & FLD_POWER)      && row < max_rows) { meter_diff_and_draw(row, start_chars, st->power,     now_power); row++; }

  if ((enabled_mask & FLD_UTC_TIME)   && row < max_rows) { meter_diff_and_draw(row, start_chars, st->utc_time,   utc_time ? utc_time : "");     row++; }
  if ((enabled_mask & FLD_LOCAL_TIME) && row < max_rows) { meter_diff_and_draw(row, start_chars, st->local_time, local_time ? local_time : ""); row++; }
  if ((enabled_mask & FLD_UTC_DATE)   && row < max_rows) { meter_diff_and_draw(row, start_chars, st->utc_date,   utc_date ? utc_date : "");     row++; }
  if ((enabled_mask & FLD_LOCAL_DATE) && row < max_rows) { meter_diff_and_draw(row, start_chars, st->local_date, local_date ? local_date : ""); row++; }
  if ((enabled_mask & FLD_LAT)        && row < max_rows) { meter_diff_and_draw(row, start_chars, st->lat,       lat_str ? lat_str : "");       row++; }
  if ((enabled_mask & FLD_LON)        && row < max_rows) { meter_diff_and_draw(row, start_chars, st->lon,       lon_str ? lon_str : "");       row++; }

  if ((enabled_mask & FLD_SPEED)      && row < max_rows) { meter_diff_and_draw(row, start_chars, st->speed,  speed_kn_str  ? speed_kn_str  : ""); row++; }
  if ((enabled_mask & FLD_ALT)        && row < max_rows) { meter_diff_and_draw(row, start_chars, st->alt,    alt_m_str     ? alt_m_str     : ""); row++; }
  if ((enabled_mask & FLD_SATS)       && row < max_rows) { meter_diff_and_draw(row, start_chars, st->sats,   sats_str      ? sats_str      : ""); row++; }
  if ((enabled_mask & FLD_COURSE)     && row < max_rows) { meter_diff_and_draw(row, start_chars, st->course, course_deg_str? course_deg_str: ""); row++; }
  if ((enabled_mask & FLD_HDOP)       && row < max_rows) { meter_diff_and_draw(row, start_chars, st->hdop,   hdop_str      ? hdop_str      : ""); row++; }

  if (vlist && vlist->count > 1) {
    for (int i = 0; i < vid_count && row < max_rows; ++i) {
      int label_chars = (int)strlen("VID0:");
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), row);
      ssd1306_oled_write_line(METER_FONT_SIZE, vid_texts[i]);
      row++;
      if (row >= max_rows) break;
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), row);
      ssd1306_oled_write_line(METER_FONT_SIZE, fmt_texts[i]);
      row++;
    }
  } else {
    if ((enabled_mask & FLD_VID)      && row < max_rows) { meter_diff_and_draw(row, vid_label_chars_single, st->vid, now_vid); row++; }
    if ((enabled_mask & FLD_FMT)      && row < max_rows) { meter_diff_and_draw(row, fmt_label_chars_single, st->fmt, now_fmt); row++; }
  }
  if ((enabled_mask & FLD_PCT)        && row < max_rows) { meter_diff_and_draw(row, start_chars, st->pct, now_pct); row++; }
  return row;
}

// ---------------- Battery icon rendering ----------------
//typedef enum { CHARGE_ICON_PLUG=0, CHARGE_ICON_PLUS=1, CHARGE_ICON_CHECK=2, CHARGE_ICON_BOLT_H=3 } ChargeIconStyle;

static inline void carve_bit(uint8_t* cols, int width, int x, int y) {
  if (x < 0 || x >= width) return;
  if (y < 0 || y > 7) return;
  cols[x] &= (uint8_t)~(1u << y);
}

static void carve_hline(uint8_t* cols, int width, int x0, int x1, int y, int inner_left, int inner_right, int thick) {
  if (y < 0 || y > 7) return;
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (x0 < inner_left) x0 = inner_left;
  if (x1 > inner_right) x1 = inner_right;
  for (int x = x0; x <= x1; ++x) {
    for (int dy = -thick + 1; dy <= thick - 1; ++dy) {
      int yy = y + dy;
      if (yy > 0 && yy < 7) carve_bit(cols, width, x, yy);
    }
  }
}

static void carve_vline(uint8_t* cols, int width, int x, int y0, int y1, int inner_left, int inner_right, int thick) {
  if (x < inner_left || x > inner_right) return;
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  if (y0 < 1) y0 = 1;
  if (y1 > 6) y1 = 6;
  for (int y = y0; y <= y1; ++y) {
    for (int dx = -thick + 1; dx <= thick - 1; ++dx) {
      int xx = x + dx;
      if (xx >= inner_left && xx <= inner_right) carve_bit(cols, width, xx, y);
    }
  }
}

static inline void clamp_center(int* cx, int* cy, int inner_left, int inner_right, int top, int bottom) {
  if (*cx < inner_left + 2) *cx = inner_left + 2;
  if (*cx > inner_right - 2) *cx = inner_right - 2;
  if (*cy < top + 2) *cy = top + 2;
  if (*cy > bottom - 2) *cy = bottom - 2;
}

static void overlay_plug(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  for (int x = cx - 3; x <= cx; ++x) carve_vline(cols, width, x, cy - 2, cy + 3, inner_left, inner_right, 1);
  carve_vline(cols, width, cx + 3, cy - 1, cy - 1, inner_left, inner_right, 2);
  carve_vline(cols, width, cx + 3, cy + 2, cy + 2, inner_left, inner_right, 2);
  carve_hline(cols, width, cx - 1, cx, cy, inner_left, inner_right, 1);
}

static void overlay_plus(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  carve_hline(cols, width, cx - 2, cx + 2, cy, inner_left, inner_right, 1);
  carve_vline(cols, width, cx, cy - 2, cy + 2, inner_left, inner_right, 1);
}

static void overlay_check(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  carve_bit(cols, width, cx - 3, cy + 1);
  carve_bit(cols, width, cx - 2, cy);
  carve_bit(cols, width, cx - 1, cy);
  carve_bit(cols, width, cx,     cy - 1);
  carve_bit(cols, width, cx + 1, cy - 2);
  carve_bit(cols, width, cx + 2, cy - 3);
  carve_hline(cols, width, cx - 3, cx - 1, cy, inner_left, inner_right, 1);
  carve_hline(cols, width, cx, cx + 2, cy - 2, inner_left, inner_right, 1);
}

static void overlay_bolt_h(uint8_t* cols, int width, int inner_left, int inner_right, int top, int bottom, int cx, int cy) {
  clamp_center(&cx, &cy, inner_left, inner_right, top, bottom);
  carve_bit(cols, width, cx - 3, cy - 2);
  carve_bit(cols, width, cx - 2, cy - 1);
  carve_bit(cols, width, cx - 1, cy);
  carve_bit(cols, width, cx,     cy);
  carve_bit(cols, width, cx + 1, cy + 1);
  carve_bit(cols, width, cx + 2, cy + 2);
  carve_hline(cols, width, cx - 3, cx - 1, cy - 1, inner_left, inner_right, 1);
  carve_hline(cols, width, cx, cx + 2, cy + 1, inner_left, inner_right, 1);
}

static inline int calc_fill_w(int inner_w, int level) {
  if (level <= 0) return 0;
  if (level >= 100) return inner_w;
  return (inner_w * level) / 100;
}

static void draw_battery_icon_with_style(uint8_t last_page, int x_px, int width_px, int level, int charging, ChargeIconStyle style) {
  if (width_px < 14) width_px = 14;
  if (width_px > 24) width_px = 24;
  if (level < 0) level = 0;
  if (level > 100) level = 100;

  uint8_t cols[24];
  memset(cols, 0x00, (size_t)width_px);

  const int left = 0, right = width_px - 1, top = 0, bottom = 7;
  const int tip_w = 2, tip_x0 = right - (tip_w - 1), tip_y0 = 2, tip_y1 = 5;
  const int body_left = left, body_right = tip_x0 - 1;
  const int inner_left = body_left + 1, inner_right = body_right - 1;

  for (int x = body_left; x <= body_right; ++x) { cols[x] |= (1u << top); cols[x] |= (1u << bottom); }
  for (int y = top; y <= bottom; ++y) { cols[body_left] |= (1u << y); cols[body_right] |= (1u << y); }
  for (int x = tip_x0; x <= right; ++x) for (int y = tip_y0; y <= tip_y1; ++y) cols[x] |= (1u << y);

  const int inner_w = (inner_right >= inner_left) ? (inner_right - inner_left + 1) : 0;
  const int fill_w = calc_fill_w(inner_w, level);

  for (int x = inner_left; x < inner_left + fill_w; ++x) for (int y = top + 1; y <= bottom - 1; ++y) cols[x] |= (1u << y);

  if (charging) {
    int cx = inner_left + inner_w / 2;
    int cy = (top + bottom) / 2;
    switch (style) {
      case CHARGE_ICON_PLUG:  overlay_plug(cols, width_px, inner_left, inner_right, top, bottom, cx, cy); break;
      case CHARGE_ICON_PLUS:  overlay_plus(cols, width_px, inner_left, inner_right, top, bottom, cx, cy); break;
      case CHARGE_ICON_CHECK: overlay_check(cols, width_px, inner_left, inner_right, top, bottom, cx, cy); break;
      case CHARGE_ICON_BOLT_H:
      default:                overlay_bolt_h(cols, width_px, inner_left, inner_right, top, bottom, cx, cy); break;
    }
  }

  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)width_px);
}

static void draw_battery_icon(uint8_t last_page, int x_px, int width_px, int level, int charging) {
  if (width_px < 14) width_px = 14;
  if (width_px > 24) width_px = 24;
  if (level < 0) level = 0;
  if (level > 100) level = 100;

  uint8_t cols[24];
  memset(cols, 0x00, (size_t)width_px);

  const int left = 0, right = width_px - 1, top = 0, bottom = 7;
  const int tip_w = 2, tip_x0 = right - (tip_w - 1), tip_y0 = 2, tip_y1 = 5;
  const int body_left = left, body_right = tip_x0 - 1;
  const int inner_left = body_left + 1, inner_right = body_right - 1;

  for (int x = body_left; x <= body_right; ++x) { cols[x] |= (1u << top); cols[x] |= (1u << bottom); }
  for (int y = top; y <= bottom; ++y) { cols[body_left] |= (1u << y); cols[body_right] |= (1u << y); }
  for (int x = tip_x0; x <= right; ++x) for (int y = tip_y0; y <= tip_y1; ++y) cols[x] |= (1u << y);

  const int inner_w = (inner_right >= inner_left) ? (inner_right - inner_left + 1) : 0;
  const int fill_w = calc_fill_w(inner_w, level);

  for (int x = inner_left; x < inner_left + fill_w; ++x) for (int y = top + 1; y <= bottom - 1; ++y) cols[x] |= (1u << y);

  if (charging) {
    int cx = inner_left + inner_w / 2;
    if (cx < inner_left + 2) cx = inner_left + 2;
    if (cx > inner_right - 2) cx = inner_right - 2;
    const int y_coords[6] = { top+1, top+2, top+3, top+4, top+5, top+6 };
    const int x_coords[6] = { cx-2, cx-1, cx, cx, cx+1, cx+2 };
    for (int i = 0; i < 6; ++i) {
      int by = y_coords[i];
      if (by <= top || by >= bottom) continue;
      int bx = x_coords[i];
      if (bx < inner_left || bx > inner_right) continue;
      cols[bx] &= (uint8_t)~(1u << by);
    }
  }

  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)width_px);
}

typedef struct { int prev_level, prev_chg, prev_style, drawn; } batt_state_style_t;
typedef struct { int prev_level, prev_chg, drawn; } batt_state_t;

static void battery_icon_update_style(batt_state_style_t* st, uint8_t last_page, int x_px, int width_px, int level, int charging, ChargeIconStyle style) {
  if (!st) return;
  if (!st->drawn || level != st->prev_level || charging != st->prev_chg || style != st->prev_style) {
    draw_battery_icon_with_style(last_page, x_px, width_px, level, charging, style);
    st->prev_level = level; st->prev_chg = charging; st->prev_style = style; st->drawn = 1;
  }
}
static void battery_icon_update(batt_state_t* st, uint8_t last_page, int x_px, int width_px, int level, int charging) {
  if (!st) return;
  if (!st->drawn || level != st->prev_level || charging != st->prev_chg) {
    draw_battery_icon(last_page, x_px, width_px, level, charging);
    st->prev_level = level; st->prev_chg = charging; st->drawn = 1;
  }
}

// ---------------- Video status parsing + icons ----------------
typedef enum { VID_UNKNOWN=0, VID_ABSENT=1, VID_PRESENT=2 } video_state_t;

static video_state_t parse_video_status_for_dev(const char* devnode, char* fmt_out, size_t fmt_len) {
  if (fmt_out && fmt_len) fmt_out[0] = '\0';
  if (!devnode) return VID_UNKNOWN;
  char cmd[256];
  snprintf(cmd, sizeof(cmd), "v4l2-ctl --log-status -d %s 2>&1", devnode);
  FILE* fp = popen(cmd, "r");
  if (!fp) return VID_UNKNOWN;
  char line[512];
  video_state_t st = VID_UNKNOWN;
  while (fgets(line, sizeof(line), fp)) {
    if (strstr(line, "No video detected")) {
      st = VID_ABSENT;
      if (fmt_out && fmt_len) snprintf(fmt_out, fmt_len, "No HDMI signal");
    }
    if (strstr(line, "Detected format:")) {
      st = VID_PRESENT;
      char* p = strstr(line, "Detected format:");
      p += strlen("Detected format:");
      while (*p == ' ' || *p == '\t') p++;
      size_t n = strcspn(p, " (");
      if (fmt_out && fmt_len) {
        if (n >= fmt_len) n = fmt_len - 1;
        memcpy(fmt_out, p, n);
        fmt_out[n] = '\0';
      }
    }
  }
  pclose(fp);
  return st;
}

#define VICON_W  12
#define VICON_SP 2

static void draw_video_icon_box(uint8_t last_page, int x_px, int is_ok) {
  uint8_t cols[VICON_W];
  memset(cols, 0x00, sizeof(cols));
  for (int x = 0; x < VICON_W; ++x) { cols[x] |= (1u<<0); cols[x] |= (1u<<7); }
  for (int y = 0; y < 8; ++y) { cols[0] |= (1u<<y); cols[VICON_W-1] |= (1u<<y); }
  if (is_ok) {
    int cx = 4;
    cols[cx - 1] |= (1u << 4);
    cols[cx]     |= (1u << 5);
    cols[cx + 1] |= (1u << 5);
    cols[cx + 2] |= (1u << 4);
    cols[cx + 3] |= (1u << 3);
    cols[cx + 4] |= (1u << 2);
  } else {
    for (int i = 2; i <= 5; ++i) {
      int x1 = 4 + (i - 2);
      int x2 = (VICON_W - 3) - i;
      cols[x1] |= (1u << i);
      cols[x2] |= (1u << i);
    }
  }
  oled_write_page_row((uint8_t)x_px, last_page, cols, (uint8_t)sizeof(cols));
}

typedef struct { int prev_state; int drawn; } vid_box_state_t;

static void video_box_update(vid_box_state_t* st, uint8_t last_page, int x_px, video_state_t s) {
  if (!st) return;
  if (!st->drawn || st->prev_state != (int)s) {
    draw_video_icon_box(last_page, x_px, (s == VID_PRESENT));
    st->prev_state = (int)s;
    st->drawn = 1;
  }
}

// ---------------- GPS/NMEA helpers ----------------
typedef struct {
  int fd;
  int baud;
  char dev[128];
  int initialized;
  int verbosity;
  nmea_info_t info;
  nmea_acc_t acc;

  char utc_date[32];
  char utc_time[32];
  char local_date[32];
  char local_time[32];
  char local_dt[64];
  int  have_local;
  double lat;
  double lon;
  int has_any;

  char speed_kn[24];
  char alt_m[24];
  char sats[24];
  char course_deg[24];
  char hdop[24];
} gps_state_t;

static speed_t gps_baud_to_speed_t(unsigned long baud) {
  switch (baud) {
    case 0: return B0; case 50: return B50; case 75: return B75; case 110: return B110; case 134: return B134;
    case 150: return B150; case 200: return B200; case 300: return B300; case 600: return B600; case 1200: return B1200;
    case 1800: return B1800; case 2400: return B2400; case 4800: return B4800; case 9600: return B9600;
    case 19200: return B19200; case 38400: return B38400;
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}

static void gps_set_raw_termios(struct termios* tio) {
  tio->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tio->c_oflag &= ~OPOST;
  tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio->c_cflag &= ~(CSIZE | PARENB);
  tio->c_cflag |= CS8;
}

static int gps_setup_serial(const char* dev, unsigned long baud) {
  int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }
  gps_set_raw_termios(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  speed_t spd = gps_baud_to_speed_t(baud);
  if (cfsetispeed(&tio, spd) != 0 || cfsetospeed(&tio, spd) != 0) { close(fd); return -1; }
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
  tcflush(fd, TCIFLUSH);
  return fd;
}

static int gps_on_line_cb(const char* line, void* user) {
  nmea_info_t* info = (nmea_info_t*)user;
  (void)nmea_parse_line(line, info);
  return 1;
}

static void gps_init(gps_state_t* gps, const char* dev, int baud) {
  if (!gps) return;
  memset(gps, 0, sizeof(*gps));
  snprintf(gps->dev, sizeof(gps->dev), "%s", (dev && *dev) ? dev : "/dev/ttyS0");
  gps->baud = (baud > 0) ? baud : 115200;
  gps->fd = gps_setup_serial(gps->dev, (unsigned long)gps->baud);
  nmea_info_clear(&gps->info);
  nmea_acc_init(&gps->acc);
  gps->initialized = (gps->fd >= 0) ? 1 : 0;
}

static void gps_close(gps_state_t* gps) {
  if (gps && gps->fd >= 0) { close(gps->fd); gps->fd = -1; }
}

static void gps_poll(gps_state_t* gps, unsigned poll_window_ms, int verbose_echo) {
  if (!gps || gps->fd < 0) return;
  struct timeval t0, t;
  gettimeofday(&t0, NULL);
  for (;;) {
    gettimeofday(&t, NULL);
    unsigned elapsed = (unsigned)((t.tv_sec - t0.tv_sec) * 1000u + (t.tv_usec - t0.tv_usec) / 1000u);
    if (elapsed >= (poll_window_ms ? poll_window_ms : 150u)) break;
    struct timeval tv = { 0, 50000 };
    fd_set rfds; FD_ZERO(&rfds); FD_SET(gps->fd, &rfds);
    int sel = select(gps->fd + 1, &rfds, NULL, NULL, &tv);
    if (sel <= 0) continue;
    if (FD_ISSET(gps->fd, &rfds)) {
      char buf[256];
      ssize_t n = read(gps->fd, buf, sizeof(buf));
      if (n > 0) nmea_acc_feed(&gps->acc, buf, (size_t)n, gps_on_line_cb, &gps->info, verbose_echo ? 1 : 0);
    }
  }
}

static void gps_format_outputs(gps_state_t* gps, const char* tz_opt) {
  if (!gps) return;
  gps->utc_date[0] = gps->utc_time[0] = 0;
  gps->local_date[0] = gps->local_time[0] = gps->local_dt[0] = 0;
  gps->have_local = 0;
  gps->speed_kn[0] = gps->alt_m[0] = gps->sats[0] = gps->course_deg[0] = gps->hdop[0] = 0;

  if (gps->info.has_time || gps->info.has_date) {
    nmea_format_utc(&gps->info, gps->utc_date, sizeof gps->utc_date, gps->utc_time, sizeof gps->utc_time);
#if defined(NMEA_MINI_ENABLE_TZ)
    if (tz_opt && *tz_opt && gps->info.has_time) {
      gps->have_local = nmea_format_local_from_info(&gps->info, tz_opt,
                                                    gps->local_date, sizeof gps->local_date,
                                                    gps->local_time, sizeof gps->local_time,
                                                    gps->local_dt, sizeof gps->local_dt);
    }
#else
    (void)tz_opt;
#endif
  }

  if (gps->info.has_lat) gps->lat = gps->info.lat;
  if (gps->info.has_lon) gps->lon = gps->info.lon;

  if (gps->info.has_speed)  snprintf(gps->speed_kn,   sizeof(gps->speed_kn),   "%.1f", gps->info.speed_knots);
  if (gps->info.has_alt)    snprintf(gps->alt_m,      sizeof(gps->alt_m),      "%.1f", gps->info.alt_m);
  if (gps->info.has_sats)   snprintf(gps->sats,       sizeof(gps->sats),       "%d",   gps->info.satellites);
  if (gps->info.has_course) snprintf(gps->course_deg, sizeof(gps->course_deg), "%.1f", gps->info.course_deg);
  if (gps->info.has_hdop)   snprintf(gps->hdop,       sizeof(gps->hdop),       "%.1f", gps->info.hdop);

  if (gps->info.has_time || gps->info.has_date || gps->info.has_lat || gps->info.has_lon ||
      gps->info.has_speed || gps->info.has_alt || gps->info.has_sats || gps->info.has_course || gps->info.has_hdop)
    gps->has_any = 1;
}

// ---------------- Power meter loop with video overlays and GPS ----------------
static void run_power_meter_loop(const ssd1306_app_cfg_t* cfg, tcp_log_t* logt) {
  const logger_t* extra = &cfg->logger;

  ina219_t meter;
  if (ina219_open(&meter, cfg->ina_bus, (uint8_t)cfg->ina_addr) != 0) {
    LOGF(logt, extra, LOG_ERROR, "INA219 open failed (bus %d addr 0x%02X)", cfg->ina_bus, cfg->ina_addr);
    return;
  }
  if (ina219_calibrate_16V_5A(&meter) != 0) {
    LOGF(logt, extra, LOG_ERROR, "INA219 calibration failed");
    ina219_close(&meter);
    return;
  }

  field_mask_t fields_mask = cfg->fields_mask;
  if (cfg->fields_arg[0]) fields_mask = parse_enabled_fields(cfg->fields_arg);

  int need_gps = (fields_mask & (FLD_UTC_TIME | FLD_UTC_DATE | FLD_LOCAL_TIME | FLD_LOCAL_DATE | FLD_LAT | FLD_LON |
                                 FLD_SPEED | FLD_ALT | FLD_SATS | FLD_COURSE | FLD_HDOP)) ? 1 : 0;

  gps_state_t gps; memset(&gps, 0, sizeof(gps));
  char nmea_dev[128] = "/dev/ttyS0"; int nmea_baud = 115200; int nmea_timeout = 1;
  if (need_gps) {
    if (cfg->nmea_arg[0]) {
      char tmp[192]; strncpy(tmp, cfg->nmea_arg, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
      char* a = strtok(tmp, ",");
      char* b = strtok(NULL, ",");
      char* c = strtok(NULL, ",");
      if (a && *a) snprintf(nmea_dev, sizeof(nmea_dev), "%s", a);
      if (b) nmea_baud = atoi(b);
      if (c) nmea_timeout = atoi(c);
      (void)nmea_timeout;
    }
    gps_init(&gps, nmea_dev, nmea_baud);
  }

  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);

  meter_prev_t prev = {0};
  batt_state_t bstate_fill = {0};
  batt_state_style_t bstate_icon = {0};

  int oled_cols = 0, oled_lines = 0;
  if (read_saved_resolution(&oled_cols, &oled_lines) != 0) { oled_cols = 128; oled_lines = 64; } // fallback only
  // Prefer cfg resolution in main init; but keep safe fallback.

  uint8_t page_count = (uint8_t)(oled_lines / 8);
  if (page_count == 0) page_count = 1;
  uint8_t last_page = (uint8_t)(page_count - 1);
  uint8_t max_text_rows = cfg->show_batt_icon ? last_page : page_count;

  const int start_chars = 8;

  int icon_x = cfg->batt_x_offset;
  if (icon_x < 0) {
    icon_x = (oled_cols - cfg->batt_width_px) / 2;
    if (icon_x < 0) icon_x = 0;
  }

  video_list_t vlist;
  build_video_list(cfg->video_devs, &vlist);
  vid_box_state_t vicons[MAX_VDEVS];
  memset(vicons, 0, sizeof(vicons));

  char vid_texts[MAX_VDEVS][24];
  char fmt_texts[MAX_VDEVS][24];
  for (int i = 0; i < MAX_VDEVS; ++i) { vid_texts[i][0] = '\0'; fmt_texts[i][0] = '\0'; }

  unsigned last_video_poll_ms = 0;
  static char agg_fmt_buf[64];
  static video_state_t agg_state = VID_UNKNOWN;

  while (!SHOULD_STOP(cfg)) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    unsigned now_ms = (unsigned)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);

    int do_video_poll = 0;
    if (last_video_poll_ms == 0 || (now_ms - last_video_poll_ms) >= (cfg->video_poll_ms ? cfg->video_poll_ms : 2000)) {
      do_video_poll = 1;
      last_video_poll_ms = now_ms;
    }

    if (do_video_poll) {
      agg_state = VID_ABSENT;
      agg_fmt_buf[0] = '\0';
      for (int i = 0; i < vlist.count; ++i) {
        char fmt_local[128];
        video_state_t st = parse_video_status_for_dev(vlist.devs[i], fmt_local, sizeof(fmt_local));

        int x_box = cfg->vid_icon_x_offset + i * (VICON_W + VICON_SP);
        if (x_box < oled_cols) video_box_update(&vicons[i], last_page, x_box, st);

        if (st == VID_PRESENT) {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "Cable detected");
          snprintf(fmt_texts[i], sizeof(fmt_texts[i]), "%s", fmt_local);
          if (agg_state != VID_PRESENT) { agg_state = VID_PRESENT; snprintf(agg_fmt_buf, sizeof(agg_fmt_buf), "%s", fmt_local); }
        } else if (st == VID_ABSENT) {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "Cable undetected");
          snprintf(fmt_texts[i], sizeof(fmt_texts[i]), "%s", fmt_local);
          if (agg_state != VID_PRESENT) { agg_state = VID_ABSENT; if (agg_fmt_buf[0] == '\0') snprintf(agg_fmt_buf, sizeof(agg_fmt_buf), "%s", fmt_local); }
        } else {
          snprintf(vid_texts[i], sizeof(vid_texts[i]), "UNK");
          fmt_texts[i][0] = '\0';
          if (agg_state == VID_ABSENT) agg_state = VID_UNKNOWN;
        }
      }
    }

    double busV    = ina219_get_bus_voltage_V(&meter);
    double shuntV  = ina219_get_shunt_voltage_V(&meter);
    double curr_mA = ina219_get_current_mA(&meter);
    double power_W = ina219_get_power_W(&meter);

    double percent = (busV - 3.0) / 1.2 * 100.0;
    if (percent > 100.0) percent = 100.0;
    if (percent < 0.0)   percent = 0.0;

    char gps_utc_time[24] = "", gps_utc_date[24] = "";
    char gps_local_time[24] = "", gps_local_date[24] = "";
    char gps_lat[24] = "", gps_lon[24] = "";
    char gps_speed_kn[24] = "", gps_alt_m[24] = "";
    char gps_sats[24] = "", gps_course_deg[24] = "";
    char gps_hdop[24] = "";

    if (need_gps && gps.initialized) {
      gps_poll(&gps, 150, 0);
      gps_format_outputs(&gps, (cfg->tz_arg[0]) ? cfg->tz_arg : NULL);

      if (gps.info.has_time) {
        snprintf(gps_utc_time, sizeof(gps_utc_time), "%s", gps.utc_time[0] ? gps.utc_time : "");
        if (gps.have_local && gps.local_time[0]) snprintf(gps_local_time, sizeof(gps_local_time), "%s", gps.local_time);
      }
      if (gps.info.has_date) {
        snprintf(gps_utc_date, sizeof(gps_utc_date), "%s", gps.utc_date[0] ? gps.utc_date : "");
        if (gps.have_local && gps.local_date[0]) snprintf(gps_local_date, sizeof(gps_local_date), "%s", gps.local_date);
      }
      if (gps.info.has_lat) snprintf(gps_lat, sizeof(gps_lat), "%.6f", gps.lat);
      if (gps.info.has_lon) snprintf(gps_lon, sizeof(gps_lon), "%.6f", gps.lon);

      if (gps.speed_kn[0])   snprintf(gps_speed_kn,   sizeof(gps_speed_kn),   "%s", gps.speed_kn);
      if (gps.alt_m[0])      snprintf(gps_alt_m,      sizeof(gps_alt_m),      "%s", gps.alt_m);
      if (gps.sats[0])       snprintf(gps_sats,       sizeof(gps_sats),       "%s", gps.sats);
      if (gps.course_deg[0]) snprintf(gps_course_deg, sizeof(gps_course_deg), "%s", gps.course_deg);
      if (gps.hdop[0])       snprintf(gps_hdop,       sizeof(gps_hdop),       "%s", gps.hdop);
    }

    if (!(fields_mask & FLD_NONE)) {
      meter_update_values_dyn(&prev, &vlist, fields_mask, max_text_rows, start_chars,
                              busV, shuntV, curr_mA, power_W, percent,
                              (const char(*)[24])vid_texts, (const char(*)[24])fmt_texts, vlist.count,
                              gps_utc_time, gps_utc_date,
                              gps_local_time, gps_local_date,
                              gps_lat, gps_lon,
                              gps_speed_kn, gps_alt_m, gps_sats, gps_course_deg, gps_hdop,
                              (cfg->tz_arg[0]) ? cfg->tz_arg : NULL);
    }

    if (cfg->show_batt_icon) {
      int charging = ((curr_mA < 0.0) ? 1 : 0) ^ (cfg->invert_charge ? 1 : 0);
      if (cfg->flash_icon) {
        unsigned half_ms = (cfg->ina_period_ms / 2u);
        if (half_ms < 50u) half_ms = 50u;
        if (!((int)(percent + 0.5) >= 99)) draw_battery_icon(last_page, icon_x, cfg->batt_width_px, (int)(percent + 0.5), 0);
        usleep(half_ms * 1000);
        draw_battery_icon_with_style(last_page, icon_x, cfg->batt_width_px, (int)(percent + 0.5), charging, (ChargeIconStyle)cfg->icon_style);
        usleep(half_ms * 1000);
      } else {
        if (charging) battery_icon_update_style(&bstate_icon, last_page, icon_x, cfg->batt_width_px, (int)(percent + 0.5), 1, (ChargeIconStyle)cfg->icon_style);
        else battery_icon_update(&bstate_fill, last_page, icon_x, cfg->batt_width_px, (int)(percent + 0.5), 0);

        unsigned ms = cfg->ina_period_ms ? cfg->ina_period_ms : 2000;
        if (ms < 50u) ms = 50u;
        usleep(ms * 1000);
      }
    } else {
      unsigned ms = cfg->ina_period_ms ? cfg->ina_period_ms : 2000;
      if (ms < 50u) ms = 50u;
      usleep(ms * 1000);
    }

    // --- Previously printed to console; now ONLY to TCP log server ---
    LOGF(logt, extra, LOG_INFO, "PSU Voltage:   %6.3f V", busV + shuntV);
    LOGF(logt, extra, LOG_INFO, "Shunt Voltage: %9.6f V", shuntV);
    LOGF(logt, extra, LOG_INFO, "Load Voltage:  %6.3f V", busV);
    LOGF(logt, extra, LOG_INFO, "Current:       %6.3f A", curr_mA / 1000.0);
    LOGF(logt, extra, LOG_INFO, "Power:         %6.3f W", power_W);
    LOGF(logt, extra, LOG_INFO, "Percent:       %3.1f%%", percent);
    LOGF(logt, extra, LOG_INFO, "Video State:   %s", (agg_state == VID_PRESENT) ? "PRESENT" : ((agg_state == VID_ABSENT) ? "ABSENT" : "UNKNOWN"));
    if (agg_state == VID_PRESENT) LOGF(logt, extra, LOG_INFO, "Detected fmt:  %s", agg_fmt_buf);

    if (need_gps) {
      if (gps_utc_date[0] || gps_utc_time[0]) LOGF(logt, extra, LOG_INFO, "UTC:           %s %s", gps_utc_date, gps_utc_time);
      if (gps_local_date[0] || gps_local_time[0]) LOGF(logt, extra, LOG_INFO, "Local:         %s %s", gps_local_date, gps_local_time);
      if (gps_lat[0] || gps_lon[0]) LOGF(logt, extra, LOG_INFO, "Lat/Lon:       %s, %s", gps_lat, gps_lon);
      if (gps_speed_kn[0])   LOGF(logt, extra, LOG_INFO, "Speed (kn):    %s", gps_speed_kn);
      if (gps_alt_m[0])      LOGF(logt, extra, LOG_INFO, "Altitude (m):  %s", gps_alt_m);
      if (gps_sats[0])       LOGF(logt, extra, LOG_INFO, "Satellites:    %s", gps_sats);
      if (gps_course_deg[0]) LOGF(logt, extra, LOG_INFO, "Course (deg):  %s", gps_course_deg);
      if (gps_hdop[0])       LOGF(logt, extra, LOG_INFO, "HDOP:          %s", gps_hdop);
    }
  }

  if (need_gps) gps_close(&gps);
  ina219_close(&meter);
}

// ---------------- Video-only monitor loop ----------------
static void run_video_monitor_loop(const ssd1306_app_cfg_t* cfg) {
  int oled_cols = 0, oled_lines = 0;
  if (read_saved_resolution(&oled_cols, &oled_lines) != 0) { oled_cols = 128; oled_lines = 64; }

  ssd1306_oled_set_mem_mode(SSD1306_PAGE_MODE);
  uint8_t page_count = (uint8_t)(oled_lines / 8);
  if (page_count == 0) page_count = 1;
  uint8_t last_page = (uint8_t)(page_count - 1);

  video_list_t vlist;
  build_video_list(cfg->video_devs, &vlist);
  vid_box_state_t vicons[MAX_VDEVS];
  memset(vicons, 0, sizeof(vicons));

  uint8_t max_text_rows = (last_page > 0) ? last_page : 0;

  uint8_t label_row = 0;
  for (int i = 0; i < vlist.count && label_row < max_text_rows; ++i) {
    char lab[16];
    snprintf(lab, sizeof(lab), "VID%d:", i);
    ssd1306_oled_set_XY(0, label_row);
    ssd1306_oled_write_line(METER_FONT_SIZE, lab);
    label_row++;
    if (label_row >= max_text_rows) break;
    snprintf(lab, sizeof(lab), "FMT%d:", i);
    ssd1306_oled_set_XY(0, label_row);
    ssd1306_oled_write_line(METER_FONT_SIZE, lab);
    label_row++;
  }

  char vid_texts[MAX_VDEVS][24];
  char fmt_texts[MAX_VDEVS][24];
  for (int i = 0; i < MAX_VDEVS; ++i) { vid_texts[i][0] = '\0'; fmt_texts[i][0] = '\0'; }

  while (!SHOULD_STOP(cfg)) {
    for (int i = 0; i < vlist.count; ++i) {
      char fmt_local[96];
      video_state_t st = parse_video_status_for_dev(vlist.devs[i], fmt_local, sizeof(fmt_local));

      int x_box = cfg->vid_icon_x_offset + i * (VICON_W + VICON_SP);
      if (x_box < oled_cols) video_box_update(&vicons[i], last_page, x_box, st);

      const char* vid_text = (st == VID_PRESENT) ? "Cable detected" : ((st == VID_ABSENT) ? "Cable undetected" : "UNK");
      snprintf(vid_texts[i], sizeof(vid_texts[i]), "%s", vid_text);

      if (st == VID_PRESENT) snprintf(fmt_texts[i], sizeof(fmt_texts[i]), "%s", fmt_local);
      else fmt_texts[i][0] = '\0';
    }

    int row = 0;
    for (int i = 0; i < vlist.count && row < (int)max_text_rows; ++i) {
      int label_chars = (int)strlen("VID0:");
      ssd1306_oled_clear_line((uint8_t)row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), (uint8_t)row);
      ssd1306_oled_write_line(METER_FONT_SIZE, vid_texts[i]);
      row++;
      if (row >= (int)max_text_rows) break;

      ssd1306_oled_clear_line((uint8_t)row);
      ssd1306_oled_set_XY((uint8_t)(label_chars * meter_char_width()), (uint8_t)row);
      ssd1306_oled_write_line(METER_FONT_SIZE, fmt_texts[i]);
      row++;
    }

    unsigned ms = cfg->video_poll_ms ? cfg->video_poll_ms : 2000;
    if (ms < 100u) ms = 100u;
    usleep(ms * 1000);
  }
}

// ---------------- Public API ----------------
void ssd1306_app_cfg_init(ssd1306_app_cfg_t* cfg) {
  if (!cfg) return;
  memset(cfg, 0, sizeof(*cfg));
  cfg->stop_flag = NULL;

  cfg->i2c_node_address = 1;
  cfg->oled_type[0] = 0;
  cfg->orientation = -1;
  cfg->inverted = -1;
  cfg->display = -1;

  cfg->animate = 0;
  cfg->frame_delay_ms = 30;

  cfg->test_frame = 0;

  cfg->power_meter = 1;
  cfg->ina_bus = 10;
  cfg->ina_addr = 0x43;
  cfg->ina_period_ms = 2000;

  cfg->fields_arg[0] = 0;
  cfg->fields_mask = default_field_mask();

  cfg->show_batt_icon = 0;
  cfg->invert_charge = 0;
  cfg->batt_x_offset = -1;
  cfg->batt_width_px = 18;
  cfg->flash_icon = 0;
  cfg->icon_style = (ChargeIconStyle)CHARGE_ICON_PLUG;

  cfg->video_monitor = 0;
  cfg->video_devs[0] = 0;
  cfg->video_poll_ms = 2000;
  cfg->vid_icon_x_offset = 0;

  snprintf(cfg->nmea_arg, sizeof(cfg->nmea_arg), "/dev/ttyS0,115200,1");
  cfg->tz_arg[0] = 0;

  cfg->log_tcp_enable = 1;
  snprintf(cfg->log_bind_ip, sizeof(cfg->log_bind_ip), "%s", "0.0.0.0");
  cfg->log_port = 9090;

  cfg->logger.fn = NULL;
  cfg->logger.user = NULL;
}

static int oled_apply_type_or_load(const char* oled_type, int* cols, int* lines) {
  if (!cols || !lines) return -1;
  if (oled_type && *oled_type) {
    if (!strcmp(oled_type, "128x64")) { ssd1306_oled_default_config(64,128); *cols=128; *lines=64; return 0; }
    if (!strcmp(oled_type, "128x32")) { ssd1306_oled_default_config(32,128); *cols=128; *lines=32; return 0; }
    if (!strcmp(oled_type, "64x48"))  { ssd1306_oled_default_config(48, 64); *cols= 64; *lines=48; return 0; }
    return -1;
  }
  if (ssd1306_oled_load_resolution() != 0) return -1;
  if (read_saved_resolution(cols, lines) != 0) { *cols=128; *lines=64; }
  return 0;
}

int ssd1306_app_run(const ssd1306_app_cfg_t* cfg) {
  if (!cfg) return 1;

  tcp_log_t logt;
  tcp_log_init(&logt, cfg);
  tcp_log_start(&logt);

  uint8_t rc = ssd1306_init(cfg->i2c_node_address);
  if (rc != 0) {
    LOGF(&logt, &cfg->logger, LOG_ERROR, "no oled attached to /dev/i2c-%d", cfg->i2c_node_address);
    tcp_log_stop(&logt);
    return 1;
  }

  int oled_cols=0, oled_lines=0;
  if (oled_apply_type_or_load(cfg->oled_type, &oled_cols, &oled_lines) != 0) {
    LOGF(&logt, &cfg->logger, LOG_ERROR, "OLED resolution not set; set oled_type or init once");
    ssd1306_end();
    tcp_log_stop(&logt);
    return 1;
  }

  if (cfg->orientation > -1) ssd1306_oled_set_rotate((uint8_t)cfg->orientation);
  if (cfg->inverted > -1)    ssd1306_oled_display_flip((uint8_t)cfg->inverted);
  if (cfg->display > -1)     ssd1306_oled_onoff((uint8_t)cfg->display);

  if ((cfg->power_meter + cfg->animate + cfg->test_frame + cfg->video_monitor) > 1)
    LOGF(&logt, &cfg->logger, LOG_WARN, "Modes are exclusive; choose one.");

  if (cfg->animate) {
    ssd1306_oled_onoff(1);
    run_animation_loop(cfg, oled_cols, oled_lines, cfg->frame_delay_ms);
  } else if (cfg->test_frame) {
    ssd1306_oled_onoff(1);
    (void)ssd1306_blit_from_01(image_data, 128, 64);
  } else if (cfg->video_monitor) {
    ssd1306_oled_onoff(1);
    run_video_monitor_loop(cfg);
  } else if (cfg->power_meter) {
    ssd1306_oled_onoff(1);
    run_power_meter_loop(cfg, &logt);
  }

  ssd1306_end();
  tcp_log_stop(&logt);
  return rc;
}
