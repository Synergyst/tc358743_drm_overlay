#ifndef NMEA_MINIPARSER_H
#define NMEA_MINIPARSER_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#ifndef NMEA_MAX_LINE
#define NMEA_MAX_LINE 256
#endif

typedef struct {
    int has_time;
    int has_date;
    int has_lat;
    int has_lon;
    int hour, minute, second;   /* UTC time */
    int year, month, day;       /* UTC date (YYYY, 1..12, 1..31) */
    double lat, lon;            /* decimal degrees */

    /* New fields */
    int has_speed;
    int has_course;
    int has_alt;
    int has_sats;
    int has_hdop;
    double speed_knots;         /* speed over ground in knots (RMC) */
    double course_deg;          /* course over ground in degrees (RMC) */
    double alt_m;               /* altitude in meters (GGA) */
    int satellites;             /* satellites in use (GGA) */
    double hdop;                /* horizontal dilution of precision (GGA) */
} nmea_info_t;

enum {
    NMEA_UPDATED_NONE  = 0,
    NMEA_UPDATED_TIME  = 1 << 0,
    NMEA_UPDATED_DATE  = 1 << 1,
    NMEA_UPDATED_LAT   = 1 << 2,
    NMEA_UPDATED_LON   = 1 << 3,
    NMEA_UPDATED_SPEED = 1 << 4,
    NMEA_UPDATED_COURSE= 1 << 5,
    NMEA_UPDATED_ALT   = 1 << 6,
    NMEA_UPDATED_SATS  = 1 << 7,
    NMEA_UPDATED_HDOP  = 1 << 8
};

static inline void nmea_info_clear(nmea_info_t* info) {
    if (info) memset(info, 0, sizeof(*info));
}

/* checksum: allow lines without '*' */
static inline int nmea_checksum_ok_allow_missing(const char* line) {
    if (!line) return 0;
    const char* star = strrchr(line, '*');
    if (!star) return 1;
    if ((size_t)(strlen(star + 1)) < 2) return 0;
    unsigned int given = 0;
    if (sscanf(star + 1, "%2x", &given) != 1) return 0;
    unsigned char sum = 0;
    for (const char* p = line + 1; p < star; ++p) sum ^= (unsigned char)(*p);
    return sum == (unsigned char)given;
}

static inline int nmea_parse_hhmmss(const char* s, int* H, int* M, int* S) {
    if (!s || !*s) return 0;
    int h = 0, m = 0; double sec = 0.0;
    if (sscanf(s, "%2d%2d%lf", &h, &m, &sec) < 2) return 0;
    if (h < 0 || h > 23 || m < 0 || m > 59) return 0;
    int isec = (int)floor(sec + 0.5);
    if (isec < 0) isec = 0;
    if (isec > 59) isec = 59;
    if (H) *H = h; if (M) *M = m; if (S) *S = isec;
    return 1;
}

static inline int nmea_parse_ddmmyy(const char* s, int* Y, int* M, int* D) {
    if (!s || strlen(s) < 6) return 0;
    int d = 0, m = 0, y = 0;
    if (sscanf(s, "%2d%2d%2d", &d, &m, &y) != 3) return 0;
    if (d < 1 || d > 31 || m < 1 || m > 12) return 0;
    int year = (y >= 80) ? (1900 + y) : (2000 + y);
    if (Y) *Y = year; if (M) *M = m; if (D) *D = d;
    return 1;
}

static inline int nmea_starts_with_talker(const char* line, const char* type3) {
    if (!line || line[0] != '$') return 0;
    size_t len = strlen(line);
    if (len < 6) return 0;
    return memcmp(line + 3, type3, 3) == 0;
}

static inline double nmea_degmin_to_decimal(const char* fld, char hemi) {
    if (!fld || !*fld) return NAN;
    double v = atof(fld);
    if (v == 0.0 && fld[0] != '0') return NAN;
    double deg = floor(v / 100.0);
    double min = v - deg * 100.0;
    double dec = deg + (min / 60.0);
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

/* simple field splitter (in-place). Returns count in out[] */
static inline int nmea_split_fields(char* s, const char* sep, char** out, int max_out) {
    if (!s || !out || max_out <= 0) return 0;
    int n = 0;
    char* p = s;
    while (n < max_out) {
        out[n++] = p;
        char* c = p;
        while (*c && *c != *sep) c++;
        if (*c == '\0') break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

/* sentence parsers (mutate s) */
static inline unsigned nmea_parse_rmc(char* s, nmea_info_t* info) {
    unsigned updated = 0;
    char* star = strchr(s, '*'); if (star) *star = '\0';
    char* f[24] = {0};
    int idx = nmea_split_fields(s, ",", f, 24);
    const char* times = (idx > 1) ? f[1] : NULL;
    const char* status= (idx > 2) ? f[2] : NULL;
    const char* lats  = (idx > 3) ? f[3] : NULL;
    const char* ns    = (idx > 4) ? f[4] : NULL;
    const char* lons  = (idx > 5) ? f[5] : NULL;
    const char* ew    = (idx > 6) ? f[6] : NULL;
    const char* spdkn = (idx > 7) ? f[7] : NULL; /* speed over ground (knots) */
    const char* crs   = (idx > 8) ? f[8] : NULL; /* course over ground (deg) */
    const char* dates = (idx > 9) ? f[9] : NULL;

    /* Always update if present (do not make fields "sticky") */

    if (times && *times) {
        int H,M,S;
        if (nmea_parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
            updated |= NMEA_UPDATED_TIME;
        }
    }
    if (dates && *dates) {
        int Y,M,D;
        if (nmea_parse_ddmmyy(dates, &Y, &M, &D)) {
            info->year = Y; info->month = M; info->day = D; info->has_date = 1;
            updated |= NMEA_UPDATED_DATE;
        }
    }

    int fix_ok = (status && *status == 'A');
    if (fix_ok) {
        if (lats && ns && *lats && *ns) {
            double lat = nmea_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; updated |= NMEA_UPDATED_LAT; }
        }
        if (lons && ew && *lons && *ew) {
            double lon = nmea_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; updated |= NMEA_UPDATED_LON; }
        }
    }

    if (spdkn && *spdkn) {
        info->speed_knots = atof(spdkn);
        info->has_speed = 1;
        updated |= NMEA_UPDATED_SPEED;
    }

    if (crs && *crs) {
        info->course_deg = atof(crs);
        info->has_course = 1;
        updated |= NMEA_UPDATED_COURSE;
    }

    return updated;
}

static inline unsigned nmea_parse_gga(char* s, nmea_info_t* info) {
    unsigned updated = 0;
    char* star = strchr(s, '*'); if (star) *star = '\0';
    char* f[20] = {0};
    int idx = nmea_split_fields(s, ",", f, 20);
    const char* times = (idx > 1) ? f[1] : NULL;
    const char* lats  = (idx > 2) ? f[2] : NULL;
    const char* ns    = (idx > 3) ? f[3] : NULL;
    const char* lons  = (idx > 4) ? f[4] : NULL;
    const char* ew    = (idx > 5) ? f[5] : NULL;
    const char* fixq  = (idx > 6) ? f[6] : NULL;
    const char* sats  = (idx > 7) ? f[7] : NULL;
    const char* hdop  = (idx > 8) ? f[8] : NULL;
    const char* alt   = (idx > 9) ? f[9] : NULL;
    // f[10] unit 'M'

    if (times && *times) {
        int H,M,S;
        if (nmea_parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
            updated |= NMEA_UPDATED_TIME;
        }
    }

    int fix_ok = (fixq && *fixq != '0');
    if (fix_ok) {
        if (lats && ns && *lats && *ns) {
            double lat = nmea_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; updated |= NMEA_UPDATED_LAT; }
        }
        if (lons && ew && *lons && *ew) {
            double lon = nmea_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; updated |= NMEA_UPDATED_LON; }
        }
    }

    if (sats && *sats) {
        int nsats = atoi(sats);
        info->satellites = nsats;
        info->has_sats = 1;
        updated |= NMEA_UPDATED_SATS;
    }

    if (hdop && *hdop) {
        info->hdop = atof(hdop);
        info->has_hdop = 1;
        updated |= NMEA_UPDATED_HDOP;
    }

    if (alt && *alt) {
        info->alt_m = atof(alt);
        info->has_alt = 1;
        updated |= NMEA_UPDATED_ALT;
    }

    return updated;
}

static inline unsigned nmea_parse_gll(char* s, nmea_info_t* info) {
    unsigned updated = 0;
    char* star = strchr(s, '*'); if (star) *star = '\0';
    char* f[16] = {0};
    int idx = nmea_split_fields(s, ",", f, 16);
    const char* lats  = (idx > 1) ? f[1] : NULL;
    const char* ns    = (idx > 2) ? f[2] : NULL;
    const char* lons  = (idx > 3) ? f[3] : NULL;
    const char* ew    = (idx > 4) ? f[4] : NULL;
    const char* times = (idx > 5) ? f[5] : NULL;
    const char* stat  = (idx > 6) ? f[6] : NULL;

    if (times && *times) {
        int H,M,S;
        if (nmea_parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
            updated |= NMEA_UPDATED_TIME;
        }
    }
    int fix_ok = (stat && *stat == 'A');
    if (fix_ok) {
        if (lats && ns && *lats && *ns) {
            double lat = nmea_degmin_to_decimal(lats, ns[0]);
            if (!isnan(lat)) { info->lat = lat; info->has_lat = 1; updated |= NMEA_UPDATED_LAT; }
        }
        if (lons && ew && *lons && *ew) {
            double lon = nmea_degmin_to_decimal(lons, ew[0]);
            if (!isnan(lon)) { info->lon = lon; info->has_lon = 1; updated |= NMEA_UPDATED_LON; }
        }
    }
    return updated;
}

static inline unsigned nmea_parse_zda(char* s, nmea_info_t* info) {
    unsigned updated = 0;
    char* star = strchr(s, '*'); if (star) *star = '\0';
    char* f[12] = {0};
    int idx = nmea_split_fields(s, ",", f, 12);
    const char* times = (idx > 1) ? f[1] : NULL;
    const char* day   = (idx > 2) ? f[2] : NULL;
    const char* mon   = (idx > 3) ? f[3] : NULL;
    const char* year  = (idx > 4) ? f[4] : NULL;

    if (times && *times) {
        int H,M,S;
        if (nmea_parse_hhmmss(times, &H, &M, &S)) {
            info->hour = H; info->minute = M; info->second = S; info->has_time = 1;
            updated |= NMEA_UPDATED_TIME;
        }
    }
    if (day && mon && year && *day && *mon && *year) {
        int D = atoi(day), M = atoi(mon), Y = atoi(year);
        if (Y >= 1900 && M >= 1 && M <= 12 && D >= 1 && D <= 31) {
            info->year = Y; info->month = M; info->day = D; info->has_date = 1;
            updated |= NMEA_UPDATED_DATE;
        }
    }
    return updated;
}

/* Parse one full NMEA line (CR/LF allowed). Returns bitmask of updates. */
static inline unsigned nmea_parse_line(const char* line, nmea_info_t* info) {
    if (!line || !info) return NMEA_UPDATED_NONE;
    char buf[NMEA_MAX_LINE];
    size_t n = 0;
    for (; n + 1 < sizeof(buf) && line[n]; ++n) buf[n] = line[n];
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    buf[n] = '\0';
    while (n > 0 && (buf[n-1] == '\r' || buf[n-1] == '\n')) buf[--n] = '\0';
    if (n < 1 || buf[0] != '$') return NMEA_UPDATED_NONE;
    if (!nmea_checksum_ok_allow_missing(buf)) return NMEA_UPDATED_NONE;

    unsigned updated = 0;
    char work[NMEA_MAX_LINE];
    if (nmea_starts_with_talker(buf, "RMC")) {
        strncpy(work, buf, sizeof(work)-1); work[sizeof(work)-1] = '\0';
        updated |= nmea_parse_rmc(work, info);
    } else if (nmea_starts_with_talker(buf, "GGA")) {
        strncpy(work, buf, sizeof(work)-1); work[sizeof(work)-1] = '\0';
        updated |= nmea_parse_gga(work, info);
    } else if (nmea_starts_with_talker(buf, "GLL")) {
        strncpy(work, buf, sizeof(work)-1); work[sizeof(work)-1] = '\0';
        updated |= nmea_parse_gll(work, info);
    } else if (nmea_starts_with_talker(buf, "ZDA")) {
        strncpy(work, buf, sizeof(work)-1); work[sizeof(work)-1] = '\0';
        updated |= nmea_parse_zda(work, info);
    }
    return updated;
}

/* ----- UTC conversion without timegm/tz tricks ----- */
/* days since 1970-01-01 (can be negative) */
static inline int64_t nmea_days_from_civil(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static inline int nmea_utc_time_t(const nmea_info_t* info, time_t* out) {
    if (!info || !out || !info->has_time || !info->has_date) return 0;
    int64_t days = nmea_days_from_civil(info->year, (unsigned)info->month, (unsigned)info->day);
    int64_t secs = days * 86400
                 + (int64_t)info->hour * 3600
                 + (int64_t)info->minute * 60
                 + (int64_t)info->second;
    *out = (time_t)secs;
    return 1;
}

/* Format UTC strings */
static inline int nmea_format_utc(const nmea_info_t* info,
                                  char* date_buf, size_t date_sz,
                                  char* time_buf, size_t time_sz) {
    if (!info) return 0;
    int ok = 0;
    if (info->has_date && date_buf && date_sz) {
        snprintf(date_buf, date_sz, "%04d-%02d-%02d", info->year, info->month, info->day);
        ok = 1;
    }
    if (info->has_time && time_buf && time_sz) {
        snprintf(time_buf, time_sz, "%02d:%02d:%02d", info->hour, info->minute, info->second);
        ok = 1;
    }
    return ok;
}

/* Optional: local formatting (requires TZ names, uses env). Disabled by default. */
#ifdef NMEA_MINI_ENABLE_TZ
/* Provide portable wrappers for gmtime_r/localtime_r */
static inline struct tm* nmea_gmtime_r(const time_t* t, struct tm* out) {
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
    return gmtime_r(t, out);
#else
    struct tm* p = gmtime(t);
    if (!p) return NULL;
    *out = *p;
    return out;
#endif
}

static inline struct tm* nmea_localtime_r(const time_t* t, struct tm* out) {
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200112L
    return localtime_r(t, out);
#else
    struct tm* p = localtime(t);
    if (!p) return NULL;
    *out = *p;
    return out;
#endif
}

/* If your headers don’t declare these, forward-declare to silence warnings. */
int setenv(const char*, const char*, int);
int unsetenv(const char*);
void tzset(void);

static inline int nmea_format_local_from_info(const nmea_info_t* info,
                                              const char* tz,
                                              char* date_buf, size_t date_sz,
                                              char* time_buf, size_t time_sz,
                                              char* dt_buf,   size_t dt_sz) {
    if (!info || !tz || !info->has_time) return 0;
    /* Build a UTC time_t (use today’s UTC date if missing) */
    int y, m, d;
    if (info->has_date) {
        y = info->year; m = info->month; d = info->day;
    } else {
        time_t now = time(NULL);
        struct tm g;
        nmea_gmtime_r(&now, &g);
        y = g.tm_year + 1900; m = g.tm_mon + 1; d = g.tm_mday;
    }
    nmea_info_t tmp = *info;
    tmp.year = y; tmp.month = m; tmp.day = d;
    time_t t_utc = 0;
    if (!nmea_utc_time_t(&tmp, &t_utc)) return 0;

    char* old = getenv("TZ");
    char oldbuf[512]; int had_old = 0;
    if (old) { strncpy(oldbuf, old, sizeof(oldbuf)-1); oldbuf[sizeof(oldbuf)-1]=0; had_old=1; }
    setenv("TZ", tz, 1); tzset();

    struct tm loc;
    nmea_localtime_r(&t_utc, &loc);
    int ok = 1;
    if (date_buf && date_sz) ok &= (strftime(date_buf, date_sz, "%Y-%m-%d", &loc) != 0);
    if (time_buf && time_sz) ok &= (strftime(time_buf, time_sz, "%H:%M:%S", &loc) != 0);
    if (dt_buf && dt_sz)     ok &= (strftime(dt_buf,   dt_sz,   "%Y-%m-%d %H:%M:%S %Z", &loc) != 0);

    if (had_old) setenv("TZ", oldbuf, 1); else unsetenv("TZ");
    tzset();
    return ok;
}
#endif /* NMEA_MINI_ENABLE_TZ */

/* Byte-stream -> line accumulator with callback */
typedef struct {
    char line[NMEA_MAX_LINE];
    size_t len;
} nmea_acc_t;

typedef int (*nmea_on_line_fn)(const char* line, void* user); /* return nonzero to continue */

static inline void nmea_acc_init(nmea_acc_t* acc) { if (acc) acc->len = 0; }

/* Calls on_line for each complete line (without CR/LF). */
static inline void nmea_acc_feed(nmea_acc_t* acc, const char* data, size_t n,
                                 nmea_on_line_fn on_line, void* user, int echo_raw) {
    if (!acc || !data || !on_line) return;
    for (size_t i = 0; i < n; ++i) {
        char c = data[i];
        if (c == '\n') {
            acc->line[acc->len] = '\0';
            if (acc->len > 0) {
                if (echo_raw) fprintf(stderr, "RAW: %s\n", acc->line);
                (void)on_line(acc->line, user);
            }
            acc->len = 0;
        } else if (c != '\r') {
            if (acc->len + 1 < sizeof(acc->line)) acc->line[acc->len++] = c;
            else acc->len = 0;
        }
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* NMEA_MINIPARSER_H */
