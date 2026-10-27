#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>
#include <esp_heap_caps.h>
#include "esp_sntp.h"

#include "driver/spi_master.h"
#include <Wire.h>
#include <TouchDrvCSTXXX.hpp>
#include <SensorQMI8658.hpp>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP_I2S.h>
#include "mbedtls/base64.h"

// -----------------------------------------------------------------------------
// Fallbacks das Definições do Ficheiro .env
// -----------------------------------------------------------------------------
#ifndef WIFI_SSID
#define WIFI_SSID "WIFI_INDEFINIDO"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "SENHA_INDEFINIDA"
#endif

#ifndef GMT_OFFSET_SEC
#define GMT_OFFSET_SEC -10800 // UTC-3 (Horário de Brasília)
#endif

#ifndef DAYLIGHT_OFFSET_SEC
#define DAYLIGHT_OFFSET_SEC 0
#endif

#ifndef NTP_SERVER1
#define NTP_SERVER1 "a.st1.ntp.br"
#endif

#ifndef NTP_SERVER2
#define NTP_SERVER2 "b.st1.ntp.br"
#endif

#ifndef NTP_SERVER3
#define NTP_SERVER3 "pool.ntp.org"
#endif

#ifndef LATITUDE
#define LATITUDE ""
#endif
#ifndef LONGITUDE
#define LONGITUDE ""
#endif
#ifndef CITY_NAME
#define CITY_NAME ""
#endif

#ifndef NTP_SYNC_INTERVAL_MS
#define NTP_SYNC_INTERVAL_MS 3600000 // 1 hora
#endif

// -----------------------------------------------------------------------------
// Parâmetros Físicos do Ecrã AMOLED 2.16" e Pinagem QSPI
// -----------------------------------------------------------------------------
#define SCREEN_WIDTH   480
#define SCREEN_HEIGHT  480
#define CENTER_X       240
#define CENTER_Y       240

#define LCD_HOST       SPI2_HOST
#define LCD_PIN_CS     12
#define LCD_PIN_SCLK   38
#define LCD_PIN_DATA0  4
#define LCD_PIN_DATA1  5
#define LCD_PIN_DATA2  6
#define LCD_PIN_DATA3  7
#define LCD_PIN_RST    39

#define I2C_SDA        15
#define I2C_SCL        14
#define TOUCH_INT      11
#define TOUCH_RST      40
#define TOUCH_ADDR     0x5A

// Linhas por buffer: 480*30*2 = 28800 bytes, abaixo do limite de 32768 bytes
// por transação SPI do ESP32-S3.
#define DRAW_BUF_LINES 30

static spi_device_handle_t lcd_spi = NULL;
static lv_disp_drv_t *lcd_disp_drv = NULL;

// Declaração do recurso gráfico (gerado em src/bg_racing.c a partir do logótipo)
#ifdef __cplusplus
extern "C" {
#endif
LV_IMG_DECLARE(bg_racing);
#ifdef __cplusplus
}
#endif

// Buffers do LVGL
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_buf;

// Elementos Visuais dos Ponteiros
static lv_obj_t *line_hour;
static lv_obj_t *line_minute;
static lv_obj_t *line_second;
static lv_obj_t *outline_hour;
static lv_obj_t *outline_minute;
static lv_point_t hour_pts[2];
static lv_point_t min_pts[2];
static lv_point_t sec_pts[2];

// -----------------------------------------------------------------------------
// Driver de Hardware: QSPI + painel AMOLED CO5300
// Comando: cmd 0x02, endereço = registo << 8, parâmetros em 1 linha
// Pixels : cmd 0x32, endereço = 0x2C00, dados em 4 linhas
// -----------------------------------------------------------------------------
static void co_cmd(uint8_t reg, const uint8_t *params, int n) {
    spi_transaction_t t = {};
    t.cmd = 0x02;
    t.addr = ((uint32_t)reg) << 8;
    if (n > 0) {
        t.flags = SPI_TRANS_USE_TXDATA;
        t.length = n * 8;
        for (int i = 0; i < n && i < 4; i++) t.tx_data[i] = params[i];
    }
    spi_device_polling_transmit(lcd_spi, &t);
}

static void co_cmd1(uint8_t reg, uint8_t v) { co_cmd(reg, &v, 1); }

static void lvgl_rounder_cb(lv_disp_drv_t *disp, lv_area_t *area) {
    // O CO5300 exige janelas com início par e fim ímpar
    area->x1 &= ~1;
    area->y1 &= ~1;
    area->x2 |= 1;
    area->y2 |= 1;
}

static void IRAM_ATTR lcd_pixels_done(spi_transaction_t *t) {
    // O post_cb também dispara nas transações de comando; só os pixels (0x32) terminam o flush
    if (t->cmd == 0x32) {
        lv_disp_flush_ready(lcd_disp_drv);
    }
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint8_t c[4] = { (uint8_t)(area->x1 >> 8), (uint8_t)area->x1, (uint8_t)(area->x2 >> 8), (uint8_t)area->x2 };
    uint8_t r[4] = { (uint8_t)(area->y1 >> 8), (uint8_t)area->y1, (uint8_t)(area->y2 >> 8), (uint8_t)area->y2 };
    co_cmd(0x2A, c, 4);
    co_cmd(0x2B, r, 4);

    static spi_transaction_t t;
    t = {};
    t.cmd = 0x32;
    t.addr = 0x2C00;
    t.flags = SPI_TRANS_MODE_QIO;
    t.tx_buffer = color_p;
    t.length = (size_t)lv_area_get_width(area) * lv_area_get_height(area) * 2 * 8;
    if (spi_device_queue_trans(lcd_spi, &t, portMAX_DELAY) != ESP_OK) {
        lv_disp_flush_ready(disp);
    }
}

void init_amoled_hardware(lv_disp_drv_t *disp_drv) {
    lcd_disp_drv = disp_drv;

    pinMode(LCD_PIN_RST, OUTPUT);
    digitalWrite(LCD_PIN_RST, HIGH); delay(10);
    digitalWrite(LCD_PIN_RST, LOW);  delay(200);
    digitalWrite(LCD_PIN_RST, HIGH); delay(200);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LCD_PIN_DATA0;
    buscfg.miso_io_num = LCD_PIN_DATA1;
    buscfg.sclk_io_num = LCD_PIN_SCLK;
    buscfg.quadwp_io_num = LCD_PIN_DATA2;
    buscfg.quadhd_io_num = LCD_PIN_DATA3;
    buscfg.max_transfer_sz = SCREEN_WIDTH * DRAW_BUF_LINES * 2 + 64;
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits = 8;
    devcfg.address_bits = 24;
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 40 * 1000 * 1000;
    devcfg.spics_io_num = LCD_PIN_CS;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    devcfg.queue_size = 2;
    devcfg.post_cb = lcd_pixels_done;
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi));

    co_cmd(0x11, NULL, 0);          // Sleep out
    delay(120);
    co_cmd1(0xFE, 0x00);            // Comandos de utilizador
    co_cmd1(0xC4, 0x80);            // Modo QSPI
    co_cmd1(0x3A, 0x55);            // RGB565
    co_cmd1(0x53, 0x20);            // Controlo de brilho
    co_cmd1(0x63, 0xFF);            // Brilho HBM
    co_cmd1(0x36, 0x00);            // Orientação
    co_cmd(0x29, NULL, 0);          // Display ON
    co_cmd1(0x51, 0xD0);            // Brilho
    co_cmd1(0x58, 0x00);
    co_cmd(0x20, NULL, 0);          // Inversão desligada
    delay(10);
}

// -----------------------------------------------------------------------------
// Touch CST9220 (I2C) como dispositivo de entrada do LVGL
// -----------------------------------------------------------------------------
static TouchDrvCST92xx touch;
static bool touch_ok = false;

static void my_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    static int16_t last_x = 0, last_y = 0;
    int16_t x[1], y[1];
    if (touch_ok && touch.getPoint(x, y, 1) > 0) {
        last_x = x[0];
        last_y = y[0];
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

void init_touch() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    touch.setPins(TOUCH_RST, TOUCH_INT);
    touch_ok = touch.begin(Wire, TOUCH_ADDR, I2C_SDA, I2C_SCL);
    Serial.printf("[TOUCH] %s\n", touch_ok ? "CST9220 OK" : "FALHA ao iniciar");
    touch.setMaxCoordinates(SCREEN_WIDTH, SCREEN_HEIGHT);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touch_read;
    lv_indev_drv_register(&indev_drv);
}

// -----------------------------------------------------------------------------
// Cálculos Trigonométricos dos Ponteiros
// -----------------------------------------------------------------------------
void update_hand_geometry(lv_obj_t *line, lv_point_t *pts, int angle_deg, int length, lv_obj_t *outline = NULL) {
    float rad = (angle_deg - 90) * 0.0174532925f;

    pts[0].x = CENTER_X;
    pts[0].y = CENTER_Y;
    pts[1].x = CENTER_X + (int)(cosf(rad) * length);
    pts[1].y = CENTER_Y + (int)(sinf(rad) * length);

    lv_line_set_points(line, pts, 2);
    if (outline) lv_line_set_points(outline, pts, 2);
}

static void clock_timer_cb(lv_timer_t *timer) {
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    int sec  = timeinfo.tm_sec;
    int min  = timeinfo.tm_min;
    int hour = timeinfo.tm_hour % 12;

    int sec_angle  = sec * 6;
    int min_angle  = (min * 6) + (sec / 10);
    int hour_angle = (hour * 30) + (min / 2);

    update_hand_geometry(line_hour,   hour_pts, hour_angle, 110, outline_hour);
    update_hand_geometry(line_minute, min_pts,  min_angle,  160, outline_minute);
    update_hand_geometry(line_second, sec_pts,  sec_angle,  190);
}

// -----------------------------------------------------------------------------
// Interface Gráfica do Relógio Analógico
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Nível de bolha (acelerómetro QMI8658)
// -----------------------------------------------------------------------------
static SensorQMI8658 imu;
static bool imu_ok = false;
static lv_obj_t *bubble;
static lv_obj_t *lbl_angle;
static float filt_x = 0, filt_y = 0;

// Ajuste estes sinais se a bolha se mover no sentido contrário ao esperado
// A tela está rodada 90° em relação ao sensor: X da tela vem do eixo Y do IMU e vice-versa
#define LEVEL_SIGN_X  (-1.0f)
#define LEVEL_SIGN_Y  (-1.0f)
#define LEVEL_RADIUS  150    // raio máximo de deslocação da bolha (px)
#define LEVEL_FULL_G  0.5f   // inclinação (em g, ~30°) que leva a bolha à borda

static void level_timer_cb(lv_timer_t *timer) {
    float ax, ay, az;
    if (!imu_ok || !imu.getDataReady() || !imu.getAccelerometer(ax, ay, az)) return;

    filt_x += 0.2f * (ax - filt_x);
    filt_y += 0.2f * (ay - filt_y);

    float dx = LEVEL_SIGN_X * filt_y / LEVEL_FULL_G * LEVEL_RADIUS;
    float dy = LEVEL_SIGN_Y * filt_x / LEVEL_FULL_G * LEVEL_RADIUS;
    float d = sqrtf(dx * dx + dy * dy);
    if (d > LEVEL_RADIUS) {
        dx *= LEVEL_RADIUS / d;
        dy *= LEVEL_RADIUS / d;
    }
    lv_obj_align(bubble, LV_ALIGN_CENTER, (int)dx, (int)dy);

    float tilt_x = asinf(constrain(filt_x, -1.0f, 1.0f)) * 57.29578f;
    float tilt_y = asinf(constrain(filt_y, -1.0f, 1.0f)) * 57.29578f;
    float tilt = sqrtf(tilt_x * tilt_x + tilt_y * tilt_y);
    bool level = tilt < 1.0f;
    lv_obj_set_style_bg_color(bubble, level ? lv_color_hex(0x00E060) : lv_color_hex(0xFFD400), 0);
    lv_label_set_text_fmt(lbl_angle, "%.1f\xC2\xB0", tilt);
}

void init_imu() {
    imu_ok = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
    if (!imu_ok) imu_ok = imu.begin(Wire, QMI8658_H_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
    Serial.printf("[IMU] %s\n", imu_ok ? "QMI8658 OK" : "FALHA ao iniciar");
    if (imu_ok) {
        imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_125Hz, SensorQMI8658::LPF_MODE_0);
        imu.enableAccelerometer();
    }
}

void build_level_ui(lv_obj_t *parent) {
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ring = lv_obj_create(parent);
    lv_obj_set_size(ring, 400, 400);
    lv_obj_align(ring, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ring, lv_color_hex(0x101820), 0);
    lv_obj_set_style_border_width(ring, 4, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0x808080), 0);

    static lv_point_t hline[2] = {{40, 240}, {440, 240}};
    static lv_point_t vline[2] = {{240, 40}, {240, 440}};
    static lv_style_t style_cross;
    lv_style_init(&style_cross);
    lv_style_set_line_width(&style_cross, 2);
    lv_style_set_line_color(&style_cross, lv_color_hex(0x606060));
    lv_obj_t *h = lv_line_create(parent);
    lv_line_set_points(h, hline, 2);
    lv_obj_add_style(h, &style_cross, 0);
    lv_obj_t *v = lv_line_create(parent);
    lv_line_set_points(v, vline, 2);
    lv_obj_add_style(v, &style_cross, 0);

    lv_obj_t *target = lv_obj_create(parent);
    lv_obj_set_size(target, 90, 90);
    lv_obj_align(target, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(target, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(target, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(target, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(target, 3, 0);
    lv_obj_set_style_border_color(target, lv_color_white(), 0);

    bubble = lv_obj_create(parent);
    lv_obj_set_size(bubble, 70, 70);
    lv_obj_align(bubble, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(bubble, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(bubble, lv_color_hex(0xFFD400), 0);
    lv_obj_set_style_bg_opa(bubble, LV_OPA_80, 0);
    lv_obj_set_style_border_width(bubble, 3, 0);
    lv_obj_set_style_border_color(bubble, lv_color_black(), 0);

    lbl_angle = lv_label_create(parent);
    lv_label_set_text(lbl_angle, "0.0\xC2\xB0");
    lv_obj_set_style_text_font(lbl_angle, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(lbl_angle, lv_color_white(), 0);
    lv_obj_align(lbl_angle, LV_ALIGN_BOTTOM_MID, 0, -8);

    init_imu();
    lv_timer_create(level_timer_cb, 33, NULL);
}

// -----------------------------------------------------------------------------
// Estação meteorológica (Open-Meteo, sem chave de API)
// -----------------------------------------------------------------------------
#define WX_HOURS 24

struct WeatherData {
    bool valid;
    char city[32];
    float temp, hum, uv, pres;
    float h_temp[WX_HOURS], h_hum[WX_HOURS], h_uv[WX_HOURS], h_pres[WX_HOURS];
};

static WeatherData wx_shared;       // escrito pela tarefa de rede
static volatile bool wx_dirty = false;
static portMUX_TYPE wx_mux = portMUX_INITIALIZER_UNLOCKED;

static bool resolve_location(float &lat, float &lon, char *city, size_t city_len) {
    if (strlen(LATITUDE) > 0 && strlen(LONGITUDE) > 0) {
        lat = atof(LATITUDE);
        lon = atof(LONGITUDE);
        strlcpy(city, strlen(CITY_NAME) ? CITY_NAME : "Local", city_len);
        return true;
    }
    HTTPClient http;
    http.begin("http://ip-api.com/json?fields=status,city,lat,lon");
    http.setTimeout(8000);
    bool ok = false;
    if (http.GET() == 200) {
        JsonDocument doc;
        if (!deserializeJson(doc, http.getString()) && doc["status"] == "success") {
            lat = doc["lat"];
            lon = doc["lon"];
            strlcpy(city, doc["city"] | "Local", city_len);
            ok = true;
        }
    }
    http.end();
    return ok;
}

static bool fetch_weather() {
    static float lat = 0, lon = 0;
    static char city[32] = "";
    static bool located = false;
    if (!located) located = resolve_location(lat, lon, city, sizeof(city));
    if (!located) {
        Serial.println("[WX] Localizacao indisponivel");
        return false;
    }

    char url[400];
    snprintf(url, sizeof(url),
             "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&current=temperature_2m,relative_humidity_2m,pressure_msl,uv_index"
             "&hourly=temperature_2m,relative_humidity_2m,pressure_msl,uv_index"
             "&forecast_days=2&timezone=auto", lat, lon);

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setTimeout(10000);
    if (!http.begin(client, url)) return false;
    int code = http.GET();
    if (code != 200) {
        Serial.printf("[WX] HTTP %d\n", code);
        http.end();
        return false;
    }
    String body = http.getString();
    http.end();

    JsonDocument filter;
    filter["current"]["time"] = true;
    filter["current"]["temperature_2m"] = true;
    filter["current"]["relative_humidity_2m"] = true;
    filter["current"]["pressure_msl"] = true;
    filter["current"]["uv_index"] = true;
    filter["hourly"] = true;

    JsonDocument doc;
    if (deserializeJson(doc, body, DeserializationOption::Filter(filter))) {
        Serial.println("[WX] JSON invalido");
        return false;
    }

    static WeatherData w;
    memset(&w, 0, sizeof(w));
    strlcpy(w.city, city, sizeof(w.city));
    w.temp = doc["current"]["temperature_2m"];
    w.hum  = doc["current"]["relative_humidity_2m"];
    w.pres = doc["current"]["pressure_msl"];
    w.uv   = doc["current"]["uv_index"];

    JsonArray times = doc["hourly"]["time"];
    const char *now_str = doc["current"]["time"] | "";
    int start = 0;
    for (int i = 0; i < (int)times.size(); i++) {
        const char *t = times[i] | "";
        if (strncmp(t, now_str, 13) == 0) { start = i; break; }
    }
    for (int i = 0; i < WX_HOURS; i++) {
        int idx = start + i;
        w.h_temp[i] = doc["hourly"]["temperature_2m"][idx] | w.temp;
        w.h_hum[i]  = doc["hourly"]["relative_humidity_2m"][idx] | w.hum;
        w.h_uv[i]   = doc["hourly"]["uv_index"][idx] | 0.0f;
        w.h_pres[i] = doc["hourly"]["pressure_msl"][idx] | w.pres;
    }
    w.valid = true;

    portENTER_CRITICAL(&wx_mux);
    wx_shared = w;
    wx_dirty = true;
    portEXIT_CRITICAL(&wx_mux);
    Serial.printf("[WX] %s: %.1fC %.0f%% UV %.1f %.0fhPa\n", w.city, w.temp, w.hum, w.uv, w.pres);
    return true;
}

enum { WX_TEMP, WX_HUM, WX_UV, WX_PRES, WX_COUNT };
static WeatherData wx;
static int wx_metric = WX_TEMP;
static lv_obj_t *arc_hum, *arc_uv, *arc_pres;
static lv_obj_t *lbl_wx_temp, *lbl_wx_city, *lbl_wx_hum, *lbl_wx_uv, *lbl_wx_pres;
static lv_obj_t *wx_chart, *lbl_chart_title, *lbl_chart_range;
static lv_chart_series_t *wx_series;

static const uint32_t COL_TEMP = 0xFF7A45, COL_HUM = 0x38BDF8, COL_UV = 0xFACC15, COL_PRES = 0xA78BFA;

static lv_obj_t *make_arc(lv_obj_t *parent, int size, uint32_t color, int min, int max) {
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, size, size);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_range(arc, min, max);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(arc, 12, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x1C1C22), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 12, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_INDICATOR);
    lv_arc_set_value(arc, min);
    return arc;
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    return l;
}

static void wx_refresh_chart() {
    if (!wx.valid) return;
    const float *src = wx.h_temp;
    uint32_t col = COL_TEMP;
    const char *title = "TEMPERATURA 24h";
    const char *unit = "\xC2\xB0";
    switch (wx_metric) {
        case WX_HUM:  src = wx.h_hum;  col = COL_HUM;  title = "UMIDADE 24h";  unit = "%"; break;
        case WX_UV:   src = wx.h_uv;   col = COL_UV;   title = "INDICE UV 24h"; unit = "";  break;
        case WX_PRES: src = wx.h_pres; col = COL_PRES; title = "PRESSAO 24h";  unit = "hPa"; break;
    }
    float mn = src[0], mx = src[0];
    for (int i = 1; i < WX_HOURS; i++) { mn = fminf(mn, src[i]); mx = fmaxf(mx, src[i]); }
    float pad = fmaxf((mx - mn) * 0.15f, wx_metric == WX_UV ? 0.5f : 1.0f);
    float lo = mn - pad, hi = mx + pad;
    if (wx_metric == WX_UV) lo = 0;

    lv_chart_set_range(wx_chart, LV_CHART_AXIS_PRIMARY_Y, (int)(lo * 10), (int)(hi * 10));
    lv_obj_set_style_line_color(wx_chart, lv_color_hex(col), LV_PART_ITEMS);
    lv_chart_set_series_color(wx_chart, wx_series, lv_color_hex(col));
    for (int i = 0; i < WX_HOURS; i++) lv_chart_set_value_by_id(wx_chart, wx_series, i, (int)(src[i] * 10));
    lv_chart_refresh(wx_chart);

    lv_label_set_text(lbl_chart_title, title);
    lv_obj_set_style_text_color(lbl_chart_title, lv_color_hex(col), 0);
    lv_label_set_text_fmt(lbl_chart_range, "%.1f%s  -  %.1f%s", mn, unit, mx, unit);
}

static void wx_refresh_values() {
    if (!wx.valid) return;
    lv_label_set_text_fmt(lbl_wx_temp, "%.0f\xC2\xB0", wx.temp);
    lv_label_set_text(lbl_wx_city, wx.city);
    lv_label_set_text_fmt(lbl_wx_hum, "%.0f%%", wx.hum);
    lv_label_set_text_fmt(lbl_wx_uv, "UV %.1f", wx.uv);
    lv_label_set_text_fmt(lbl_wx_pres, "%.0f hPa", wx.pres);
    lv_arc_set_value(arc_hum, (int)wx.hum);
    lv_arc_set_value(arc_uv, (int)roundf(fminf(wx.uv, 12.0f) * 10));
    lv_arc_set_value(arc_pres, (int)constrain(wx.pres, 960.0f, 1060.0f));
    wx_refresh_chart();
}

static void wx_timer_cb(lv_timer_t *t) {
    if (!wx_dirty) return;
    portENTER_CRITICAL(&wx_mux);
    wx = wx_shared;
    wx_dirty = false;
    portEXIT_CRITICAL(&wx_mux);
    wx_refresh_values();
}

static uint32_t last_gesture_ms = 0;

static void wx_tap_cb(lv_event_t *e) {
    if (millis() - last_gesture_ms < 500) return;  // ignora o toque gerado ao final de um arrasto
    wx_metric = (wx_metric + 1) % WX_COUNT;
    wx_refresh_chart();
}

void build_weather_ui(lv_obj_t *parent) {
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    arc_hum  = make_arc(parent, 460, COL_HUM, 0, 100);
    arc_uv   = make_arc(parent, 424, COL_UV, 0, 120);
    arc_pres = make_arc(parent, 388, COL_PRES, 960, 1060);

    lbl_wx_city = make_label(parent, &lv_font_montserrat_20, 0x9CA3AF);
    lv_label_set_text(lbl_wx_city, "Aguardando dados...");
    lv_obj_align(lbl_wx_city, LV_ALIGN_CENTER, 0, -150);

    lbl_wx_temp = make_label(parent, &lv_font_montserrat_48, COL_TEMP);
    lv_label_set_text(lbl_wx_temp, "--");
    lv_obj_align(lbl_wx_temp, LV_ALIGN_CENTER, 0, -108);

    lbl_wx_hum = make_label(parent, &lv_font_montserrat_20, COL_HUM);
    lv_label_set_text(lbl_wx_hum, "--%");
    lv_obj_align(lbl_wx_hum, LV_ALIGN_CENTER, -90, -62);
    lbl_wx_uv = make_label(parent, &lv_font_montserrat_20, COL_UV);
    lv_label_set_text(lbl_wx_uv, "UV --");
    lv_obj_align(lbl_wx_uv, LV_ALIGN_CENTER, 0, -62);
    lbl_wx_pres = make_label(parent, &lv_font_montserrat_20, COL_PRES);
    lv_label_set_text(lbl_wx_pres, "---- hPa");
    lv_obj_align(lbl_wx_pres, LV_ALIGN_CENTER, 95, -62);

    lbl_chart_title = make_label(parent, &lv_font_montserrat_14, COL_TEMP);
    lv_label_set_text(lbl_chart_title, "TEMPERATURA 24h");
    lv_obj_align(lbl_chart_title, LV_ALIGN_CENTER, 0, -25);

    wx_chart = lv_chart_create(parent);
    lv_obj_set_size(wx_chart, 280, 120);
    lv_obj_align(wx_chart, LV_ALIGN_CENTER, 0, 50);
    lv_chart_set_type(wx_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(wx_chart, WX_HOURS);
    lv_chart_set_div_line_count(wx_chart, 3, 0);
    lv_obj_set_style_bg_opa(wx_chart, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wx_chart, 0, 0);
    lv_obj_set_style_pad_all(wx_chart, 2, 0);
    lv_obj_set_style_line_color(wx_chart, lv_color_hex(0x25252B), LV_PART_MAIN);
    lv_obj_set_style_size(wx_chart, 0, LV_PART_INDICATOR);
    lv_obj_set_style_line_width(wx_chart, 3, LV_PART_ITEMS);
    lv_obj_clear_flag(wx_chart, LV_OBJ_FLAG_CLICKABLE);
    wx_series = lv_chart_add_series(wx_chart, lv_color_hex(COL_TEMP), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(wx_chart, wx_series, LV_CHART_POINT_NONE);

    lbl_chart_range = make_label(parent, &lv_font_montserrat_14, 0x9CA3AF);
    lv_label_set_text(lbl_chart_range, "");
    lv_obj_align(lbl_chart_range, LV_ALIGN_CENTER, 0, 125);

    // Toque na tela alterna o gráfico entre temperatura, umidade, UV e pressão
    lv_obj_add_event_cb(parent, wx_tap_cb, LV_EVENT_CLICKED, NULL);
    lv_timer_create(wx_timer_cb, 500, NULL);
}

static lv_obj_t *lbl_c_date, *lbl_c_year, *lbl_c_wday, *lbl_c_temp;

static lv_obj_t *make_corner_label(lv_obj_t *parent, lv_align_t align, int dx, int dy) {
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    lv_obj_set_style_bg_color(l, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(l, LV_OPA_70, 0);
    lv_obj_set_style_radius(l, 8, 0);
    lv_obj_set_style_pad_all(l, 4, 0);
    lv_obj_align(l, align, dx, dy);
    return l;
}

static void corner_info_timer_cb(lv_timer_t *t) {
    static const char *wdays[] = {"Domingo", "Segunda-feira", "Terca-feira", "Quarta-feira",
                                  "Quinta-feira", "Sexta-feira", "Sabado"};
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    if (tm.tm_year >= 120) {
        lv_label_set_text_fmt(lbl_c_date, "%02d/%02d", tm.tm_mday, tm.tm_mon + 1);
        lv_label_set_text_fmt(lbl_c_year, "%04d", tm.tm_year + 1900);
        lv_label_set_text(lbl_c_wday, wdays[tm.tm_wday]);
    }
    if (wx.valid) lv_label_set_text_fmt(lbl_c_temp, "%.0f\xC2\xB0" "C", wx.temp);
}

// -----------------------------------------------------------------------------
// Navegação: relógio <-> tela inicial (ícones) <-> aplicativos
// Arrastar para cima, em qualquer tela, abre a tela inicial; para baixo volta ao relógio.
// -----------------------------------------------------------------------------
static void no_click(lv_obj_t *o) {
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}

// -----------------------------------------------------------------------------
// Assistente de IA por voz (microfone ES7210 -> Gemini)
// -----------------------------------------------------------------------------
#ifndef GEMINI_API_KEY
#define GEMINI_API_KEY ""
#endif
#ifndef GEMINI_MODEL
#define GEMINI_MODEL "gemini-3.5-flash-lite"
#endif

#define MIC_RATE 16000
#define MIC_MAX_SEC 10
#define MIC_MAX_SAMPLES (MIC_RATE * MIC_MAX_SEC)
#define PIN_I2S_MCLK 42
#define PIN_I2S_BCLK 9
#define PIN_I2S_WS 45
#define PIN_I2S_DIN 10
#define ES7210_ADDR 0x40
#define ES8311_ADDR 0x18
#define PIN_I2S_DOUT 8
#define PIN_PA_EN 46

static I2SClass i2s_mic;
static bool mic_ok = false;
static SemaphoreHandle_t wifi_mtx = NULL;

enum { AI_IDLE = 0, AI_LISTENING, AI_THINKING, AI_SPEAKING, AI_DONE };
static volatile int ai_state = AI_IDLE;
static volatile bool ai_rec = false;
static volatile uint32_t ai_rec_ms = 0;
static char ai_text[1024] = "";
static volatile bool ai_dirty = true;
static portMUX_TYPE ai_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t ai_task_handle = NULL;

static bool es7210_w(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES7210_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Sequência de inicialização baseada no driver ES7210 da Espressif (Apache-2.0):
// 16 kHz, MCLK 4,096 MHz (256 x fs), I2S 16 bits, microfones 1/2 ligados.
static bool es7210_init_16k() {
    bool ok = es7210_w(0x00, 0xFF);
    es7210_w(0x00, 0x32);
    es7210_w(0x09, 0x30);
    es7210_w(0x0A, 0x30);
    es7210_w(0x23, 0x2A);
    es7210_w(0x22, 0x0A);
    es7210_w(0x21, 0x2A);
    es7210_w(0x20, 0x0A);
    es7210_w(0x11, 0x00 | 0x60);
    es7210_w(0x12, 0x00);
    es7210_w(0x40, 0xC3);
    es7210_w(0x41, 0x70);
    es7210_w(0x42, 0x70);
    for (uint8_t r = 0x43; r <= 0x46; r++) es7210_w(r, 0x10 | 12);  // ~34,5 dB
    for (uint8_t r = 0x47; r <= 0x4A; r++) es7210_w(r, 0x08);
    es7210_w(0x02, 0x01 | (0x01 << 6) | (0x01 << 7));  // adc_div 1, doubler, dll
    es7210_w(0x07, 0x20);
    es7210_w(0x03, 0x00);
    es7210_w(0x04, 0x01);
    es7210_w(0x05, 0x00);
    es7210_w(0x06, 0x04);
    es7210_w(0x4B, 0x0F);
    es7210_w(0x4C, 0x0F);
    es7210_w(0x00, 0x71);
    es7210_w(0x00, 0x41);
    return ok;
}

static void es8311_w(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8311_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Baseado no driver ES8311 da Espressif (Apache-2.0): 16 kHz, MCLK 4,096 MHz, I2S 16 bits, escravo
static void es8311_init_16k() {
    es8311_w(0x00, 0x1F);
    delay(20);
    es8311_w(0x00, 0x00);
    es8311_w(0x00, 0x80);
    es8311_w(0x01, 0x3F);
    es8311_w(0x02, 0x00);
    es8311_w(0x03, 0x10);
    es8311_w(0x04, 0x10);
    es8311_w(0x05, 0x00);
    es8311_w(0x06, 0x03);
    es8311_w(0x07, 0x00);
    es8311_w(0x08, 0xFF);
    es8311_w(0x09, 0x0C);
    es8311_w(0x0A, 0x0C);
    es8311_w(0x0D, 0x01);
    es8311_w(0x0E, 0x02);
    es8311_w(0x12, 0x00);
    es8311_w(0x13, 0x10);
    es8311_w(0x1C, 0x6A);
    es8311_w(0x37, 0x08);
    es8311_w(0x31, 0x00);
    es8311_w(0x32, 0xD0);
}

static void init_audio() {
    i2s_mic.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK);
    pinMode(PIN_PA_EN, OUTPUT);
    digitalWrite(PIN_PA_EN, LOW);
    if (!i2s_mic.begin(I2S_MODE_STD, MIC_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
        Serial.println("[MIC] Falha ao iniciar I2S");
        return;
    }
    delay(20);
    if (!es7210_init_16k()) {
        Serial.println("[MIC] ES7210 nao respondeu");
        return;
    }
    es8311_init_16k();
    i2s_mic.setTimeout(200);
    mic_ok = true;
    Serial.println("[MIC] ES7210 OK");
}

static void ai_set(int state, const char *text) {
    portENTER_CRITICAL(&ai_mux);
    if (text) {
        strlcpy(ai_text, text, sizeof(ai_text));
        ai_dirty = true;
    }
    ai_state = state;
    portEXIT_CRITICAL(&ai_mux);
}

// A fonte padrão só tem ASCII: converte acentos do UTF-8 e descarta o resto
static void to_ascii(const char *src, char *dst, size_t dst_len) {
    static const char *map_c3 = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTsaaaaaaaceeeeiiiidnooooo/ouuuuyty";
    size_t o = 0;
    for (const uint8_t *p = (const uint8_t *)src; *p && o + 1 < dst_len; p++) {
        if (*p < 0x80) {
            dst[o++] = (*p == '\n' || *p >= 32) ? *p : ' ';
        } else if (*p == 0xC3 && p[1] >= 0x80 && p[1] < 0xC0) {
            dst[o++] = map_c3[p[1] - 0x80];
            p++;
        } else if (*p == 0xE2 && p[1] == 0x80) {
            uint8_t c = p[2];
            dst[o++] = (c == 0x93 || c == 0x94) ? '-' : (c == 0x98 || c == 0x99) ? '\'' : (c == 0x9C || c == 0x9D) ? '"' : (c == 0xA2) ? '*' : ' ';
            p += 2;
        } else {
            int skip = (*p >= 0xF0) ? 3 : (*p >= 0xE0) ? 2 : (*p >= 0xC0) ? 1 : 0;
            for (int i = 0; i < skip && p[1]; i++) p++;
        }
    }
    dst[o] = 0;
}

static bool ai_wifi_up() {
    if (WiFi.status() == WL_CONNECTED) return true;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(400));
    return WiFi.status() == WL_CONNECTED;
}

static void ai_wifi_down() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

static void ai_put_le(uint8_t *p, uint32_t v, int n) {
    for (int i = 0; i < n; i++) p[i] = (v >> (8 * i)) & 0xFF;
}

#ifndef GEMINI_TTS_MODEL
#define GEMINI_TTS_MODEL "gemini-3.1-flash-tts-preview"
#endif
#define TTS_MAX_BYTES (24000 * 2 * 25)

static int b64v(int c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

// Pede a fala ao Gemini e devolve PCM 16 bit 24 kHz mono (buffer em PSRAM) ou NULL
static uint8_t *ai_tts_fetch(const char *text, size_t *out_bytes) {
    JsonDocument req;
    String prompt = String("Diga de forma natural e simpatica: ") + text;
    req["contents"][0]["parts"][0]["text"] = prompt;
    req["generationConfig"]["responseModalities"][0] = "AUDIO";
    req["generationConfig"]["speechConfig"]["voiceConfig"]["prebuiltVoiceConfig"]["voiceName"] = "Kore";
    String body;
    serializeJson(req, body);

    uint8_t *pcm = (uint8_t *)heap_caps_malloc(TTS_MAX_BYTES, MALLOC_CAP_SPIRAM);
    if (!pcm) return NULL;
    size_t n = 0;

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.useHTTP10(true);  // sem chunked: o corpo pode ser lido direto do stream
    http.setTimeout(45000);
    http.begin(client, String("https://generativelanguage.googleapis.com/v1beta/models/") + GEMINI_TTS_MODEL + ":generateContent");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-goog-api-key", GEMINI_API_KEY);
    int code = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        code = http.POST(body);
        if (code != 503 && code != 500) break;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    Serial.printf("[TTS] HTTP %d\n", code);
    if (code == 200) {
        WiFiClient *st = http.getStreamPtr();
        const char key[] = "\"data\":";
        int ki = 0, state = 0;  // 0 procura chave, 1 procura aspas, 2 le base64
        int q[4], qn = 0;
        uint32_t last = millis();
        while ((http.connected() || st->available()) && millis() - last < 20000) {
            if (!st->available()) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            int c = st->read();
            last = millis();
            if (state == 0) {
                ki = (c == key[ki]) ? ki + 1 : (c == key[0] ? 1 : 0);
                if (key[ki] == 0) state = 1;
            } else if (state == 1) {
                if (c == '"') state = 2;
            } else {
                if (c == '"') break;
                int v = b64v(c);
                if (v < 0) continue;
                q[qn++] = v;
                if (qn == 4) {
                    if (n + 3 > TTS_MAX_BYTES) break;
                    pcm[n++] = (q[0] << 2) | (q[1] >> 4);
                    pcm[n++] = ((q[1] & 15) << 4) | (q[2] >> 2);
                    pcm[n++] = ((q[2] & 3) << 6) | q[3];
                    qn = 0;
                }
            }
        }
    }
    http.end();
    if (n < 4800) {
        free(pcm);
        return NULL;
    }
    *out_bytes = n & ~1u;
    return pcm;
}

// Reamostra 24 kHz -> 16 kHz (linear) e toca pelo ES8311 + amplificador
static void ai_play_pcm24(const int16_t *in, size_t n) {
    digitalWrite(PIN_PA_EN, HIGH);
    static int16_t out[256 * 2];
    size_t out_n = n * 2 / 3, i = 0;
    while (i < out_n) {
        size_t cnt = min<size_t>(256, out_n - i);
        for (size_t k = 0; k < cnt; k++) {
            float pos = (i + k) * 1.5f;
            size_t i0 = (size_t)pos;
            float f = pos - i0;
            int a = in[i0], b = (i0 + 1 < n) ? in[i0 + 1] : a;
            int16_t v = (int16_t)(a + (b - a) * f);
            out[2 * k] = v;
            out[2 * k + 1] = v;
        }
        i2s_mic.write((uint8_t *)out, cnt * 4);
        i += cnt;
    }
    memset(out, 0, sizeof(out));
    for (int k = 0; k < 4; k++) i2s_mic.write((uint8_t *)out, sizeof(out));
    digitalWrite(PIN_PA_EN, LOW);
}

static void ai_query(int16_t *pcm, size_t n) {
    static const char prefix[] =
        "{\"contents\":[{\"parts\":[{\"text\":\"Voce e um assistente de voz em um relogio de mesa. "
        "Responda em portugues do Brasil ao que foi dito no audio, em no maximo 3 frases curtas "
        "(ate 160 caracteres), sem markdown e sem emojis.\"},{\"inline_data\":{\"mime_type\":\"audio/wav\",\"data\":\"";
    static const char suffix[] = "\"}}]}]}";

    size_t wav_len = 44 + n * 2;
    uint8_t *wav = (uint8_t *)heap_caps_malloc(wav_len, MALLOC_CAP_SPIRAM);
    size_t b64_cap = 4 * ((wav_len + 2) / 3) + 1;
    size_t body_cap = sizeof(prefix) + b64_cap + sizeof(suffix);
    char *body = (char *)heap_caps_malloc(body_cap, MALLOC_CAP_SPIRAM);
    if (!wav || !body) {
        free(wav);
        free(body);
        ai_set(AI_DONE, "Memoria insuficiente.");
        return;
    }
    memcpy(wav, "RIFF", 4);
    ai_put_le(wav + 4, wav_len - 8, 4);
    memcpy(wav + 8, "WAVEfmt ", 8);
    ai_put_le(wav + 16, 16, 4);
    ai_put_le(wav + 20, 1, 2);
    ai_put_le(wav + 22, 1, 2);
    ai_put_le(wav + 24, MIC_RATE, 4);
    ai_put_le(wav + 28, MIC_RATE * 2, 4);
    ai_put_le(wav + 32, 2, 2);
    ai_put_le(wav + 34, 16, 2);
    memcpy(wav + 36, "data", 4);
    ai_put_le(wav + 40, n * 2, 4);
    memcpy(wav + 44, pcm, n * 2);

    size_t pl = strlen(prefix), olen = 0;
    memcpy(body, prefix, pl);
    mbedtls_base64_encode((uint8_t *)body + pl, b64_cap, &olen, wav, wav_len);
    memcpy(body + pl + olen, suffix, sizeof(suffix));
    size_t body_len = pl + olen + sizeof(suffix) - 1;
    free(wav);

    if (xSemaphoreTake(wifi_mtx, pdMS_TO_TICKS(40000)) != pdTRUE) {
        free(body);
        ai_set(AI_DONE, "Wi-Fi ocupado, tente de novo.");
        return;
    }
    if (!ai_wifi_up()) {
        ai_wifi_down();
        xSemaphoreGive(wifi_mtx);
        free(body);
        ai_set(AI_DONE, "Sem Wi-Fi.");
        return;
    }

    String resp;
    int code;
    {
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        http.setTimeout(45000);
        http.begin(client, String("https://generativelanguage.googleapis.com/v1beta/models/") + GEMINI_MODEL + ":generateContent");
        http.addHeader("Content-Type", "application/json");
        http.addHeader("x-goog-api-key", GEMINI_API_KEY);
        for (int attempt = 0; attempt < 3; attempt++) {
            code = http.POST((uint8_t *)body, body_len);
            if (code > 0) resp = http.getString();
            if (code != 503) break;
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        http.end();
    }
    free(body);

    char raw[1024], out[1024];
    raw[0] = 0;
    bool ok = false;
    if (code == 200) {
        JsonDocument doc;
        if (!deserializeJson(doc, resp)) {
            const char *t = doc["candidates"][0]["content"]["parts"][0]["text"] | "";
            strlcpy(raw, t, sizeof(raw));
        }
        ok = raw[0] != 0;
        if (!ok) strlcpy(raw, "Resposta vazia da IA.", sizeof(raw));
    } else if (code == 429) {
        strlcpy(raw, "Limite gratuito da IA atingido. Aguarde um pouco.", sizeof(raw));
    } else if (code == 400 || code == 403) {
        snprintf(raw, sizeof(raw), "Erro %d: verifique GEMINI_API_KEY no .env.", code);
    } else {
        snprintf(raw, sizeof(raw), "Falha na consulta (codigo %d).", code);
    }
    Serial.printf("[AI] HTTP %d\n", code);

    size_t tts_bytes = 0;
    uint8_t *tts = ok ? ai_tts_fetch(raw, &tts_bytes) : NULL;
    ai_wifi_down();
    xSemaphoreGive(wifi_mtx);

    if (tts) {
        ai_set(AI_SPEAKING, "");
        ai_play_pcm24((int16_t *)tts, tts_bytes / 2);
        free(tts);
        ai_set(AI_DONE, "");
    } else {
        to_ascii(raw, out, sizeof(out));
        ai_set(AI_DONE, out);
    }
}

static void ai_record_and_ask() {
    int16_t *pcm = (int16_t *)heap_caps_malloc(MIC_MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!pcm) {
        ai_set(AI_DONE, "Memoria insuficiente.");
        return;
    }
    static int16_t tmp[512 * 2];
    // Descarta áudio acumulado antes do toque
    for (int i = 0; i < 4; i++) i2s_mic.readBytes((char *)tmp, sizeof(tmp));

    size_t n = 0;
    int dry = 0;
    ai_rec_ms = millis();
    while (ai_rec && n < MIC_MAX_SAMPLES - 512) {
        size_t got = i2s_mic.readBytes((char *)tmp, sizeof(tmp));
        if (got == 0) {
            if (++dry > 10) break;
            continue;
        }
        dry = 0;
        for (size_t i = 0; i + 1 < got / 2; i += 2) pcm[n++] = ((int)tmp[i] + (int)tmp[i + 1]) / 2;
    }
    ai_rec = false;

    int peak = 1;
    for (size_t i = 0; i < n; i++) peak = max(peak, abs((int)pcm[i]));
    Serial.printf("[AI] %u amostras, pico %d\n", (unsigned)n, peak);

    if (n < MIC_RATE / 2) {
        ai_set(AI_DONE, "Toque curto demais. Segure a tela enquanto fala.");
    } else if (peak < 150) {
        ai_set(AI_DONE, "Nao captei som. Fale mais perto do aparelho.");
    } else {
        float gain = min(16.0f, 24000.0f / peak);
        for (size_t i = 0; i < n; i++) pcm[i] = (int16_t)constrain((int)(pcm[i] * gain), -32768, 32767);
        ai_set(AI_THINKING, NULL);
        ai_query(pcm, n);
    }
    free(pcm);
}

static void ai_task(void *arg) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ai_record_and_ask();
    }
}

static lv_obj_t *lbl_ai_text, *lbl_ai_status, *ai_box;
static int ai_shown_state = -1;
extern uint32_t talk_until;

static void ai_btn_cb(lv_event_t *e) {
    lv_event_code_t c = lv_event_get_code(e);
    if (c == LV_EVENT_LONG_PRESSED) {
        if (!mic_ok) {
            ai_set(AI_DONE, "Microfone nao iniciou.");
        } else if (!strlen(GEMINI_API_KEY)) {
            ai_set(AI_DONE, "Defina GEMINI_API_KEY no .env (chave gratuita em aistudio.google.com).");
        } else if (ai_state != AI_LISTENING && ai_state != AI_THINKING && ai_state != AI_SPEAKING) {
            ai_rec = true;
            ai_set(AI_LISTENING, "");
            xTaskNotifyGive(ai_task_handle);
        }
    } else if (c == LV_EVENT_RELEASED || c == LV_EVENT_PRESS_LOST) {
        ai_rec = false;
    }
}

static void ai_timer_cb(lv_timer_t *t) {
    if (ai_dirty) {
        char buf[1024];
        portENTER_CRITICAL(&ai_mux);
        strlcpy(buf, ai_text, sizeof(buf));
        ai_dirty = false;
        portEXIT_CRITICAL(&ai_mux);
        lv_label_set_text(lbl_ai_text, buf);
        if (buf[0]) lv_obj_clear_flag(ai_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(ai_box, LV_OBJ_FLAG_HIDDEN);
    }
    int st = ai_state;
    if (st == AI_LISTENING) {
        lv_label_set_text_fmt(lbl_ai_status, "Ouvindo... %us\nSolte a tela quando terminar", (unsigned)((millis() - ai_rec_ms) / 1000));
    } else if (st != ai_shown_state) {
        lv_label_set_text(lbl_ai_status, st == AI_THINKING ? "Pensando...\nConsultando a IA" : st == AI_SPEAKING ? "Falando...\nOuca a resposta" : "Segure a tela e fale\nArraste para cima para sair");
    }
    if (st == AI_DONE && ai_shown_state != AI_DONE && lv_label_get_text(lbl_ai_text)[0]) talk_until = millis() + min<uint32_t>(8000, strlen(lv_label_get_text(lbl_ai_text)) * 55);
    ai_shown_state = st;
}

// Robô em pixel art (grade 64x64 ampliada) desenhado em um canvas
#define SPR_W 64
#define SPR_H 64
static lv_color_t spr_buf[SPR_W * SPR_H];
static lv_obj_t *spr_canvas;
uint32_t talk_until = 0;
static int spr_dy = 0;

enum : uint32_t {
    C_OUT = 0x2A3F8F, C_LIGHT = 0xE4F0FF, C_MID = 0xAFCFF5, C_SHADE = 0x7BA3DD,
    C_SCREEN = 0x0A1630, C_CYAN = 0x5CF0F0, C_CHEEK = 0xF27A8A, C_GLOW = 0x7CFFD8,
    C_PANEL = 0x3A6FD0, C_DIM = 0x7777B0
};

static inline void spr_px(int x, int y, uint32_t c) {
    y += spr_dy;
    if (x >= 0 && x < SPR_W && y >= 0 && y < SPR_H) spr_buf[y * SPR_W + x] = lv_color_hex(c);
}

static void spr_rect(int x, int y, int w, int h, uint32_t c) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) spr_px(x + i, y + j, c);
}

static void spr_ell(float cx, float cy, float rx, float ry, uint32_t c) {
    for (int y = (int)(cy - ry - 1); y <= (int)(cy + ry + 1); y++)
        for (int x = (int)(cx - rx - 1); x <= (int)(cx + rx + 1); x++) {
            float ux = (x - cx) / rx, uy = (y - cy) / ry;
            if (ux * ux + uy * uy <= 1.0f) spr_px(x, y, c);
        }
}

// Elipse com contorno e sombreamento (luz vinda de cima à esquerda)
static void spr_shell(float cx, float cy, float rx, float ry) {
    spr_ell(cx, cy, rx + 1, ry + 1, C_OUT);
    for (int y = (int)(cy - ry - 1); y <= (int)(cy + ry + 1); y++)
        for (int x = (int)(cx - rx - 1); x <= (int)(cx + rx + 1); x++) {
            float ux = (x - cx) / rx, uy = (y - cy) / ry;
            if (ux * ux + uy * uy > 1.0f) continue;
            float n = -0.5f * ux - 0.7f * uy;
            spr_px(x, y, n > 0.25f ? C_LIGHT : (n < -0.35f ? C_SHADE : C_MID));
        }
}

static void spr_rrect(int x, int y, int w, int h, int r, uint32_t c, int ymin = -999) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) {
            if (y + j < ymin) continue;
            int dx = i < r ? r - i : (i >= w - r ? i - (w - r - 1) : 0);
            int dy = j < r ? r - j : (j >= h - r ? j - (h - r - 1) : 0);
            if (dx * dx + dy * dy <= r * r) spr_px(x + i, y + j, c);
        }
}

static void spr_line(int x0, int y0, int x1, int y1, uint32_t c) {
    int n = max(abs(x1 - x0), abs(y1 - y0));
    for (int i = 0; i <= n; i++) spr_px(x0 + (x1 - x0) * i / max(n, 1), y0 + (y1 - y0) * i / max(n, 1), c);
}

static void spr_gear(float cx, float cy, float r, float ang, uint32_t c) {
    for (int y = (int)(cy - r - 2); y <= (int)(cy + r + 2); y++)
        for (int x = (int)(cx - r - 2); x <= (int)(cx + r + 2); x++) {
            float dx = x - cx, dy = y - cy, d = sqrtf(dx * dx + dy * dy);
            if (d < r * 0.35f) continue;
            if (d <= r - 0.3f || (d <= r + 1.3f && cosf(8 * (atan2f(dy, dx) + ang)) > 0.1f)) spr_px(x, y, c);
        }
}

static void spr_note(int x, int y, uint32_t c) {
    spr_ell(x, y + 5, 1.8f, 1.4f, c);
    spr_rect(x + 1, y - 1, 1, 6, c);
    spr_px(x + 2, y, c);
    spr_px(x + 3, y + 1, c);
}

static void spr_ring_eye(int cx, int cy, float r, float pdx, float pdy) {
    spr_ell(cx, cy, r, r, C_CYAN);
    spr_ell(cx + pdx, cy + pdy, r - 1.4f, r - 1.4f, C_SCREEN);
    spr_px(cx - 1, cy - 1, 0xFFFFFF);
}

static void spr_arm(float cx, float cy, float rx, float ry, int hx, int hy) {
    spr_shell(cx, cy, rx, ry);
    spr_shell(hx, hy, 2.6f, 2.6f);
}

static void spr_draw(uint32_t now, int st, bool talking) {
    for (int i = 0; i < SPR_W * SPR_H; i++) spr_buf[i] = lv_color_black();
    enum { M_IDLE, M_LISTEN, M_THINK, M_SPEAK } mode =
        talking ? M_SPEAK : (st == AI_LISTENING ? M_LISTEN : (st == AI_THINKING ? M_THINK : M_IDLE));
    float t = now / 1000.0f;

    // Brilhos de fundo (só repouso e fala)
    spr_dy = 0;
    if (mode == M_IDLE || mode == M_SPEAK) {
        static const int8_t sp[5][2] = {{5, 20}, {58, 26}, {6, 50}, {57, 48}, {10, 8}};
        for (int i = 0; i < 5; i++) {
            int ph = (now / 300 + i * 4) % 8;
            if (ph > 3) continue;
            bool big = ph == 1 || ph == 2;
            spr_px(sp[i][0], sp[i][1], big ? 0xFFFFFF : C_DIM);
            if (big) {
                spr_px(sp[i][0] - 1, sp[i][1], C_DIM);
                spr_px(sp[i][0] + 1, sp[i][1], C_DIM);
                spr_px(sp[i][0], sp[i][1] - 1, C_DIM);
                spr_px(sp[i][0], sp[i][1] + 1, C_DIM);
            }
        }
    }
    spr_ell(32, 61, 15, 2, 0x1A1A38);

    int bob = (int)lroundf(sinf(t * (mode == M_SPEAK ? 5.0f : 2.2f)) * 1.5f);
    spr_dy = bob;

    // Pés
    spr_shell(25, 56, 3.5f, 2.2f);
    spr_shell(39, 56, 3.5f, 2.2f);

    // Braços
    float sway = sinf(t * 2.2f) * 1.2f;
    bool waving = mode == M_IDLE && (now % 8000) < 1600;
    float osc = sinf(t * 9.0f);
    if (mode == M_IDLE) {
        spr_arm(14, 43 + sway, 2.8f, 6, 13, 49 + sway);
        if (waving) spr_arm(51, 36, 2.8f, 6, 54 + osc * 2, 28);
        else spr_arm(50, 43 - sway, 2.8f, 6, 51, 49 - sway);
    } else if (mode == M_LISTEN) {
        spr_arm(12, 36, 2.8f, 6, 12, 27);
        spr_arm(50, 43, 2.8f, 6, 51, 49);
    } else if (mode == M_THINK) {
        spr_arm(14, 44, 2.8f, 5, 20, 47);
    } else {
        spr_arm(12, 36, 2.8f, 6, 10, 28 + osc);
        spr_arm(52, 36, 2.8f, 6, 54, 28 - osc);
    }

    // Corpo
    spr_shell(32, 45, 13, 11);
    spr_rrect(29, 46, 6, 3, 1, C_PANEL);

    // Antenas
    float pulse = (sinf(t * 4) + 1) * 0.5f;
    uint32_t glow = pulse > 0.5f ? 0xC8FFF0 : C_GLOW;
    spr_line(24, 11, 22, 7, C_OUT);
    spr_line(40, 11, 42, 7, C_OUT);
    spr_shell(22, 5.5f, 1.6f, 1.6f);
    spr_shell(42, 5.5f, 1.6f, 1.6f);
    spr_ell(22, 5.5f, 1.6f, 1.6f, glow);
    spr_ell(42, 5.5f, 1.6f, 1.6f, glow);

    // Cabeça
    spr_rrect(14, 10, 36, 32, 14, C_OUT);
    spr_rrect(15, 11, 34, 30, 13, C_LIGHT);
    spr_rrect(15, 11, 34, 30, 13, C_MID, 33);
    spr_ell(14, 27, 2, 4, C_OUT);
    spr_ell(50, 27, 2, 4, C_OUT);
    spr_ell(14, 27, 1, 3, C_SHADE);
    spr_ell(50, 27, 1, 3, C_SHADE);
    spr_rrect(17, 14, 30, 21, 9, C_OUT);
    spr_rrect(18, 15, 28, 19, 8, C_SCREEN);

    bool blink = mode != M_LISTEN && mode != M_SPEAK && (now % 3600) < 140;
    int ex = 26, ex2 = 38, ey = 23;
    if (mode == M_IDLE) {
        float look = sinf(t * 0.8f);
        float pdx = look > 0.6f ? 1 : (look < -0.6f ? -1 : 0);
        if (blink) { spr_rect(ex - 2, ey, 5, 1, C_CYAN); spr_rect(ex2 - 2, ey, 5, 1, C_CYAN); }
        else { spr_ring_eye(ex, ey, 3.6f, pdx * 0.6f, 0); spr_ring_eye(ex2, ey, 3.6f, pdx * 0.6f, 0); }
        spr_px(30, 30, C_CYAN); spr_rect(31, 31, 3, 1, C_CYAN); spr_px(34, 30, C_CYAN);
    } else if (mode == M_LISTEN) {
        spr_ring_eye(ex, ey, 4.5f, 0, 0);
        spr_ring_eye(ex2, ey, 4.5f, 0, 0);
        spr_ell(32, 31, 2, 2, C_CYAN);
        spr_ell(32, 31, 0.9f, 0.9f, C_SCREEN);
    } else if (mode == M_THINK) {
        if (blink) { spr_rect(ex - 2, ey, 5, 1, C_CYAN); spr_rect(ex2 - 2, ey, 5, 1, C_CYAN); }
        else { spr_ring_eye(ex, ey, 3.8f, 1.3f, -1.5f); spr_ring_eye(ex2, ey, 3.8f, 1.3f, -1.5f); }
        spr_px(30, 31, C_CYAN); spr_rect(31, 30, 3, 1, C_CYAN); spr_px(34, 31, C_CYAN);
    } else {
        for (int k = 0; k < 2; k++) {
            int cx = k ? ex2 : ex;
            for (int dx = -3; dx <= 3; dx++) {
                int y = ey + 1 - (int)lroundf(2.4f - dx * dx / 3.8f);
                spr_px(cx + dx, y, C_CYAN);
                spr_px(cx + dx, y + 1, C_CYAN);
            }
        }
        float open = (sinf(now / 75.0f) + 1.0f) * 0.5f;
        spr_ell(32, 30, 4, 1.2f + open * 2.4f, C_CYAN);
        spr_ell(32, 30.5f, 2.5f, 0.2f + open * 1.4f, 0x1B6E86);
    }
    spr_ell(22, 29, 2.6f, 1.6f, C_CHEEK);
    spr_ell(42, 29, 2.6f, 1.6f, C_CHEEK);

    // Mão no queixo (pensando) fica à frente da cabeça
    if (mode == M_THINK) spr_arm(40, 40, 2.8f, 5, 36, 37);

    // Efeitos
    spr_dy = 0;
    if (mode == M_LISTEN) {
        int ph = (now / 220) % 3;
        for (int k = 0; k <= ph; k++) {
            spr_line(7 - k * 2, 21, 7 - k * 2, 27, C_CYAN);
            spr_line(57 + k * 2, 21, 57 + k * 2, 27, C_CYAN);
        }
        if ((now / 350) & 1) {
            for (int k = 0; k < 2; k++) {
                spr_rect(54 + k * 4, 2, 2, 5, 0xBFD4FF);
                spr_rect(54 + k * 4, 8, 2, 2, 0xBFD4FF);
            }
        }
    } else if (mode == M_THINK) {
        float a = t * 1.5f;
        spr_gear(34, 4, 3.5f, a, C_SHADE);
        spr_gear(25, 2, 2, -a * 1.6f, C_DIM);
        spr_gear(45, 1, 2, -a * 1.6f, C_DIM);
        spr_ell(55, 8, 7, 5, C_LIGHT);
        spr_px(50, 14, C_LIGHT);
        spr_px(49, 16, C_LIGHT);
        spr_gear(55, 8, 3, a, C_SHADE);
    } else if (mode == M_SPEAK) {
        int f = (now / 400) & 1;
        spr_note(5, 14 - f, C_DIM);
        spr_note(57, 10 + f, C_DIM);
        for (int k = 0; k < 5; k++) {
            int h = 2 + (int)((sinf(now / 90.0f + k * 1.7f) + 1.0f) * 3);
            spr_rect(58 + k, 36 - h / 2, 1, h, 0x3AD0C0);
        }
    }
    lv_obj_invalidate(spr_canvas);
}

static void face_timer_cb(lv_timer_t *t) {
    uint32_t now = millis();
    int st = ai_state;
    bool talking = (st == AI_SPEAKING) || (st == AI_DONE && now < talk_until);
    spr_draw(now, st, talking);
    lv_obj_align(spr_canvas, LV_ALIGN_CENTER, 0, 40);
}

void build_ai_ui(lv_obj_t *parent) {
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    // Personagem
    spr_canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(spr_canvas, spr_buf, SPR_W, SPR_H, LV_IMG_CF_TRUE_COLOR);
    lv_img_set_antialias(spr_canvas, false);
    lv_img_set_zoom(spr_canvas, 1600);
    lv_obj_align(spr_canvas, LV_ALIGN_CENTER, 0, 40);
    no_click(spr_canvas);

    // Balão de fala
    lv_obj_t *box = ai_box = lv_obj_create(parent);
    lv_obj_set_size(box, 320, 130);
    lv_obj_align(box, LV_ALIGN_BOTTOM_MID, 0, -45);
    lv_obj_set_style_bg_opa(box, LV_OPA_90, 0);
    lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x101014), 0);
    lv_obj_set_style_border_color(box, lv_color_hex(0x2A2A33), 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_radius(box, 24, 0);
    lv_obj_set_style_pad_all(box, 12, 0);
    no_click(box);

    lbl_ai_text = lv_label_create(box);
    lv_obj_set_width(lbl_ai_text, 290);
    lv_label_set_long_mode(lbl_ai_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(lbl_ai_text, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_ai_text, lv_color_white(), 0);
    lv_obj_set_style_text_align(lbl_ai_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_ai_text, ai_text);
    lv_obj_align(lbl_ai_text, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(parent, ai_btn_cb, LV_EVENT_ALL, NULL);

    lbl_ai_status = lv_label_create(parent);
    lv_obj_set_style_text_font(lbl_ai_status, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl_ai_status, lv_color_hex(0xA0A0A8), 0);
    lv_label_set_text(lbl_ai_status, "Segure a tela e fale\nArraste para cima para sair");
    lv_obj_set_style_text_align(lbl_ai_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_ai_status, LV_ALIGN_TOP_MID, 0, 34);

    lv_timer_create(ai_timer_cb, 100, NULL);
    lv_timer_create(face_timer_cb, 40, NULL);
}

static lv_obj_t *scr_clock, *scr_launcher, *scr_level, *scr_weather, *scr_ai;

static void nav_to(lv_obj_t *target, lv_scr_load_anim_t anim) {
    if (target && lv_scr_act() != target) lv_scr_load_anim(target, anim, 250, 0, false);
}

static void nav_gesture_cb(lv_event_t *e) {
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_dir_t dir = lv_indev_get_gesture_dir(indev);
    last_gesture_ms = millis();
    lv_obj_t *cur = lv_scr_act();

    if (cur == scr_launcher) {
        if (dir == LV_DIR_BOTTOM) nav_to(scr_clock, LV_SCR_LOAD_ANIM_MOVE_BOTTOM);
    } else if (dir == LV_DIR_TOP) {
        nav_to(scr_launcher, LV_SCR_LOAD_ANIM_MOVE_TOP);
    }
}

static void app_icon_cb(lv_event_t *e) {
    if (millis() - last_gesture_ms < 500) return;
    nav_to((lv_obj_t *)lv_event_get_user_data(e), LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

// Cria o botão circular de um app; devolve-o para o desenho do ícone
static lv_obj_t *make_app_icon(lv_obj_t *parent, int dx, int dy, const char *name, uint32_t color, lv_obj_t *target) {
    lv_obj_t *btn = lv_obj_create(parent);
    lv_obj_set_size(btn, 120, 120);
    lv_obj_align(btn, LV_ALIGN_CENTER, dx, dy);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x15151A), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 3, 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A33), LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, app_icon_cb, LV_EVENT_CLICKED, target);

    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_label_set_text(lbl, name);
    lv_obj_align(lbl, LV_ALIGN_CENTER, dx, dy + 82);
    return btn;
}

void build_launcher_ui(lv_obj_t *parent) {
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(parent, nav_gesture_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *title = lv_label_create(parent);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_label_set_text(title, "Aplicativos");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

    // Relógio: mostrador com dois ponteiros (volta à tela do relógio)
    lv_obj_t *b1 = make_app_icon(parent, -85, -80, "Relogio", 0xFFFFFF, scr_clock);
    static lv_point_t p_h[2] = {{30, 30}, {30, 12}}, p_m[2] = {{30, 30}, {46, 36}};
    static lv_style_t st_hand;
    lv_style_init(&st_hand);
    lv_style_set_line_width(&st_hand, 5);
    lv_style_set_line_color(&st_hand, lv_color_white());
    lv_style_set_line_rounded(&st_hand, true);
    lv_obj_t *face = lv_obj_create(b1);
    lv_obj_set_size(face, 64, 64);
    lv_obj_center(face);
    lv_obj_set_style_radius(face, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(face, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(face, 4, 0);
    lv_obj_set_style_border_color(face, lv_color_white(), 0);
    no_click(face);
    lv_obj_t *lh = lv_line_create(face);
    lv_line_set_points(lh, p_h, 2);
    lv_obj_add_style(lh, &st_hand, 0);
    lv_obj_set_pos(lh, -2, -2);
    lv_obj_t *lm = lv_line_create(face);
    lv_line_set_points(lm, p_m, 2);
    lv_obj_add_style(lm, &st_hand, 0);
    lv_obj_set_pos(lm, -2, -2);
    no_click(lh);
    no_click(lm);

    // Nível: anel com bolha
    lv_obj_t *b2 = make_app_icon(parent, 85, -80, "Nivel", 0x00E060, scr_level);
    lv_obj_t *ring = lv_obj_create(b2);
    lv_obj_set_size(ring, 64, 64);
    lv_obj_center(ring);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ring, 4, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0x808080), 0);
    no_click(ring);
    lv_obj_t *dot = lv_obj_create(ring);
    lv_obj_set_size(dot, 24, 24);
    lv_obj_center(dot);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(0x00E060), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    no_click(dot);

    // Clima: três arcos concêntricos (umidade, UV, pressão)
    lv_obj_t *b3 = make_app_icon(parent, -85, 90, "Clima", COL_HUM, scr_weather);
    const uint32_t cols[3] = {COL_HUM, COL_UV, COL_PRES};
    for (int i = 0; i < 3; i++) {
        lv_obj_t *arc = lv_arc_create(b3);
        int sz = 78 - i * 22;
        lv_obj_set_size(arc, sz, sz);
        lv_obj_center(arc);
        lv_arc_set_rotation(arc, 135);
        lv_arc_set_bg_angles(arc, 0, 270);
        lv_arc_set_range(arc, 0, 100);
        lv_arc_set_value(arc, 70 - i * 15);
        lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
        lv_obj_set_style_arc_width(arc, 6, LV_PART_MAIN);
        lv_obj_set_style_arc_color(arc, lv_color_hex(0x2A2A33), LV_PART_MAIN);
        lv_obj_set_style_arc_width(arc, 6, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(arc, lv_color_hex(cols[i]), LV_PART_INDICATOR);
        no_click(arc);
    }

    // IA: microfone
    lv_obj_t *b4 = make_app_icon(parent, 85, 90, "IA", 0xFF4D8D, scr_ai);
    lv_obj_t *m = lv_obj_create(b4);
    lv_obj_set_size(m, 22, 38);
    lv_obj_align(m, LV_ALIGN_CENTER, 0, -6);
    lv_obj_set_style_radius(m, 11, 0);
    lv_obj_set_style_bg_color(m, lv_color_hex(0xFF4D8D), 0);
    lv_obj_set_style_bg_opa(m, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(m, 0, 0);
    no_click(m);
    lv_obj_t *st = lv_obj_create(b4);
    lv_obj_set_size(st, 36, 14);
    lv_obj_align(st, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_radius(st, 7, 0);
    lv_obj_set_style_bg_opa(st, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(st, 3, 0);
    lv_obj_set_style_border_color(st, lv_color_hex(0xFF4D8D), 0);
    no_click(st);
}

void setup_watchface_ui() {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    scr_clock = scr;
    scr_level = lv_obj_create(NULL);
    scr_weather = lv_obj_create(NULL);
    scr_ai = lv_obj_create(NULL);
    scr_launcher = lv_obj_create(NULL);
    lv_obj_add_event_cb(scr, nav_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_level, nav_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_weather, nav_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(scr_ai, nav_gesture_cb, LV_EVENT_GESTURE, NULL);
    build_ai_ui(scr_ai);
    build_level_ui(scr_level);
    build_weather_ui(scr_weather);
    build_launcher_ui(scr_launcher);

    // 1. Fundo com o logótipo Schroeder's Racing
    lv_obj_t *bg_img = lv_img_create(scr);
    lv_img_set_src(bg_img, &bg_racing);
    lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(bg_img, LV_OBJ_FLAG_CLICKABLE);

    // Marcadores: números em 12/3/6/9 e pontos nas demais horas
    for (int h = 1; h <= 12; h++) {
        float rad = (h * 30 - 90) * 0.0174532925f;
        bool is_num = (h % 3 == 0);
        int r = is_num ? 195 : 210;
        int x = CENTER_X + (int)(cosf(rad) * r);
        int y = CENTER_Y + (int)(sinf(rad) * r);

        if (is_num) {
            lv_obj_t *lbl = lv_label_create(scr);
            lv_label_set_text_fmt(lbl, "%d", h);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_40, 0);
            lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
            lv_obj_set_style_bg_color(lbl, lv_color_black(), 0);
            lv_obj_set_style_bg_opa(lbl, LV_OPA_70, 0);
            lv_obj_set_style_radius(lbl, 12, 0);
            lv_obj_set_style_pad_all(lbl, 4, 0);
            lv_obj_align(lbl, LV_ALIGN_CENTER, x - CENTER_X, y - CENTER_Y);
        } else {
            lv_obj_t *dot = lv_obj_create(scr);
            lv_obj_set_size(dot, 16, 16);
            lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(dot, lv_color_white(), 0);
            lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(dot, 3, 0);
            lv_obj_set_style_border_color(dot, lv_color_black(), 0);
            lv_obj_align(dot, LV_ALIGN_CENTER, x - CENTER_X, y - CENTER_Y);
        }
    }

    // 2. Ponteiro das Horas (Branco com ponta arredondada)
    static lv_style_t style_outline_hour, style_outline_min;
    lv_style_init(&style_outline_hour);
    lv_style_set_line_width(&style_outline_hour, 20);
    lv_style_set_line_color(&style_outline_hour, lv_color_black());
    lv_style_set_line_rounded(&style_outline_hour, true);
    lv_style_init(&style_outline_min);
    lv_style_set_line_width(&style_outline_min, 15);
    lv_style_set_line_color(&style_outline_min, lv_color_black());
    lv_style_set_line_rounded(&style_outline_min, true);

    outline_hour = lv_line_create(scr);
    lv_obj_add_style(outline_hour, &style_outline_hour, 0);
    outline_minute = lv_line_create(scr);
    lv_obj_add_style(outline_minute, &style_outline_min, 0);

    static lv_style_t style_hour;
    lv_style_init(&style_hour);
    lv_style_set_line_width(&style_hour, 12);
    lv_style_set_line_color(&style_hour, lv_color_hex(0xFFD400));
    lv_style_set_line_rounded(&style_hour, true);

    line_hour = lv_line_create(scr);
    lv_obj_add_style(line_hour, &style_hour, 0);

    // 3. Ponteiro dos Minutos (Prata / Cinza)
    static lv_style_t style_min;
    lv_style_init(&style_min);
    lv_style_set_line_width(&style_min, 7);
    lv_style_set_line_color(&style_min, lv_color_white());
    lv_style_set_line_rounded(&style_min, true);

    line_minute = lv_line_create(scr);
    lv_obj_add_style(line_minute, &style_min, 0);

    // 4. Ponteiro dos Segundos (Laranja vivo em sintonia com o logótipo)
    static lv_style_t style_sec;
    lv_style_init(&style_sec);
    lv_style_set_line_width(&style_sec, 2);
    lv_style_set_line_color(&style_sec, lv_color_hex(0xFF4500));
    lv_style_set_line_rounded(&style_sec, true);

    line_second = lv_line_create(scr);
    lv_obj_add_style(line_second, &style_sec, 0);

    // 5. Cubo do Eixo Central
    lv_obj_t *axis_hub = lv_obj_create(scr);
    lv_obj_set_size(axis_hub, 18, 18);
    lv_obj_align(axis_hub, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(axis_hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(axis_hub, lv_color_hex(0xFF4500), 0);
    lv_obj_set_style_border_width(axis_hub, 2, 0);
    lv_obj_set_style_border_color(axis_hub, lv_color_white(), 0);

    // Informações nos cantos
    lbl_c_date = make_corner_label(scr, LV_ALIGN_TOP_LEFT, 40, 40);
    lv_label_set_text(lbl_c_date, "--/--");
    lbl_c_year = make_corner_label(scr, LV_ALIGN_TOP_RIGHT, -40, 40);
    lv_label_set_text(lbl_c_year, "----");
    lbl_c_wday = make_corner_label(scr, LV_ALIGN_BOTTOM_LEFT, 40, -40);
    lv_label_set_text(lbl_c_wday, "---");
    lbl_c_temp = make_corner_label(scr, LV_ALIGN_BOTTOM_RIGHT, -40, -40);
    lv_label_set_text(lbl_c_temp, "--\xC2\xB0" "C");
    lv_timer_create(corner_info_timer_cb, 1000, NULL);
    corner_info_timer_cb(NULL);

    // Atualização de 1 em 1 segundo
    lv_timer_create(clock_timer_cb, 1000, NULL);
    clock_timer_cb(NULL);
}

// -----------------------------------------------------------------------------
// Sincronização Horária NTP (FreeRTOS)
// -----------------------------------------------------------------------------
static bool weather_ok = false;

void sync_ntp_time() {
    xSemaphoreTake(wifi_mtx, portMAX_DELAY);
    Serial.println("[NTP] A verificar sincronizacao com o servidor...");

    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("[NTP] A ligar a rede: %s\n", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 25) {
            vTaskDelay(pdMS_TO_TICKS(400));
            attempts++;
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[NTP] Wi-Fi ligado. A requisitar hora atualizada...");
        
        long gmt_sec = atol(String(GMT_OFFSET_SEC).c_str());
        int daylight = atoi(String(DAYLIGHT_OFFSET_SEC).c_str());

        configTime(gmt_sec, daylight, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);

        struct tm timeinfo;
        int retries = 0;
        while (!getLocalTime(&timeinfo) && retries < 15) {
            vTaskDelay(pdMS_TO_TICKS(500));
            retries++;
        }

        if (retries < 15) {
            Serial.printf("[NTP] Hora confirmada com sucesso: %02d:%02d:%02d\n",
                          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else {
            Serial.println("[NTP] Tempo limite de resposta excedido.");
        }

        weather_ok = fetch_weather();

        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Serial.println("[NTP] Modulo Wi-Fi colocado em modo de repouso.");
    } else {
        Serial.println("[NTP] Falha ao ligar a rede Wi-Fi. Nova tentativa no proximo ciclo.");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
    xSemaphoreGive(wifi_mtx);
}

void ntp_task(void *pvParameters) {
    sync_ntp_time(); // Sincronização inicial no arranque

    uint32_t interval = (uint32_t)strtoul(String(NTP_SYNC_INTERVAL_MS).c_str(), NULL, 10);
    if (interval == 0) interval = 3600000;

    while (true) {
        // Repete em 1 minuto se a previsão falhou
        vTaskDelay(pdMS_TO_TICKS(weather_ok ? interval : 60000));
        sync_ntp_time();
    }
}

// -----------------------------------------------------------------------------
// Inicialização Principal do Sistema
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("[BOOT] Relogio iniciado");

    // 1. Inicializar o núcleo do LVGL
    lv_init();

    // 2. Alocar buffer de desenho em RAM interna compatível com DMA
    disp_buf = (lv_color_t *)heap_caps_malloc(SCREEN_WIDTH * DRAW_BUF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!disp_buf) {
        Serial.println("[ERRO] Falha critica: Memoria interna insuficiente para alocar o buffer do ecrã!");
        while (1) { delay(1000); }
    }
    lv_disp_draw_buf_init(&draw_buf, disp_buf, NULL, SCREEN_WIDTH * DRAW_BUF_LINES);

    // 3. Inicializar a estrutura do driver de exibição
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.rounder_cb = lvgl_rounder_cb;
    disp_drv.draw_buf = &draw_buf;

    // 4. Inicializar o barramento e o circuito de hardware do AMOLED
    init_amoled_hardware(&disp_drv);

    // 5. Concluir o registo do display no motor LVGL
    lv_disp_drv_register(&disp_drv);
    init_touch();
    wifi_mtx = xSemaphoreCreateMutex();
    init_audio();
    xTaskCreatePinnedToCore(ai_task, "AI_Task", 20480, NULL, 1, &ai_task_handle, 0);

    // 6. Construir o mostrador do relógio
    setup_watchface_ui();

    // 7. Criar a tarefa do relógio NTP no Núcleo 0 (Core 0)
    xTaskCreatePinnedToCore(
        ntp_task,
        "NTP_Daemon",
        16384,
        NULL,
        1,
        NULL,
        0
    );
}

void loop() {
    lv_timer_handler();
    delay(5);
}