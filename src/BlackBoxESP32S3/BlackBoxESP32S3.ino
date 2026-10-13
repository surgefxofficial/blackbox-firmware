/*
  SurgeFX Black Box — ESP32-S3 + Waveshare ESP32-S3-Touch-LCD-3.5 (non-B)
  LVGL UI + Captive Portal + BTS7960 (forward-only) + Smooth Ramp
  Presets in NVS + local OTA via portal (/update)

  v1.1.0 changes (vs 1.0.0):
   - FIX: setting speed to 0 (arc or web) now stops the pump instead of resuming last speed
   - FIX: web plug toggles are queued to the plug task (no longer block UI/ramp ~9s)
   - FIX: on-screen PLUG buttons update when toggled from the web
   - Minimum pump speed floor (MIN_PUMP_PCT)
   - Task watchdog on the main loop
   - Cross-core flags are std::atomic (noInterrupts() is per-core and gave no protection)
   - /cmd takes form params instead of hand-parsed JSON; /state built with snprintf (no String heap churn)
   - Smart plug API format cached per plug after first success
   - AP max clients 4 -> 8
   - Pump GPIOs driven safe as the very first thing in setup()
   - Firmware upload page at http://192.168.3.1/update (needs an OTA partition scheme, see notes)
   - LVGL: lv_tick_set_cb, CLICKED-only event filters, sleep per lv_timer_handler()
   - Web UI: slider throttled, polling paused while dragging, slider tracks target not ramp
*/

#define FIRMWARE_VERSION "1.1.0"

#include <Arduino.h>
#include <atomic>
#include <Wire.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include "TCA9554.h"
#include <XPowersLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_task_wdt.h>

// ========= Logo asset =========
extern const lv_image_dsc_t black_box;  // from black_box.c

// ========= build options =========
#define USE_HW_SPI        0      // pass 2: set 1 to use hardware SPI via GPIO matrix (test on bench)
#define HW_SPI_HZ         40000000
#define DEBUG_HEAP        0      // 1 = print free heap every 10s

// ========= toggles / motion =========
#define EN_ACTIVE_HIGH   1
#define PWM_ACTIVE_HIGH  1

#define RAMP_UP_PCT_PER_S     50
#define RAMP_DOWN_PCT_PER_S   80
#define RAMP_TICK_MS          20
#define MIN_PUMP_PCT          10   // lowest speed the pump is allowed to run at (tune to your pump's stall point)

// Presets
#define MAX_PRESETS 8
#define MAIN_SCREEN_PRESETS 4      // how many fit in the left column of the main screen

// Watchdog
#define WDT_TIMEOUT_S 10

// ========= sizes / pins (Waveshare non-B) =========
#define LCD_W   480
#define LCD_H   320
#define SCLK_PIN     5
#define MOSI_PIN     1
#define DC_PIN       3
#define LCD_CS     (-1)
#define BL_PIN       6
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  7
#define I2C_HZ       400000

// BTS7960 / IBT-2 (forward-only on RPWM)
#define PIN_R_EN   41
#define PIN_L_EN   42
#define PIN_RPWM   10
#define PIN_LPWM   11

// PWM
#define PUMP_PWM_FREQ_HZ   20000   // above audible range (BTS7960 OK to 25 kHz); was 2000 in v1.0
#define PUMP_PWM_RES_BITS  8

// I2C addrs
#define TCA_ADDR 0x20
#define AXP_ADDR 0x34
#define FT_ADDR  0x38

// Touch orientation (LANDSCAPE)
#define TOUCH_SWAP_XY   1
#define TOUCH_INVERT_X  1
#define TOUCH_INVERT_Y  0

// ========= Wi-Fi / portal =========
static const char* AP_PASS = "blackbox123";
#define AP_CHANNEL      6
#define AP_MAX_CLIENTS  8
IPAddress apIP(192,168,3,1), apGW(192,168,3,1), apMASK(255,255,255,0);
static char AP_SSID[32] = "";   // Filled in setup

// Smart plugs (must be statically addressed on the Black Box network)
static const char* PLUG_IP[3] = { nullptr, "192.168.3.101", "192.168.3.102" };

WebServer http(80);
DNSServer dns;
Preferences prefs;

// ========= drivers =========
TCA9554 TCA(TCA_ADDR);
XPowersAXP2101 pmic;

#if USE_HW_SPI
Arduino_DataBus *bus = new Arduino_ESP32SPI(DC_PIN, LCD_CS, SCLK_PIN, MOSI_PIN, GFX_NOT_DEFINED, FSPI);
#else
Arduino_DataBus *bus = new Arduino_SWSPI(DC_PIN, LCD_CS, SCLK_PIN, MOSI_PIN, GFX_NOT_DEFINED);
#endif
Arduino_GFX *gfx = new Arduino_ST7796(bus, GFX_NOT_DEFINED /*RST via TCA*/, 3 /*landscape 180°*/, true /*IPS*/, 320, 480);

static lv_draw_buf_t *lvbuf1 = nullptr;

// ========= Presets =========
static int g_presets[MAX_PRESETS];
static int g_preset_count = 0;

// ========= app state (main loop / LVGL only) =========
static int  g_target_pct = 0;
static int  g_curr_pct   = 0;
static int  g_last_pct   = 50;
static bool g_pumpReq    = false;
static uint32_t g_lastRamp = 0;

// ========= cross-core state (plug task <-> main loop) =========
static std::atomic<bool> g_plug_on[3]   = { {false}, {false}, {false} };
static std::atomic<bool> g_plug_ui_dirty{false};

// ========= UI objects =========
static lv_obj_t *scr_main=nullptr, *scr_help=nullptr, *scr_menu=nullptr;
static lv_obj_t *arc=nullptr, *lblPct=nullptr, *imgLogo=nullptr;
static lv_obj_t *btnOn=nullptr, *btnOff=nullptr;
static lv_obj_t *btnP1=nullptr, *btnP2=nullptr, *lblWiFi=nullptr, *lblSupport=nullptr;
static lv_obj_t *btnHelpIcon=nullptr, *btnMenuIcon=nullptr;
static lv_obj_t *preset_container = nullptr;

static lv_style_t style_good, style_bad, style_dim, style_btn, style_btn_label, style_menu_btn;

// Forward decl
static void rebuild_menu_screen();
static void rebuild_main_presets();
static void refresh_presets_ui();

// ========= SSID gen =========
static void generate_ssid_suffix() {
  prefs.begin("wifi", false);
  String suffix = prefs.getString("suffix", "");
  if (suffix.length() != 4) {
    const char chars[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"; // no 0/O, 1/I ambiguity
    suffix = "";
    for (int i = 0; i < 4; i++) suffix += chars[random(0, sizeof(chars) - 1)];  // random() is HW RNG on ESP32
    prefs.putString("suffix", suffix);
    Serial.println("Generated new SSID suffix: " + suffix);
  }
  snprintf(AP_SSID, sizeof(AP_SSID), "SurgeFX-BlackBox-%s", suffix.c_str());
  prefs.end();
  Serial.printf("WiFi SSID: %s\n", AP_SSID);
}

// ========= Presets (NVS) =========
static bool preset_valid(int v){ return v >= MIN_PUMP_PCT && v <= 100; }
static void sort_presets() {
  for (int i = 1; i < g_preset_count; i++) {
    int v = g_presets[i], j = i - 1;
    while (j >= 0 && g_presets[j] > v) { g_presets[j+1] = g_presets[j]; j--; }
    g_presets[j+1] = v;
  }
}
static void save_presets_to_nvs() {
  prefs.begin("presets", false);
  prefs.putInt("count", g_preset_count);
  char key[4];
  for (int i = 0; i < g_preset_count; i++) { snprintf(key, sizeof key, "p%d", i); prefs.putInt(key, g_presets[i]); }
  prefs.end();
}
static void load_presets_from_nvs() {
  prefs.begin("presets", false);
  int n = constrain(prefs.getInt("count", 0), 0, MAX_PRESETS);
  g_preset_count = 0;
  char key[4];
  for (int i = 0; i < n; i++) {
    snprintf(key, sizeof key, "p%d", i);
    int v = prefs.getInt(key, 0);
    if (preset_valid(v)) g_presets[g_preset_count++] = v;   // drop anything corrupt / below the floor
  }
  prefs.end();
  if (g_preset_count == 0) {
    g_preset_count = 4;
    g_presets[0] = 25; g_presets[1] = 50; g_presets[2] = 75; g_presets[3] = 100;
    save_presets_to_nvs();
  } else sort_presets();
}
static bool add_preset(int value) {
  if (g_preset_count >= MAX_PRESETS || !preset_valid(value)) return false;
  for (int i = 0; i < g_preset_count; i++) if (g_presets[i] == value) return false;
  g_presets[g_preset_count++] = value;
  sort_presets(); save_presets_to_nvs(); return true;
}
static bool remove_preset(int index) {
  if (index < 0 || index >= g_preset_count) return false;
  for (int i = index; i < g_preset_count - 1; i++) g_presets[i] = g_presets[i + 1];
  g_preset_count--; save_presets_to_nvs(); return true;
}

// ========= backlight =========
static void backlight_init(uint8_t level){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BL_PIN, 5000, 8);
  ledcWrite(BL_PIN, level);
#else
  const int ch=0; ledcSetup(ch,5000,8); ledcAttachPin(BL_PIN,ch); ledcWrite(ch,level);
#endif
}

// ========= Motor =========
static inline void en_set(bool enable){
  uint8_t lev = EN_ACTIVE_HIGH ? (enable?HIGH:LOW) : (enable?LOW:HIGH);
  digitalWrite(PIN_R_EN, lev); digitalWrite(PIN_L_EN, lev);
}
static inline void other_leg_off(){ digitalWrite(PIN_LPWM, LOW); }
static inline uint8_t pct_to_duty(int pct){
  pct = constrain(pct, 0, 100);
  uint32_t d = (uint32_t)((pct * 255U + 50U) / 100U);
  if (d > 255) d = 255;
  return PWM_ACTIVE_HIGH ? (uint8_t)d : (uint8_t)(255 - d);
}
static inline void pump_pwm_write(uint8_t duty){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_RPWM, duty);
#else
  ledcWrite(1, duty);
#endif
}
static void motor_pins_safe_init(){
  // First thing in setup(): make sure the BTS7960 can't see floating inputs during boot.
  pinMode(PIN_R_EN, OUTPUT); pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_RPWM, OUTPUT); pinMode(PIN_LPWM, OUTPUT);
  en_set(false); other_leg_off();
  digitalWrite(PIN_RPWM, PWM_ACTIVE_HIGH ? LOW : HIGH);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_RPWM, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS);
#else
  ledcSetup(1, PUMP_PWM_FREQ_HZ, PUMP_PWM_RES_BITS); ledcAttachPin(PIN_RPWM, 1);
#endif
  pump_pwm_write(PWM_ACTIVE_HIGH ? 0 : 255);
}
static void motor_apply_from_current(){
  other_leg_off();
  const bool enabled = (g_curr_pct > 0) && g_pumpReq;
  en_set(enabled);
  pump_pwm_write(enabled ? pct_to_duty(g_curr_pct) : (PWM_ACTIVE_HIGH?0:255));
}
static void motor_stop_now(){            // hard stop, no ramp (OTA, faults)
  g_pumpReq = false; g_target_pct = 0; g_curr_pct = 0; motor_apply_from_current();
}
static void ramp_tick(){
  const uint32_t now = millis();
  if (now - g_lastRamp < RAMP_TICK_MS) return;
  g_lastRamp = now;
  static const int up_step   = max(1, (RAMP_UP_PCT_PER_S   * RAMP_TICK_MS + 999) / 1000);
  static const int down_step = max(1, (RAMP_DOWN_PCT_PER_S * RAMP_TICK_MS + 999) / 1000);
  if (!g_pumpReq) g_target_pct = 0;
  if (g_curr_pct < g_target_pct){
    g_curr_pct = min(g_curr_pct + up_step, g_target_pct); motor_apply_from_current();
  } else if (g_curr_pct > g_target_pct){
    g_curr_pct = max(g_curr_pct - down_step, g_target_pct); motor_apply_from_current();
  }
}
// 0 = stop. 1..MIN_PUMP_PCT-1 is floored to MIN_PUMP_PCT.
static void motor_set_target_pct(int pct){
  pct = constrain(pct, 0, 100);
  if (pct == 0){ g_pumpReq = false; g_target_pct = 0; return; }
  if (pct < MIN_PUMP_PCT) pct = MIN_PUMP_PCT;
  g_last_pct = pct; g_target_pct = pct; g_pumpReq = true;
}
static void motor_set_on(bool on){
  g_pumpReq = on;
  g_target_pct = on ? max(g_last_pct, MIN_PUMP_PCT) : 0;
}

// ========= power rails =========
static void pmic_on(){
  if (!pmic.begin(Wire, AXP_ADDR, I2C_SDA_PIN, I2C_SCL_PIN)) { Serial.println("PMIC not found"); return; }
  pmic.setDC3Voltage(3300);   pmic.enableDC3();
  pmic.setALDO1Voltage(3300); pmic.enableALDO1();
  pmic.setALDO2Voltage(3300); pmic.enableALDO2();
  pmic.setALDO3Voltage(3300); pmic.enableALDO3();
  pmic.setBLDO1Voltage(3300); pmic.enableBLDO1();
  pmic.setBLDO2Voltage(3300); pmic.enableBLDO2();
  delay(30);
}
static void tca_all_high(){ if(!TCA.begin()){ Serial.println("TCA9554 not found"); return; } for(uint8_t p=0;p<8;++p){TCA.pinMode1(p,OUTPUT); TCA.write1(p,1);} }
static void lcd_reset_pulse(){ TCA.write1(1,1); delay(5); TCA.write1(1,0); delay(20); TCA.write1(1,1); delay(120); }
static void panel_post_begin(){ bus->beginWrite(); bus->writeCommand(0x21); bus->endWrite(); delay(10); gfx->fillScreen(0x0000); }

// ========= FT6336 touch =========
static bool ft_read_one(uint16_t &x, uint16_t &y){
  Wire.beginTransmission(FT_ADDR); Wire.write(0x02);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom(FT_ADDR,(uint8_t)5)!=5) return false;
  uint8_t n=Wire.read(), xh=Wire.read(), xl=Wire.read(), yh=Wire.read(), yl=Wire.read();
  if ((n & 0x0F)==0) return false;
  x=((xh & 0x0F)<<8)|xl; y=((yh & 0x0F)<<8)|yl; return true;
}

// ========= LVGL flush =========
static void lvgl_flush_cb(lv_display_t *disp,const lv_area_t *area,uint8_t *px){
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)px,
                          area->x2-area->x1+1, area->y2-area->y1+1);
  lv_display_flush_ready(disp);
}

// ========= Async plug control (runs on core 0) =========
typedef struct { uint8_t idx; bool on; } PlugCmd;
static QueueHandle_t plugQ = nullptr;
static uint8_t g_plug_fmt[3] = {0,0,0};   // 0 = unknown, else (format index + 1) that worked last time

static bool http_try_local(const char* url){
  Serial.printf("HTTP: %s\n", url);
  HTTPClient hc; hc.setTimeout(2500); hc.setReuse(false);
  WiFiClient client;
  if(!hc.begin(client, url)) { Serial.println("HTTP begin failed"); return false; }
  int c = hc.GET();
  Serial.printf("HTTP code: %d\n", c);
  if (c <= 0) Serial.printf("GET error: %s\n", hc.errorToString(c).c_str());
  hc.end(); return (c == 200);
}
static const char* plug_url(char* buf, size_t n, const char* ip, uint8_t fmt, bool on){
  switch (fmt){
    case 0:  snprintf(buf, n, "http://%s/relay/0?turn=%s", ip, on ? "on" : "off"); break;        // Shelly Gen1
    case 1:  snprintf(buf, n, "http://%s/rpc/Switch.Set?id=0&on=%s", ip, on ? "true" : "false"); break; // Shelly Gen2
    default: snprintf(buf, n, "http://%s/cm?cmnd=Power%%20%s", ip, on ? "On" : "Off"); break;     // Tasmota
  }
  return buf;
}
static bool plug_set_http(uint8_t idx, bool on){
  const char* ip = PLUG_IP[idx];
  char url[96];
  uint8_t &known = g_plug_fmt[idx];
  if (known){
    if (http_try_local(plug_url(url, sizeof url, ip, known - 1, on))) return true;
    known = 0;   // plug changed? fall through and re-probe
  }
  for (uint8_t f = 0; f < 3; f++){
    if (http_try_local(plug_url(url, sizeof url, ip, f, on))){ known = f + 1; return true; }
    delay(100);
  }
  return false;
}
static void plugTask(void*){
  PlugCmd cmd;
  for(;;){
    if (xQueueReceive(plugQ, &cmd, portMAX_DELAY)==pdTRUE){
      if (cmd.idx < 1 || cmd.idx > 2) continue;
      if (plug_set_http(cmd.idx, cmd.on)){
        g_plug_on[cmd.idx].store(cmd.on);
        g_plug_ui_dirty.store(true);
      } else Serial.printf("Plug %u: no response\n", cmd.idx);
    }
  }
}
static void plug_enqueue(uint8_t idx, bool on){
  if (!plugQ) return; PlugCmd c={idx,on}; xQueueSend(plugQ, &c, 0);
}

// ========= Styles =========
static void apply_styles(){
  lv_style_init(&style_good); lv_style_set_bg_color(&style_good, lv_color_hex(0x1CBF7A));
  lv_style_init(&style_bad);  lv_style_set_bg_color(&style_bad,  lv_color_hex(0xF0474A));
  lv_style_init(&style_dim);  lv_style_set_bg_color(&style_dim,  lv_color_hex(0x5E6A64));
  lv_style_init(&style_btn);  lv_style_set_radius(&style_btn, 12); lv_style_set_border_width(&style_btn, 0); lv_style_set_pad_all(&style_btn, 0);
  lv_style_init(&style_btn_label); lv_style_set_text_color(&style_btn_label, lv_color_hex(0xFFFFFF)); lv_style_set_text_font(&style_btn_label, &lv_font_montserrat_18);
  lv_style_init(&style_menu_btn); lv_style_set_radius(&style_menu_btn, 8); lv_style_set_bg_color(&style_menu_btn, lv_color_hex(0x2A3138)); lv_style_set_border_width(&style_menu_btn, 0);
}
static void style_btn_on(lv_obj_t *b){ lv_obj_remove_style(b, &style_dim, 0); lv_obj_add_style(b, &style_good, 0); }
static void style_btn_off(lv_obj_t *b){ lv_obj_remove_style(b, &style_good, 0); lv_obj_add_style(b, &style_dim, 0); }

// Small helper: button with centered label
static lv_obj_t* make_btn(lv_obj_t* parent, const char* text, lv_style_t* bg, int w, int h, lv_event_cb_t cb, void* ud){
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_add_style(b, &style_btn, 0); lv_obj_add_style(b, bg, 0);
  lv_obj_set_size(b, w, h);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
  lv_obj_t* l = lv_label_create(b); lv_obj_add_style(l, &style_btn_label, 0); lv_label_set_text(l, text); lv_obj_center(l);
  return b;
}

// ========= UI callbacks (all CLICKED / VALUE_CHANGED only) =========
static void reflect_current_in_ui(){
  if (arc)    lv_arc_set_value(arc, g_curr_pct);
  if (lblPct) lv_label_set_text_fmt(lblPct, "%d%%", g_curr_pct);
}
static void arc_event(lv_event_t *e){ motor_set_target_pct(lv_arc_get_value((lv_obj_t*)lv_event_get_target(e))); }
static void on_preset_btn(lv_event_t *e){ motor_set_target_pct((int)(intptr_t)lv_event_get_user_data(e)); }
static void on_pump_on (lv_event_t*){ motor_set_on(true);  }
static void on_pump_off(lv_event_t*){ motor_set_on(false); }
static void on_plug1(lv_event_t*){ plug_enqueue(1, !g_plug_on[1].load()); }
static void on_plug2(lv_event_t*){ plug_enqueue(2, !g_plug_on[2].load()); }
static void on_help_btn (lv_event_t*){ lv_screen_load(scr_help); }
static void on_help_back(lv_event_t*){ lv_screen_load(scr_main); }
static void on_menu_btn (lv_event_t*){ rebuild_menu_screen(); lv_screen_load(scr_menu); }
static void on_menu_back(lv_event_t*){ rebuild_main_presets(); lv_screen_load(scr_main); }
static void on_del_preset(lv_event_t *e){
  if (remove_preset((int)(intptr_t)lv_event_get_user_data(e))) refresh_presets_ui();
}

// Add-preset dialog
static lv_obj_t *add_preset_dialog=nullptr,*add_preset_slider=nullptr,*add_preset_value_lbl=nullptr;
static void close_add_preset_dialog(lv_event_t*){
  if (add_preset_dialog){ lv_obj_delete_async(add_preset_dialog); add_preset_dialog=add_preset_slider=add_preset_value_lbl=nullptr; }
}
static void confirm_add_preset(lv_event_t *e){
  if (!add_preset_slider) return;
  if (add_preset(lv_slider_get_value(add_preset_slider))) refresh_presets_ui();
  close_add_preset_dialog(e);
}
static void on_add_slider_change(lv_event_t*){
  if(!add_preset_slider||!add_preset_value_lbl)return;
  lv_label_set_text_fmt(add_preset_value_lbl,"%d%%",(int)lv_slider_get_value(add_preset_slider));
}
static void on_menu_add_preset(lv_event_t*){
  if (add_preset_dialog) return;   // already open
  add_preset_dialog = lv_obj_create(lv_screen_active());
  lv_obj_set_size(add_preset_dialog, 320, 240); lv_obj_center(add_preset_dialog);
  lv_obj_set_style_bg_color(add_preset_dialog, lv_color_hex(0x1A1F25), 0);
  lv_obj_set_style_border_color(add_preset_dialog, lv_color_hex(0x74E3E0), 0);
  lv_obj_set_style_border_width(add_preset_dialog, 2, 0);
  lv_obj_set_style_radius(add_preset_dialog, 12, 0);
  lv_obj_remove_flag(add_preset_dialog, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *title = lv_label_create(add_preset_dialog); lv_label_set_text(title, "Add Preset");
  lv_obj_set_style_text_color(title, lv_color_hex(0x74E3E0), 0); lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);
  lv_obj_t *instr = lv_label_create(add_preset_dialog); lv_label_set_text(instr, "Select percentage:");
  lv_obj_set_style_text_color(instr, lv_color_hex(0xE7F0F0), 0); lv_obj_align(instr, LV_ALIGN_TOP_MID, 0, 50);
  add_preset_value_lbl = lv_label_create(add_preset_dialog); lv_label_set_text(add_preset_value_lbl, "50%");
  lv_obj_set_style_text_color(add_preset_value_lbl, lv_color_hex(0x74E3E0), 0);
  lv_obj_set_style_text_font(add_preset_value_lbl, &lv_font_montserrat_28, 0); lv_obj_align(add_preset_value_lbl, LV_ALIGN_CENTER, 0, -10);
  add_preset_slider = lv_slider_create(add_preset_dialog); lv_slider_set_range(add_preset_slider, MIN_PUMP_PCT, 100); lv_slider_set_value(add_preset_slider, 50, LV_ANIM_OFF);
  lv_obj_set_width(add_preset_slider, 260); lv_obj_align(add_preset_slider, LV_ALIGN_CENTER, 0, 30);
  lv_obj_set_style_bg_color(add_preset_slider, lv_color_hex(0x35434A), LV_PART_MAIN);
  lv_obj_set_style_bg_color(add_preset_slider, lv_color_hex(0x74E3E0), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(add_preset_slider, lv_color_hex(0x74E3E0), LV_PART_KNOB);
  lv_obj_add_event_cb(add_preset_slider, on_add_slider_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_t *add_btn    = make_btn(add_preset_dialog, "ADD",    &style_good, 120, 45, confirm_add_preset, NULL);
  lv_obj_align(add_btn, LV_ALIGN_BOTTOM_LEFT, 15, -15);
  lv_obj_t *cancel_btn = make_btn(add_preset_dialog, "CANCEL", &style_dim,  120, 45, close_add_preset_dialog, NULL);
  lv_obj_align(cancel_btn, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
}

static void rebuild_menu_screen(){
  if (!scr_menu) { scr_menu = lv_obj_create(NULL); lv_obj_set_style_bg_color(scr_menu, lv_color_hex(0x111417), 0); }
  lv_obj_clean(scr_menu);
  add_preset_dialog = add_preset_slider = add_preset_value_lbl = nullptr;  // cleaned with the screen

  lv_obj_t *title = lv_label_create(scr_menu);
  lv_label_set_text(title, "PRESET SETTINGS");
  lv_obj_set_style_text_color(title, lv_color_hex(0x74E3E0), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *list = lv_obj_create(scr_menu);
  lv_obj_set_size(list, 380, 160);
  lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 50);
  lv_obj_set_style_bg_color(list, lv_color_hex(0x1A1F25), 0);
  lv_obj_set_style_border_width(list, 0, 0);
  lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(list, 10, 0);
  lv_obj_set_style_pad_gap(list, 8, 0);
  lv_obj_set_scroll_dir(list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

  for (int i = 0; i < g_preset_count; i++) {
    lv_obj_t *item = lv_obj_create(list);
    lv_obj_set_size(item, 360, 40);
    lv_obj_set_style_bg_color(item, lv_color_hex(0x2A3138), 0);
    lv_obj_set_style_border_width(item, 0, 0);
    lv_obj_set_style_radius(item, 8, 0);
    lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *lbl = lv_label_create(item);
    lv_label_set_text_fmt(lbl, "%d%%", g_presets[i]);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_t *del_btn = make_btn(item, "DEL", &style_bad, 60, 30, on_del_preset, (void*)(intptr_t)i);
    lv_obj_align(del_btn, LV_ALIGN_RIGHT_MID, -5, 0);
  }

  lv_obj_t *add_btn  = make_btn(scr_menu, "ADD PRESET", &style_good, 180, 50, on_menu_add_preset, NULL);
  lv_obj_align(add_btn, LV_ALIGN_BOTTOM_MID, -100, -10);
  lv_obj_t *back_btn = make_btn(scr_menu, "BACK", &style_dim, 180, 50, on_menu_back, NULL);
  lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 100, -10);
}

static void rebuild_main_presets(){
  if (!preset_container) return;
  lv_obj_clean(preset_container);
  for (int i = 0; i < g_preset_count && i < MAIN_SCREEN_PRESETS; i++) {
    char txt[8]; snprintf(txt, sizeof txt, "%d%%", g_presets[i]);
    make_btn(preset_container, txt, &style_dim, 118, 44, on_preset_btn, (void*)(intptr_t)g_presets[i]);
  }
}
// Called after any preset change (touch or web): refresh whichever screens exist.
static void refresh_presets_ui(){
  rebuild_main_presets();
  if (scr_menu && lv_screen_active() == scr_menu) rebuild_menu_screen();
}

static void build_screen_help(){
  scr_help = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_help, lv_color_hex(0x111417), 0);
  lv_obj_t* btnBack = lv_button_create(scr_help);
  lv_obj_set_size(btnBack, 28, 28); lv_obj_set_style_radius(btnBack, 14, 0);
  lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x35434A), 0);
  lv_obj_align(btnBack, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_obj_add_event_cb(btnBack, on_help_back, LV_EVENT_CLICKED, NULL);
  lv_obj_t* t = lv_label_create(btnBack); lv_obj_set_style_text_color(t, lv_color_hex(0xFFFFFF), 0); lv_label_set_text(t, LV_SYMBOL_LEFT); lv_obj_center(t);
  lv_obj_t* title = lv_label_create(scr_help);
  lv_obj_set_style_text_color(title, lv_color_hex(0x74E3E0), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
  lv_label_set_text(title, "Help & Support");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_t* txt = lv_label_create(scr_help);
  lv_label_set_long_mode(txt, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(txt, 460);
  lv_obj_align(txt, LV_ALIGN_TOP_LEFT, 10, 45);
  lv_obj_set_style_text_color(txt, lv_color_hex(0xE7F0F0), 0);
  char help[512];
  snprintf(help, sizeof(help),
    "CONNECTING:\n"
    "1. Power on Black Box\n"
    "2. Join WiFi network shown\n"
    "   at bottom of main screen\n"
    "   Password: %s\n"
    "3. Open: http://192.168.3.1\n\n"
    "CONTROLS:\n"
    "Arc Dial = speed (0 = stop)\nPresets = quick set\nPlug 1/2 control\n\n"
    "SUPPORT:\n(866) 577-8743\ninfo@surgefx.com\n\n"
    "Firmware: v%s   Update: http://192.168.3.1/update", AP_PASS, FIRMWARE_VERSION);
  lv_label_set_text(txt, help);
}

static void build_screen_main(){
  scr_main = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_main, lv_color_hex(0x111417), 0);

  btnMenuIcon = lv_button_create(scr_main);
  lv_obj_set_size(btnMenuIcon, 28, 28); lv_obj_set_style_radius(btnMenuIcon, 14, 0);
  lv_obj_set_style_bg_color(btnMenuIcon, lv_color_hex(0x35434A), 0);
  lv_obj_align(btnMenuIcon, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_obj_add_event_cb(btnMenuIcon, on_menu_btn, LV_EVENT_CLICKED, NULL);
  { lv_obj_t* s = lv_label_create(btnMenuIcon); lv_obj_set_style_text_color(s, lv_color_hex(0xFFFFFF), 0); lv_label_set_text(s, LV_SYMBOL_SETTINGS); lv_obj_center(s); }

  btnHelpIcon = lv_button_create(scr_main);
  lv_obj_set_size(btnHelpIcon, 28, 28); lv_obj_set_style_radius(btnHelpIcon, 14, 0);
  lv_obj_set_style_bg_color(btnHelpIcon, lv_color_hex(0x35434A), 0);
  lv_obj_align(btnHelpIcon, LV_ALIGN_TOP_RIGHT, -8, 8);
  lv_obj_add_event_cb(btnHelpIcon, on_help_btn, LV_EVENT_CLICKED, NULL);
  { lv_obj_t* q = lv_label_create(btnHelpIcon); lv_obj_set_style_text_color(q, lv_color_hex(0xFFFFFF), 0); lv_label_set_text(q, "?"); lv_obj_center(q); }

  arc = lv_arc_create(scr_main);
  lv_obj_set_size(arc, 205, 205);
  lv_obj_align(arc, LV_ALIGN_CENTER, 0, -4);
  lv_arc_set_rotation(arc, 135);
  lv_arc_set_bg_angles(arc, 0, 270);
  lv_arc_set_range(arc, 0, 100);
  lv_obj_add_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(arc, 18, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, 18, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x35434A), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x74E3E0), LV_PART_INDICATOR);
  lv_obj_set_style_pad_all(arc, 12, LV_PART_KNOB);
  lv_obj_set_style_bg_color(arc, lv_color_hex(0x74E3E0), LV_PART_KNOB);
  lv_obj_add_event_cb(arc, arc_event, LV_EVENT_VALUE_CHANGED, NULL);

  imgLogo = lv_image_create(scr_main); lv_image_set_src(imgLogo, &black_box);
  lv_obj_align(imgLogo, LV_ALIGN_CENTER, 0, -10); lv_obj_add_flag(imgLogo, LV_OBJ_FLAG_ADV_HITTEST);

  lblPct = lv_label_create(scr_main); lv_label_set_text(lblPct,"0%");
  lv_obj_set_style_text_color(lblPct, lv_color_hex(0x74E3E0), 0);
  lv_obj_align(lblPct, LV_ALIGN_CENTER, 0, 76);

  preset_container = lv_obj_create(scr_main);
  lv_obj_set_size(preset_container, 118, 200);
  lv_obj_align(preset_container, LV_ALIGN_LEFT_MID, 10, 10);
  lv_obj_set_style_bg_opa(preset_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(preset_container, 0, 0);
  lv_obj_set_style_pad_all(preset_container, 0, 0);
  lv_obj_set_flex_flow(preset_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(preset_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(preset_container, 4, 0);
  rebuild_main_presets();

  btnOn  = make_btn(scr_main, "PUMP ON",  &style_good, 118, 44, on_pump_on,  NULL); lv_obj_align(btnOn,  LV_ALIGN_RIGHT_MID, -10, -62);
  btnOff = make_btn(scr_main, "PUMP OFF", &style_bad,  118, 44, on_pump_off, NULL); lv_obj_align(btnOff, LV_ALIGN_RIGHT_MID, -10, -14);
  btnP1  = make_btn(scr_main, "PLUG 1",   &style_dim,  118, 44, on_plug1,    NULL); lv_obj_align(btnP1,  LV_ALIGN_RIGHT_MID, -10,  34);
  btnP2  = make_btn(scr_main, "PLUG 2",   &style_dim,  118, 44, on_plug2,    NULL); lv_obj_align(btnP2,  LV_ALIGN_RIGHT_MID, -10,  82);

  lblWiFi = lv_label_create(scr_main);
  lv_obj_set_style_text_color(lblWiFi, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(lblWiFi, LV_ALIGN_BOTTOM_MID, 0, -8);
  lv_label_set_text(lblWiFi, "Support: (866) 577-8743   •   info@surgefx.com   •   surgefx.com");

  lblSupport = lv_label_create(scr_main);
  lv_obj_set_style_text_color(lblSupport, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(lblSupport, LV_ALIGN_BOTTOM_MID, 0, -26);
  lv_label_set_text(lblSupport, "WiFi AP: (starting...)");
}

static void update_plug_ui(){
  if (btnP1){ if (g_plug_on[1].load()) style_btn_on(btnP1); else style_btn_off(btnP1); }
  if (btnP2){ if (g_plug_on[2].load()) style_btn_on(btnP2); else style_btn_off(btnP2); }
}

// ========= touch read → map =========
static inline void map_touch(int16_t rx,int16_t ry,int16_t &mx,int16_t &my){
#if TOUCH_SWAP_XY
  int16_t tx = ry, ty = rx;
#else
  int16_t tx = rx, ty = ry;
#endif
#if TOUCH_INVERT_X
  tx = (LCD_W - 1) - tx;
#endif
#if TOUCH_INVERT_Y
  ty = (LCD_H - 1) - ty;
#endif
  mx = constrain(tx, 0, LCD_W - 1);
  my = constrain(ty, 0, LCD_H - 1);
}
static void lv_touch_read_cb(lv_indev_t*, lv_indev_data_t *d){
  static int16_t lx=LCD_W/2, ly=LCD_H/2;
  uint16_t rx, ry;
  if (ft_read_one(rx,ry)){ int16_t mx,my; map_touch(rx,ry,mx,my); lx=mx; ly=my; d->state=LV_INDEV_STATE_PRESSED; }
  else d->state=LV_INDEV_STATE_RELEASED;
  d->point.x = lx; d->point.y = ly;
}

// ========= Captive portal UI (HTML) =========
static const char index_html[] PROGMEM = R"HTML(
<!doctype html><html lang='en'><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>SurgeFX Black Box</title>
<style>:root{--accent:#74E3E0;--txt:#e7f0f0;--sub:#98a4ab;--muted:#2a3138;--good:#26c281;--bad:#ff4d4f}
*,*:before,*:after{box-sizing:border-box}body{margin:0;background:#111417;color:var(--txt);font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,Ubuntu}
.wrap{max-width:720px;margin:0 auto;padding:18px}.header{position:relative;text-align:center;margin-bottom:16px}
.settings-btn{position:absolute;left:0;top:0;width:40px;height:40px;background:#35434A;border:0;border-radius:50%;color:#fff;font-size:20px;cursor:pointer}
h1{margin:0;color:var(--accent);font-size:24px}.g{display:grid;place-items:center}.gw{width:260px;height:260px;position:relative}
.arc{position:absolute;inset:0;border-radius:50%;background:conic-gradient(from -135deg,var(--accent) calc(var(--p)*0.75%),#2b343b 0 75%,#0000 0 100%);-webkit-mask:radial-gradient(farthest-side,#0000 calc(100% - 18px),#000 0)}
.pct{position:absolute;left:50%;top:50%;transform:translate(-50%,58%);font-size:28px;color:var(--accent);font-weight:800}
input[type=range]{width:100%;appearance:none;height:12px;border-radius:8px;background:#23313a;outline:none}
input[type=range]::-webkit-slider-thumb{appearance:none;width:22px;height:22px;border-radius:50%;background:var(--accent);box-shadow:0 0 0 6px rgba(116,227,224,.25)}
.btn{height:52px;border:0;border-radius:12px;color:#fff;font-weight:800;letter-spacing:.3px;cursor:pointer}
.ok{background:#1cbf7a}.bad{background:#f0474a}.dim{background:#5e6a64}.grid{display:grid;gap:12px}.two{grid-template-columns:1fr 1fr}
.preset-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(80px,1fr));gap:12px;margin-top:12px}.stack{display:grid;gap:12px;margin-top:12px}
.btn.block{width:100%;height:56px}.footer{margin:14px 0 6px;color:var(--sub);text-align:center;display:flex;gap:16px;justify-content:center;flex-wrap:wrap}
a{color:#a8f1ee;text-decoration:none}.page{display:none}.page.active{display:block}.preset-manage{padding:20px;background:#1a1f25;border-radius:12px}
.preset-manage h2{margin:0 0 16px;color:var(--accent);text-align:center}.preset-item{display:flex;justify-content:space-between;align-items:center;padding:12px;background:#2a3138;border-radius:8px;margin-bottom:8px}
.add-preset{display:flex;gap:8px;margin-top:16px}.add-preset input{flex:1;padding:12px;background:#2a3138;border:1px solid #3a4148;border-radius:8px;color:var(--txt);font-size:16px}
.back-btn{margin-top:20px}.off{opacity:.45}
</style></head><body>
<div id='mainPage' class='page active'><div class='wrap'>
  <div class='header'>
    <button class='settings-btn' onclick='showSettings()'>&#9881;</button>
    <h1>SurgeFX Black Box</h1>
  </div>
  <div class='g'><div class='gw'><div id='arc' class='arc' style='--p:0'></div><div id='pct' class='pct'>0%</div></div></div>
  <input id='range' type='range' min='0' max='100' step='1' value='0'>
  <div class='grid two' style='margin-top:12px'>
    <button id='on' class='btn ok'>PUMP ON</button>
    <button id='off' class='btn bad'>PUMP OFF</button>
  </div>
  <div id='presets' class='preset-grid'></div>
  <div class='stack'>
    <button id='p1' class='btn dim block'>PLUG 1</button>
    <button id='p2' class='btn dim block'>PLUG 2</button>
  </div>
  <div class='footer'>
    <a href='tel:+18665778743'>Call (866) 577-8743</a>
    <a href='mailto:info@surgefx.com'>info@surgefx.com</a>
    <a href='https://surgefx.com' target='_blank'>Order Product</a>
    <a href='/update' id='fw'>Firmware</a>
  </div>
</div></div>
<div id='settingsPage' class='page'><div class='wrap'>
  <div class='preset-manage'>
    <h2>Manage Presets</h2>
    <div id='preset-list'></div>
    <div class='add-preset'>
      <input id='new-preset' type='number' min='1' max='100' placeholder='Enter speed %'>
      <button id='add-preset-btn' class='btn ok' style='width:100px;height:48px'>ADD</button>
    </div>
    <button class='btn dim block back-btn' onclick='showMain()'>BACK</button>
  </div>
</div></div>
<script>
let st={speed:0,target:0,pump:false,p1:false,p2:false,presets:[],fw:''};
let dragging=false,sendTimer=null,pending=null;
const $=id=>document.getElementById(id);
const arc=$('arc'),pct=$('pct'),rng=$('range'),onb=$('on'),offb=$('off'),p1=$('p1'),p2=$('p2');
const presetsCont=$('presets'),presetList=$('preset-list'),newPreset=$('new-preset'),addBtn=$('add-preset-btn');
function showSettings(){$('mainPage').classList.remove('active');$('settingsPage').classList.add('active');}
function showMain(){$('settingsPage').classList.remove('active');$('mainPage').classList.add('active');}
function paint(){
  arc.style.setProperty('--p',st.speed);pct.textContent=st.speed+'%';
  if(!dragging)rng.value=st.target;
  onb.classList.toggle('off',!st.pump);offb.classList.toggle('off',st.pump);
  p1.className='btn block '+(st.p1?'ok':'dim');p2.className='btn block '+(st.p2?'ok':'dim');
  $('fw').textContent='Firmware v'+st.fw;
  presetsCont.innerHTML='';st.presets.forEach(p=>{const b=document.createElement('button');b.className='btn '+(st.pump&&st.target==p?'ok':'dim');b.textContent=p+'%';b.style.height='44px';b.onclick=()=>send({S:p});presetsCont.appendChild(b);});
  presetList.innerHTML='';st.presets.forEach((p,i)=>{const d=document.createElement('div');d.className='preset-item';d.innerHTML='<span style="font-size:18px">'+p+'%</span><button class="btn bad" style="width:70px;height:36px" onclick="removePreset('+i+')">DEL</button>';presetList.appendChild(d);});
}
async function fetchS(){if(dragging)return;try{const r=await fetch('/state',{cache:'no-store'});if(!r.ok)throw 0;st=await r.json();paint();}catch(e){}}
async function send(c){try{const r=await fetch('/cmd',{method:'POST',body:new URLSearchParams(c)});if(r.ok){st=await r.json();paint();}}catch(e){}}
function removePreset(i){send({DP:i});}
function addPreset(){const v=parseInt(newPreset.value);if(v>=1&&v<=100){send({AP:v});newPreset.value='';}}
/* slider: live local feedback, throttled sends while dragging, final send on release */
rng.oninput=e=>{dragging=true;pending=+e.target.value;pct.textContent=pending+'%';
  if(!sendTimer)sendTimer=setTimeout(()=>{sendTimer=null;if(pending!=null)send({S:pending});},120);};
rng.onchange=e=>{dragging=false;pending=null;send({S:+e.target.value});};
onb.onclick=()=>send({M:1}); offb.onclick=()=>send({M:0});
p1.onclick=()=>send({P1:st.p1?0:1}); p2.onclick=()=>send({P2:st.p2?0:1});
addBtn.onclick=addPreset; newPreset.onkeypress=e=>{if(e.key==='Enter')addPreset();};
setInterval(fetchS,700); fetchS();
</script></body></html>
)HTML";

static const char update_html[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Black Box Firmware</title>
<style>body{margin:0;background:#111417;color:#e7f0f0;font:15px/1.5 system-ui}.wrap{max-width:520px;margin:0 auto;padding:24px}
h1{color:#74E3E0;font-size:22px}input[type=file]{display:block;margin:16px 0;color:#e7f0f0}
button{height:50px;width:100%;border:0;border-radius:12px;background:#1cbf7a;color:#fff;font-weight:800;font-size:16px}
#bar{height:10px;background:#23313a;border-radius:6px;margin-top:16px;overflow:hidden}#fill{height:100%;width:0;background:#74E3E0}
#msg{margin-top:12px;color:#98a4ab}a{color:#a8f1ee}</style></head><body><div class='wrap'>
<h1>Firmware Update</h1><p>Current: v%FW%</p>
<p>Select the <b>BlackBoxESP32S3.ino.bin</b> file and press Upload. The pump will stop and the device reboots when done. Do not power off.</p>
<form id='f'><input type='file' name='firmware' id='file' accept='.bin' required><button type='submit'>UPLOAD</button></form>
<div id='bar'><div id='fill'></div></div><div id='msg'></div><p><a href='/'>&larr; Back</a></p></div>
<script>
document.getElementById('f').onsubmit=e=>{e.preventDefault();const fd=new FormData();fd.append('firmware',document.getElementById('file').files[0]);
const x=new XMLHttpRequest();x.open('POST','/update');x.upload.onprogress=ev=>{if(ev.lengthComputable)document.getElementById('fill').style.width=(ev.loaded/ev.total*100)+'%';};
x.onload=()=>{document.getElementById('msg').textContent=x.responseText;if(x.responseText.startsWith('OK'))setTimeout(()=>location.href='/',8000);};
x.onerror=()=>{document.getElementById('msg').textContent='Upload failed (connection). If the device rebooted, the update likely succeeded.';};
document.getElementById('msg').textContent='Uploading...';x.send(fd);};
</script></body></html>
)HTML";

// ========= HTTP handlers =========
static void send_index(){ http.send_P(200,"text/html",index_html); }
static void send_state(){
  char buf[320];
  int n = snprintf(buf, sizeof buf,
    "{\"speed\":%d,\"target\":%d,\"pump\":%s,\"p1\":%s,\"p2\":%s,\"presets\":[",
    g_curr_pct, g_pumpReq ? g_target_pct : 0, g_pumpReq?"true":"false",
    g_plug_on[1].load()?"true":"false", g_plug_on[2].load()?"true":"false");
  for (int i = 0; i < g_preset_count && n < (int)sizeof buf - 24; i++)
    n += snprintf(buf + n, sizeof buf - n, "%s%d", i ? "," : "", g_presets[i]);
  n += snprintf(buf + n, sizeof buf - n, "],\"fw\":\"%s\"}", FIRMWARE_VERSION);
  http.send(200, "application/json", buf);
}
// POST /cmd with form fields: S=<pct> M=<0|1> P1=<0|1> P2=<0|1> AP=<pct> DP=<index>
static void handle_cmd(){
  if (http.hasArg("S"))  motor_set_target_pct(http.arg("S").toInt());
  if (http.hasArg("M"))  motor_set_on(http.arg("M").toInt() != 0);
  if (http.hasArg("P1")) plug_enqueue(1, http.arg("P1").toInt() != 0);
  if (http.hasArg("P2")) plug_enqueue(2, http.arg("P2").toInt() != 0);
  if (http.hasArg("AP")) { if (add_preset(http.arg("AP").toInt()))    refresh_presets_ui(); }
  if (http.hasArg("DP")) { if (remove_preset(http.arg("DP").toInt())) refresh_presets_ui(); }
  send_state();   // reply with fresh state so the page doesn't need a second round-trip
}
static void send_update_page(){
  String page = FPSTR(update_html);
  page.replace("%FW%", FIRMWARE_VERSION);
  http.send(200, "text/html", page);
}
static void handle_update_done(){
  const bool ok = !Update.hasError() && Update.isFinished();
  http.sendHeader("Connection", "close");
  String msg = ok ? String("OK - update complete, rebooting") : String("FAILED: ") + Update.errorString();
  http.send(200, "text/plain", msg);
  if (ok) { delay(500); ESP.restart(); }
}
static void handle_update_upload(){
  HTTPUpload& up = http.upload();
  if (up.status == UPLOAD_FILE_START) {
    motor_stop_now();                                   // never flash with the pump running
    Serial.printf("OTA start: %s\n", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);   // fails cleanly if no OTA partition
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!Update.hasError() && Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    esp_task_wdt_reset();
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("OTA OK: %u bytes\n", up.totalSize);
    else Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); Serial.println("OTA aborted");
  }
}

static void start_portal(){
  Serial.println("Starting WiFi AP...");
  WiFi.mode(WIFI_AP); WiFi.persistent(false); WiFi.setSleep(false);
  WiFi.softAPConfig(apIP, apGW, apMASK);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, false, AP_MAX_CLIENTS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  if (MDNS.begin("blackbox")) { MDNS.addService("http","tcp",80); Serial.println("mDNS: blackbox.local"); }
  dns.start(53, "*", apIP);
  http.on("/",       HTTP_GET,  send_index);
  http.on("/state",  HTTP_GET,  send_state);
  http.on("/cmd",    HTTP_POST, handle_cmd);
  http.on("/cmd",    HTTP_GET,  handle_cmd);          // handy for curl / integrations: /cmd?S=50
  http.on("/update", HTTP_GET,  send_update_page);
  http.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
  http.onNotFound([](){ http.sendHeader("Location","/",true); http.send(302,""); });
  http.begin();
  Serial.println("Portal ready");
}

// ========= watchdog =========
static void wdt_init(){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);   // TWDT is already initialised by the framework in core 3.x
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);           // watch the loop task
}

// ========= setup / loop =========
void setup(){
  motor_pins_safe_init();           // FIRST: pump driver held off before anything else runs

  Serial.begin(115200); delay(20);
  Serial.printf("SurgeFX Black Box v%s\n", FIRMWARE_VERSION);

  generate_ssid_suffix();
  load_presets_from_nvs();

  pinMode(DC_PIN,OUTPUT); digitalWrite(DC_PIN,HIGH);
  backlight_init(255);
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN,I2C_HZ);
  pmic_on(); tca_all_high(); lcd_reset_pulse();

#if USE_HW_SPI
  gfx->begin(HW_SPI_HZ);
#else
  gfx->begin();
#endif
  panel_post_begin();

  lv_init();
  lv_tick_set_cb([]() -> uint32_t { return millis(); });
  lv_display_t *disp = lv_display_create(LCD_W, LCD_H);
  lv_display_set_flush_cb(disp, lvgl_flush_cb);
  lv_display_set_default(disp);
  const uint16_t buf_h = 20;        // pass 2: raise to ~40 and add a second buffer once on HW SPI
  lvbuf1 = lv_draw_buf_create(LCD_W, buf_h, LV_COLOR_FORMAT_RGB565, 0);
  lv_display_set_draw_buffers(disp, lvbuf1, NULL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lv_touch_read_cb);

  plugQ = xQueueCreate(8, sizeof(PlugCmd));
  xTaskCreatePinnedToCore(plugTask, "plug", 6144, NULL, 1, NULL, 0);

  apply_styles();
  build_screen_main();
  build_screen_help();
  rebuild_menu_screen();
  lv_screen_load(scr_main);
  reflect_current_in_ui();

  start_portal();

  if (lblSupport) {
    char ssid_text[128];
    snprintf(ssid_text, sizeof(ssid_text), "WiFi AP: %s   (password: %s)", AP_SSID, AP_PASS);
    lv_label_set_text(lblSupport, ssid_text);
  }

  wdt_init();
  Serial.printf("Init done. %d presets loaded. Free heap: %u\n", g_preset_count, ESP.getFreeHeap());
}

void loop(){
  uint32_t wait_ms = lv_timer_handler();

  const int before = g_curr_pct;
  ramp_tick();
  if (g_curr_pct != before) reflect_current_in_ui();

  if (g_plug_ui_dirty.exchange(false)) update_plug_ui();

  dns.processNextRequest();
  http.handleClient();

#if DEBUG_HEAP
  static uint32_t last_mem = 0;
  if (millis() - last_mem > 10000) { last_mem = millis(); Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap()); }
#endif

  esp_task_wdt_reset();
  delay(constrain(wait_ms, 1u, 5u));   // never sleep past the ramp/HTTP cadence
}
