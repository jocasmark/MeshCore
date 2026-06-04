#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#if defined(ESP32) && defined(WIFI_SSID)
  #include <WiFi.h>
  #include <SPIFFS.h>
  #include <esp_system.h>
  #ifndef TCP_PORT
    #define TCP_PORT 5000
  #endif
  #ifndef TCP_IDLE_TIMEOUT_MS
    #define TCP_IDLE_TIMEOUT_MS  (10UL*60UL*1000UL)   // 10 min
  #endif
  #ifndef WIFI_RECONNECT_INTERVAL_MS
    #define WIFI_RECONNECT_INTERVAL_MS  30000UL
  #endif
  static WiFiServer tcp_server(TCP_PORT);
  static WiFiClient tcp_client;
  static char tcp_command[160];
  static bool tail_active = false;
  static bool tail_enabled_logging = false;
  static uint32_t tail_pos = 0;
  static unsigned long tail_next_poll = 0;
  static unsigned long tcp_last_activity = 0;
  static unsigned long wifi_next_check = 0;
  static bool tcp_server_started = false;

  static const char* reset_reason_str() {
    switch (esp_reset_reason()) {
      case ESP_RST_POWERON:  return "power-on";
      case ESP_RST_EXT:      return "external";
      case ESP_RST_SW:       return "software";
      case ESP_RST_PANIC:    return "panic";
      case ESP_RST_INT_WDT:  return "int-wdt";
      case ESP_RST_TASK_WDT: return "task-wdt";
      case ESP_RST_WDT:      return "wdt";
      case ESP_RST_DEEPSLEEP:return "deep-sleep";
      case ESP_RST_BROWNOUT: return "brownout";
      case ESP_RST_SDIO:     return "sdio";
      default:               return "unknown";
    }
  }
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

// For power saving
unsigned long POWERSAVING_FIRSTSLEEP_SECS = 120; // The first sleep (if enabled) from boot

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
static unsigned long userBtnDownAt = 0;
#define USER_BTN_HOLD_OFF_MILLIS 1500
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  // give some extra time for serial to settle so
  // boot debug messages can be seen on terminal
  delay(5000);
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

#if defined(ESP32) && defined(WIFI_SSID)
  Serial.print("WiFi: connecting to "); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  {
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(250);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi: connected to \""); Serial.print(WiFi.SSID());
    Serial.print("\" IP="); Serial.print(WiFi.localIP());
    Serial.print(" RSSI="); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    tcp_server.begin();
    tcp_server.setNoDelay(true);
    tcp_server_started = true;
    Serial.print("CLI listening on TCP port "); Serial.println(TCP_PORT);
  } else {
    Serial.println("WiFi: connect failed (will keep retrying in background)");
  }
  tcp_command[0] = 0;
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  board.onBootComplete();
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#if defined(ESP32) && defined(WIFI_SSID)
  // periodic WiFi watchdog: kick reconnect, start server once connected
  if ((long)(millis() - wifi_next_check) >= 0) {
    wifi_next_check = millis() + WIFI_RECONNECT_INTERVAL_MS;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi: not connected, retrying...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PWD);
    } else if (!tcp_server_started) {
      tcp_server.begin();
      tcp_server.setNoDelay(true);
      tcp_server_started = true;
      Serial.print("WiFi: late-connected IP="); Serial.print(WiFi.localIP());
      Serial.print(" CLI on port "); Serial.println(TCP_PORT);
    }
  }

  if (tcp_server.hasClient()) {
    if (tcp_client && tcp_client.connected()) {
      // only one client at a time
      tcp_server.available().stop();
    } else {
      tcp_client = tcp_server.available();
      tcp_client.setNoDelay(true);
      tcp_client.print("MeshCore Repeater CLI "); tcp_client.print(FIRMWARE_VERSION);
      tcp_client.print(" ("); tcp_client.print(FIRMWARE_BUILD_DATE); tcp_client.print(")\r\n");
      if (WiFi.status() == WL_CONNECTED) {
        tcp_client.print("WiFi: \""); tcp_client.print(WiFi.SSID());
        tcp_client.print("\"  IP "); tcp_client.print(WiFi.localIP());
        tcp_client.print("  RSSI "); tcp_client.print(WiFi.RSSI()); tcp_client.print(" dBm\r\n");
      }
      tcp_client.print("Type 'wifi', 'sysinfo', 'tail', 'help'.\r\n> ");
      tcp_command[0] = 0;
      tail_active = false;
      if (tail_enabled_logging) {
        the_mesh.setLoggingOn(false);
        tail_enabled_logging = false;
      }
      tcp_last_activity = millis();
    }
  }
  // idle timeout
  if (tcp_client && tcp_client.connected() &&
      (unsigned long)(millis() - tcp_last_activity) > TCP_IDLE_TIMEOUT_MS) {
    tcp_client.print("\r\n-- idle timeout, disconnecting --\r\n");
    tcp_client.stop();
    tail_active = false;
    if (tail_enabled_logging) {
      the_mesh.setLoggingOn(false);
      tail_enabled_logging = false;
    }
  }
  if (tcp_client && tcp_client.connected()) {
    if (tail_active) {
      // any byte from client stops tail
      if (tcp_client.available()) {
        while (tcp_client.available()) tcp_client.read();
        tail_active = false;
        if (tail_enabled_logging) {
          the_mesh.setLoggingOn(false);
          tail_enabled_logging = false;
        }
        tcp_last_activity = millis();
        tcp_client.print("\r\n-- tail stopped --\r\n> ");
      } else if ((long)(millis() - tail_next_poll) >= 0) {
        tail_next_poll = millis() + 500;
        File f = SPIFFS.open("/packet_log", "r");
        if (f) {
          uint32_t sz = f.size();
          if (sz < tail_pos) tail_pos = 0; // file truncated/erased
          if (sz > tail_pos) {
            f.seek(tail_pos);
            uint8_t buf[128];
            while (f.available()) {
              int n = f.read(buf, sizeof(buf));
              if (n <= 0) break;
              tcp_client.write(buf, n);
              tail_pos += n;
            }
          }
          f.close();
        }
      }
    } else {
    int tlen = strlen(tcp_command);
    while (tcp_client.available() && tlen < (int)sizeof(tcp_command)-1) {
      char c = tcp_client.read();
      tcp_last_activity = millis();
      if (c == '\r' || c == '\n') {
        tcp_command[tlen++] = '\n';
        tcp_command[tlen] = 0;
        break;
      }
      tcp_command[tlen++] = c;
      tcp_command[tlen] = 0;
    }
    if (tlen == (int)sizeof(tcp_command)-1) {
      tcp_command[sizeof(tcp_command)-1] = '\n';
    }
    if (tlen > 0 && tcp_command[tlen-1] == '\n') {
      tcp_command[tlen-1] = 0;
      char reply[160];
      if (strcasecmp(tcp_command, "wifi") == 0) {
        if (WiFi.status() == WL_CONNECTED) {
          tcp_client.print("SSID: "); tcp_client.print(WiFi.SSID()); tcp_client.print("\r\n");
          tcp_client.print("IP:   "); tcp_client.print(WiFi.localIP()); tcp_client.print("\r\n");
          tcp_client.print("GW:   "); tcp_client.print(WiFi.gatewayIP()); tcp_client.print("\r\n");
          tcp_client.print("MAC:  "); tcp_client.print(WiFi.macAddress()); tcp_client.print("\r\n");
          tcp_client.print("RSSI: "); tcp_client.print(WiFi.RSSI()); tcp_client.print(" dBm\r\n");
        } else {
          tcp_client.print("WiFi not connected\r\n");
        }
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "tail") == 0) {
        tail_enabled_logging = !the_mesh.isLoggingOn();
        if (tail_enabled_logging) the_mesh.setLoggingOn(true);
        File f = SPIFFS.open("/packet_log", "r");
        tail_pos = f ? f.size() : 0;
        if (f) f.close();
        tail_active = true;
        tail_next_poll = 0;
        tcp_client.print("-- live log (press any key to stop) --\r\n");
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "sysinfo") == 0) {
        unsigned long up = millis() / 1000;
        tcp_client.printf("Firmware:  %s (%s)\r\n", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);
        tcp_client.printf("Chip:      %s rev %d, %d cores @ %u MHz\r\n",
                          ESP.getChipModel(), ESP.getChipRevision(),
                          ESP.getChipCores(), (unsigned)ESP.getCpuFreqMHz());
        tcp_client.printf("Flash:     %u KB\r\n", (unsigned)(ESP.getFlashChipSize() / 1024));
        tcp_client.printf("Sketch:    %u / %u KB\r\n",
                          (unsigned)(ESP.getSketchSize() / 1024),
                          (unsigned)((ESP.getSketchSize() + ESP.getFreeSketchSpace()) / 1024));
        tcp_client.printf("Heap free: %u / %u KB (min %u)\r\n",
                          (unsigned)(ESP.getFreeHeap() / 1024),
                          (unsigned)(ESP.getHeapSize() / 1024),
                          (unsigned)(ESP.getMinFreeHeap() / 1024));
        tcp_client.printf("Uptime:    %lud %02luh %02lum %02lus\r\n",
                          up/86400, (up/3600)%24, (up/60)%60, up%60);
        tcp_client.printf("Reset:     %s\r\n", reset_reason_str());
        tcp_client.printf("WiFi:     ");
        if (WiFi.status() == WL_CONNECTED) {
          tcp_client.printf(" SSID=\"%s\" IP=%s RSSI=%d dBm\r\n",
                            WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
        } else {
          tcp_client.printf(" disconnected\r\n");
        }
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "log") == 0
              || strcasecmp(tcp_command, "log log") == 0) {
        File f = SPIFFS.open("/packet_log", "r");
        if (!f || f.size() == 0) {
          tcp_client.print("(log empty)\r\n");
        } else {
          uint8_t buf[128];
          while (f.available()) {
            int n = f.read(buf, sizeof(buf));
            if (n <= 0) break;
            tcp_client.write(buf, n);
          }
        }
        if (f) f.close();
        tcp_client.print("-- EOF --\r\n");
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "log start") == 0) {
        the_mesh.setLoggingOn(true);
        tcp_client.print("logging on\r\n");
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "log stop") == 0) {
        the_mesh.setLoggingOn(false);
        tcp_client.print("logging off\r\n");
        reply[0] = 0;
      } else if (strcasecmp(tcp_command, "log erase") == 0) {
        SPIFFS.remove("/packet_log");
        tail_pos = 0;
        tcp_client.print("log erased\r\n");
        reply[0] = 0;
      } else {
        the_mesh.handleCommand(0, tcp_command, reply);
      }
      if (reply[0]) {
        tcp_client.print("  -> "); tcp_client.print(reply); tcp_client.print("\r\n");
      }
      if (!tail_active) tcp_client.print("> ");
      tcp_command[0] = 0;
    }
    } // else (!tail_active)
  }
#endif

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
  // Hold the user button to power off the SenseCAP Solar repeater.
  int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) {
      Serial.println("Powering off...");
      board.powerOff();  // does not return
    }
  } else {
    userBtnDownAt = 0;
  }
#endif

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  if (the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#else
    if (the_mesh.millisHasNowPassed(POWERSAVING_FIRSTSLEEP_SECS * 1000)) { // To check if it is time to sleep
      board.sleep(30); // Sleep. Wake up after a while or when receiving a LoRa packet
    }
#endif
  }
}
