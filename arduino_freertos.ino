#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <Arduino_FreeRTOS.h>

// ======================================================
// Forward declarations (importante in C++ per FreeRTOS)
// ======================================================
void send_cmd_printf(const char* fmt, ...);

// =====================
// Bit layout
// =====================
//
// FRAME TYPE (bit7):
//   0 = DATA
//   1 = CMD
//
// DATA header (type=0):
//   bit6: subtype (0=REQ, 1=RESP)
//   bit5..bit0: payload_len (0..63)
//
// CMD header (type=1):
//   bit6..bit4: cmd_id (0..7)
//   bit3..bit0: payload_len (0..15)

// ---- Masks / constants ----
static const uint8_t m_FRAME_TYPE = 1 << 7;   // 0=DATA, 1=CMD

// DATA
static const uint8_t m_DATA_SUBTYPE = 1 << 6;
static const uint8_t DATA_SUBTYPE_REQ  = 0;
static const uint8_t DATA_SUBTYPE_RESP = 1;

// CMD
static const uint8_t m_CMD_TYPE = 0x70;       // bits 6..4
static const uint8_t CMD_OPEN_MAIN       = 0b000;
static const uint8_t CMD_OPEN_DROGUE     = 0b001;
static const uint8_t CMD_SET_AIR_BRAKES  = 0b010;
static const uint8_t CMD_PRINT           = 0b011;

// ERR
enum error_t {
  ERR_UNKNOWN_SUBTYPE = 400,
  ERR_MISMATCH_SIM_PAYLOAD_LEN,
  ERR_CMD_FRAME_FROM_PY,
  ERR_DATA_REQ_FROM_PY
};

// Telemetry payload = 7 float32 = 28 bytes
struct __attribute__((packed)) SimulationPayload28B {
  float ax, ay, az;
  float pressure;
  float lat, lon, alt;
};

static const uint8_t SIM_PAYLOAD_LEN = sizeof(SimulationPayload28B); // 28

// ---- Header helpers ----
static inline uint8_t set_frame_type(uint8_t header, uint8_t is_cmd) {
  if (is_cmd) header |= m_FRAME_TYPE;
  else header &= (uint8_t)(~m_FRAME_TYPE);
  return header;
}
static inline uint8_t get_frame_type(uint8_t header) {
  return (header & m_FRAME_TYPE) >> 7;
}

static inline uint8_t set_data_subtype(uint8_t header, uint8_t subtype) {
  if (subtype == DATA_SUBTYPE_RESP) {
    header |= m_DATA_SUBTYPE;
  } else if (subtype == DATA_SUBTYPE_REQ) {
    header &= (uint8_t)(~m_DATA_SUBTYPE);
  } else {
    // Nota: qui stai "loggando" su Serial, quindi attenzione a chiamare
    // questa funzione in contesti dove Serial non è pronta.
    send_cmd_printf("E%d", ERR_UNKNOWN_SUBTYPE);
  }
  return header;
}
static inline uint8_t get_data_subtype(uint8_t header) {
  return (header & m_DATA_SUBTYPE) >> 6;
}

static inline uint8_t set_data_payload_len(uint8_t header, uint8_t len) {
  // lower 6 bits
  header |= (len & 0x3F);
  return header;
}
static inline uint8_t get_data_payload_len(uint8_t header) {
  return header & 0x3F;
}

static inline uint8_t set_cmd(uint8_t header, uint8_t cmd_id) {
  header |= (uint8_t)((cmd_id & 0x07) << 4);
  return header;
}
static inline uint8_t get_cmd(uint8_t header) {
  return (header & 0x70) >> 4;
}

static inline uint8_t set_cmd_payload_len(uint8_t header, uint8_t len) {
  header |= (len & 0x0F);
  return header;
}
static inline uint8_t get_cmd_payload_len(uint8_t header) {
  return header & 0x0F;
}

static inline uint8_t build_data_req_header() {
  uint8_t h = 0x00;
  h = set_frame_type(h, 0);                  // DATA
  h = set_data_subtype(h, DATA_SUBTYPE_REQ);
  h = set_data_payload_len(h, 0);            // no payload
  return h;
}

static inline uint8_t build_cmd_header(uint8_t cmd_id, uint8_t payload_len) {
  uint8_t h = 0x00;
  h = set_frame_type(h, 1);                  // CMD
  h = set_cmd(h, cmd_id);
  h = set_cmd_payload_len(h, payload_len);
  return h;
}

// =====================
// TX: REQ + CMD
// =====================
static inline void send_data_req() {
  uint8_t h = build_data_req_header();
  Serial.write(&h, 1);
}

static inline void send_cmd_open_main() {
  uint8_t h = build_cmd_header(CMD_OPEN_MAIN, 0);
  Serial.write(&h, 1);
}

static inline void send_cmd_open_drogue() {
  uint8_t h = build_cmd_header(CMD_OPEN_DROGUE, 0);
  Serial.write(&h, 1);
}

static inline void send_cmd_set_air_brakes(float level01) {
  if (level01 < 0.0f) level01 = 0.0f;
  if (level01 > 1.0f) level01 = 1.0f;

  uint8_t h = build_cmd_header(CMD_SET_AIR_BRAKES, 4);
  Serial.write(&h, 1);
  Serial.write((uint8_t*)&level01, 4); // ESP32 little-endian
}

void send_cmd_printf(const char* fmt, ...) {
  static char print_buf[16];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(print_buf, sizeof(print_buf), fmt, args);
  va_end(args);

  if (n < 0) return;

  uint8_t len = (n >= (int)sizeof(print_buf)) ? (sizeof(print_buf) - 1) : (uint8_t)n;
  uint8_t h = build_cmd_header(CMD_PRINT, len);

  Serial.write(&h, 1);
  Serial.write((uint8_t*)print_buf, len);
}

// =====================
// RX FSM (byte collector)
// =====================
enum RxState {
  RX_WAIT_HEADER = 0,
  RX_WAIT_PAYLOAD = 1
};

static RxState rx_state = RX_WAIT_HEADER;
static uint8_t want_len = 0;
static uint8_t buf[64];
static uint8_t got = 0;

static bool main_deployed = false;
static bool drogue_deployed = false;

static void rx_reset() {
  rx_state = RX_WAIT_HEADER;
  want_len = 0;
  got = 0;
}

static char temp[50];

static void print_sensors(const SimulationPayload28B& sensors) {
  int min_width = 0;
  int precision = 4;

  dtostrf(sensors.ax, min_width, precision, temp); send_cmd_printf("ax=%s", temp);
  dtostrf(sensors.ay, min_width, precision, temp); send_cmd_printf("ay=%s", temp);
  dtostrf(sensors.az, min_width, precision, temp); send_cmd_printf("az=%s", temp);
  dtostrf(sensors.pressure, min_width, precision, temp); send_cmd_printf("p=%s", temp);
  dtostrf(sensors.lat, min_width, precision, temp); send_cmd_printf("lat=%s", temp);
  dtostrf(sensors.lon, min_width, precision, temp); send_cmd_printf("lon=%s", temp);
  dtostrf(sensors.alt, min_width, precision, temp); send_cmd_printf("alt=%s", temp);
}

static float maximum_altitude_reached = -1.0f;
static float deployment_level = 1.0f;
static bool airbrakes_deployed = false;

static void handle_sim_payload(const uint8_t* payload, uint8_t len) {
  if (len != SIM_PAYLOAD_LEN) {
    send_cmd_printf("E%d", ERR_MISMATCH_SIM_PAYLOAD_LEN);
    return;
  }

  SimulationPayload28B p;
  memcpy(&p, payload, sizeof(p));

  // print_sensors(p);

  // Apogee detection (semplice): se stai scendendo rispetto al max visto
  if (maximum_altitude_reached > p.alt && !drogue_deployed) {
    dtostrf(maximum_altitude_reached, 0, 4, temp);
    send_cmd_printf("apogee=%s", temp);
    send_cmd_open_drogue();
    drogue_deployed = true;
  }

  if (maximum_altitude_reached < p.alt) {
    maximum_altitude_reached = p.alt;
  }

  // Esempio airbrakes
  if (!drogue_deployed && p.alt > 1000 && p.alt < 2000 && !airbrakes_deployed) {
    send_cmd_set_air_brakes(deployment_level);
    airbrakes_deployed = true;
  }

  if (!drogue_deployed && p.alt > 2000 && airbrakes_deployed) {
    airbrakes_deployed = false;
    deployment_level = 0;
    send_cmd_set_air_brakes(deployment_level);
  }

  // Main sotto quota
  if (p.alt < 450.0f && drogue_deployed && !main_deployed) {
    send_cmd_open_main();
    main_deployed = true;
  }
}

static void poll_rx_once() {
  // “once” = consuma ciò che c’è, senza bloccare
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) return;

    uint8_t b = (uint8_t)c;

    if (rx_state == RX_WAIT_HEADER) {
      uint8_t header = b;
      uint8_t t = get_frame_type(header);

      if (t == 1) {
        // CMD da Python: in questo design non dovrebbe arrivare
        send_cmd_printf("E%d", ERR_CMD_FRAME_FROM_PY);

        uint8_t plen = get_cmd_payload_len(header);
        want_len = plen;
        got = 0;

        if (want_len == 0) {
          rx_reset();
        } else {
          rx_state = RX_WAIT_PAYLOAD;
          // consumiamo solo i bytes, senza dispatch
        }
        continue;
      }

      // DATA frame
      uint8_t subtype = get_data_subtype(header);
      uint8_t plen = get_data_payload_len(header);

      if (subtype != DATA_SUBTYPE_RESP) {
        send_cmd_printf("E%d", ERR_DATA_REQ_FROM_PY);
        rx_reset();
        continue;
      }

      if (plen == 0) {
        rx_reset();
        continue;
      }

      want_len = plen;
      got = 0;
      rx_state = RX_WAIT_PAYLOAD;
      continue;
    }

    // RX_WAIT_PAYLOAD
    buf[got++] = b;

    if (got >= want_len) {
      handle_sim_payload(buf, want_len);
      rx_reset();
    }
  }
}

// ======================================================
// FreeRTOS Task: comm + periodic request + polling rx
// ======================================================
static void SimLinkTask(void* param) {
  (void)param;

  const TickType_t periodTicks = pdMS_TO_TICKS(50); // 20 Hz
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // 1) periodic pull
    send_data_req();

    // 2) drain serial rx quickly
    //    (se ti arrivano burst, puoi farlo più volte o mettere un limite)
    poll_rx_once();

    // 3) attesa fino al prossimo tick (periodico “stabile”)
    vTaskDelayUntil(&lastWake, periodTicks);

  }
}

static void HeartbeatTask(void* param) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000);

  for (;;) {
      send_cmd_printf("alive");

      vTaskDelayUntil(&lastWake, period);
  }
}


// =====================
// Arduino setup/loop
// =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  rx_reset();

  xTaskCreate(
    SimLinkTask,          // task function
    "SimLink",            // name
    128,                  // stack (words)
    nullptr,              // param
    2,                    // priority
    nullptr               // handle
  );

  // xTaskCreate(
  //   HeartbeatTask,
  //   "Heartbeat",
  //   128,
  //   nullptr,
  //   2,
  //   nullptr
  // );

  // vTaskStartScheduler();
}

void loop() {
  // Vuoto: la logica è nel task.
  // (Volendo puoi farci blink LED o watchdog feed, ecc.)
}
