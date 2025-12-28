#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

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
static const uint8_t m_FRAME_TYPE = 1 << 7;  // 0=DATA, 1=CMD

// DATA
static const uint8_t m_DATA_SUBTYPE = 1 << 6;
static const uint8_t DATA_SUBTYPE_REQ = 0;
static const uint8_t DATA_SUBTYPE_RESP = 1;

// CMD
static const uint8_t m_CMD_TYPE = 0x70;  // bits 6..4
static const uint8_t CMD_OPEN_MAIN = 0b000;
static const uint8_t CMD_OPEN_DROGUE = 0b001;
static const uint8_t CMD_SET_AIR_BRAKES = 0b010;
static const uint8_t CMD_PRINT = 0b011;

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
static const uint8_t SIM_PAYLOAD_LEN = sizeof(SimulationPayload28B);  // 28

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
  h = set_frame_type(h, 0);  // DATA
  h = set_data_subtype(h, DATA_SUBTYPE_REQ);
  h = set_data_payload_len(h, 0);  // no payload
  return h;
}

static inline uint8_t build_cmd_header(uint8_t cmd_id, uint8_t payload_len) {
  uint8_t h = 0x00;
  h = set_frame_type(h, 1);  // CMD
  h = set_cmd(h, cmd_id);
  h = set_cmd_payload_len(h, payload_len);
  return h;
}

// =====================
// TX: REQ + CMD
// =====================
void send_data_req() {
  uint8_t h = build_data_req_header();
  Serial.write(&h, 1);
}

void send_cmd_open_main() {
  uint8_t h = build_cmd_header(CMD_OPEN_MAIN, 0);
  Serial.write(&h, 1);
}

void send_cmd_open_drogue() {
  uint8_t h = build_cmd_header(CMD_OPEN_DROGUE, 0);
  Serial.write(&h, 1);
}

void send_cmd_set_air_brakes(float level01) {
  if (level01 < 0.0f) level01 = 0.0f;
  if (level01 > 1.0f) level01 = 1.0f;

  uint8_t h = build_cmd_header(CMD_SET_AIR_BRAKES, 4);
  Serial.write(&h, 1);
  Serial.write((uint8_t*)&level01, 4);  // ESP32 is little-endian
}

void send_cmd_printf(const char* fmt, ...) {
  static char print_buf[16];

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(print_buf, sizeof(print_buf), fmt, args);
  va_end(args);

  if (n < 0) {
    return;
  }

  // n = chars that *would* have been written (excluding '\0')
  // If truncated, send the truncated content (max 15)
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

static bool main_deployed = 0;
static bool drogue_deployed = 0;

static void rx_reset() {
  rx_state = RX_WAIT_HEADER;
  want_len = 0;
  got = 0;
}

static char temp[50];
void print_sensors(SimulationPayload28B sensors) {
  int min_width = 0;
  int precision = 4;
  dtostrf(sensors.ax, min_width, precision, temp);
  send_cmd_printf("ax=%s", temp);
  
  dtostrf(sensors.ay, min_width, precision, temp);
  send_cmd_printf("ay=%s", temp);
  
  dtostrf(sensors.az, min_width, precision, temp);
  send_cmd_printf("az=%s", temp);
  
  dtostrf(sensors.pressure, min_width, precision, temp);
  send_cmd_printf("p=%s", temp);
  
  dtostrf(sensors.lat, min_width, precision, temp);
  send_cmd_printf("lat=%s", temp);
  
  dtostrf(sensors.lon, min_width, precision, temp);
  send_cmd_printf("lon=%s", temp);
  
  dtostrf(sensors.lat, min_width, precision, temp);
  send_cmd_printf("alt=%s", temp);
}

static float maximum_altitude_reached = -1.0f;
static float deployment_level = 1;
static float deployed = 0;
static void handle_sim_payload(const uint8_t* payload, uint8_t len) {
  // [ dump payload ]
  // for(int i = 0; i < len; i++) {
  //   send_cmd_printf("%x", payload[i]);
  // }

  if (len != SIM_PAYLOAD_LEN) {
    send_cmd_printf("E%d", ERR_MISMATCH_SIM_PAYLOAD_LEN);
    return;

  }
  SimulationPayload28B p;
  memcpy(&p, payload, sizeof(p));
  // print_sensors(p);

  if (maximum_altitude_reached > p.alt && !drogue_deployed) { // apogee
    dtostrf(maximum_altitude_reached, 0, 4, temp);
    send_cmd_printf("apogee=%s", temp);
    send_cmd_open_drogue();
    drogue_deployed = 1;
  }

  if (maximum_altitude_reached < p.alt) { // new height record
    maximum_altitude_reached = p.alt;
  }

  if (!drogue_deployed && p.alt > 1000 && p.alt < 2000 && !deployed) { // open airbrakes at 1000m
    send_cmd_set_air_brakes(deployment_level);
    deployed = 1;
    // deployment_level += 0.01;
    // if (deployment_level > 1) {
    //   deployment_level = 0;
    // }
  }
  if (!drogue_deployed && p.alt > 2000 && deployed) { // close airbrakes at 2000m
    deployed = 0;
    deployment_level = 0;
    send_cmd_set_air_brakes(deployment_level);
  }


  if (p.alt < 450.0f && drogue_deployed && !main_deployed) {
    send_cmd_open_main();
    main_deployed = 1;
  }
}

static void poll_rx() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) return;
    uint8_t b = (uint8_t)c;

    if (rx_state == RX_WAIT_HEADER) {
      uint8_t header = b;
      uint8_t t = get_frame_type(header);

      if (t == 1) {  // CMD frame received from Python (currently not expected!).
        send_cmd_printf("E%d", ERR_CMD_FRAME_FROM_PY);
        // Consume/Skip payload bytes (collector-style)
        uint8_t cmd_id = get_cmd(header);
        uint8_t plen = get_cmd_payload_len(header);
        want_len = plen;
        got = 0;
        if (want_len == 0) {
          rx_reset();
        } else {
          rx_state = RX_WAIT_PAYLOAD;
          // Reuse buf; but we don't dispatch anything for CMD-from-Python now.
        }
        continue;
      }

      // DATA frame
      uint8_t subtype = get_data_subtype(header);
      uint8_t plen = get_data_payload_len(header);

      // We only care about DATA RESP from Python
      if (subtype != DATA_SUBTYPE_RESP) {
        // It's weird to receive DATA REQ from Python: ignore.
        send_cmd_printf("E%d", ERR_DATA_REQ_FROM_PY);
        rx_reset();
        continue;
      }

      if (plen == 0) {
        // NO DATA
        // (ESP32 continues; next REQ later)
        rx_reset();
        continue;
      }

      // OK: collect payload
      want_len = plen;
      got = 0;
      rx_state = RX_WAIT_PAYLOAD;
      continue;
    }

    // RX_WAIT_PAYLOAD
    buf[got++] = b;
    if (got >= want_len) {
      // We only enter WAIT_PAYLOAD for DATA RESP in this design
      handle_sim_payload(buf, want_len);
      rx_reset();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  rx_reset();
}

void loop() {
  static uint32_t lastReqMs = 0;
  const uint32_t reqPeriodMs = 50;    // 20 Hz
  // const uint32_t reqPeriodMs = 1000;  //  1 sec

  // 1) Periodic PULL: DATA REQ
  uint32_t now = millis();
  if (now - lastReqMs >= reqPeriodMs) {
    lastReqMs = now;
    send_data_req();
  }

  // 2) Poll RX (non blocking)
  poll_rx();
}
