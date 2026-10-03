#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "pio_usb.h"
#include "pio_usb_ll.h"

// TinyUSB header to define USB descriptors & HID structures
#include "device/usbd.h"
#include "class/hid/hid_device.h"

static usb_device_t *usb_device = NULL;

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,

    .idVendor = 0xCafe,
    .idProduct = 0x4001,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01
};

enum {
  ITF_NUM_KEYBOARD,
  ITF_NUM_MOUSE,
  ITF_NUM_TOTAL,
};

enum {
  EPNUM_KEYBOARD = 0x81,
  EPNUM_MOUSE = 0x82,
};

uint8_t const desc_hid_keyboard_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD()
};

uint8_t const desc_hid_mouse_report[] = {
  TUD_HID_REPORT_DESC_MOUSE()
};

const uint8_t *report_desc[] = {
  desc_hid_keyboard_report,
  desc_hid_mouse_report
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN)
uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(ITF_NUM_KEYBOARD, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(desc_hid_keyboard_report), EPNUM_KEYBOARD,
                       CFG_TUD_HID_EP_BUFSIZE, 10),
    TUD_HID_DESCRIPTOR(ITF_NUM_MOUSE, 0, HID_ITF_PROTOCOL_MOUSE,
                       sizeof(desc_hid_mouse_report), EPNUM_MOUSE,
                       CFG_TUD_HID_EP_BUFSIZE, 1),
};

static_assert(sizeof(desc_device) == 18, "device desc size error");

const char *string_descriptors_base[] = {
    [0] = (const char[]){0x09, 0x04},
    [1] = "Pico KVM",
    [2] = "RP2350 HID Forwarder",
    [3] = "123456",
};
static string_descriptor_t str_desc[4];

static void init_string_desc(void) {
  for (int idx = 0; idx < 4; idx++) {
    uint8_t len = 0;
    uint16_t *wchar_str = (uint16_t *)&str_desc[idx];
    if (idx == 0) {
      wchar_str[1] = string_descriptors_base[0][0] |
                     ((uint16_t)string_descriptors_base[0][1] << 8);
      len = 1;
    } else if (idx <= 3) {
      len = strnlen(string_descriptors_base[idx], 31);
      for (int i = 0; i < len; i++) {
        wchar_str[i + 1] = string_descriptors_base[idx][i];
      }
    } else {
      len = 0;
    }

    wchar_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);
  }
}

static usb_descriptor_buffers_t desc = {
    .device = (uint8_t *)&desc_device,
    .config = desc_configuration,
    .hid_report = report_desc,
    .string = str_desc
};

// Keyboard report queue
#define KBD_QUEUE_SIZE 16
static hid_keyboard_report_t kbd_queue[KBD_QUEUE_SIZE];
static uint8_t kbd_q_head = 0;
static uint8_t kbd_q_tail = 0;

static inline bool kbd_queue_empty(void) {
  return kbd_q_head == kbd_q_tail;
}

static inline bool kbd_queue_full(void) {
  return ((kbd_q_head + 1) % KBD_QUEUE_SIZE) == kbd_q_tail;
}

static inline void kbd_queue_push(const hid_keyboard_report_t *report) {
  if (kbd_queue_full()) {
    kbd_q_tail = (kbd_q_tail + 1) % KBD_QUEUE_SIZE;
  }
  kbd_queue[kbd_q_head] = *report;
  kbd_q_head = (kbd_q_head + 1) % KBD_QUEUE_SIZE;
}

static inline hid_keyboard_report_t kbd_queue_pop(void) {
  hid_keyboard_report_t r = kbd_queue[kbd_q_tail];
  kbd_q_tail = (kbd_q_tail + 1) % KBD_QUEUE_SIZE;
  return r;
}

// Mouse state
static volatile uint8_t mouse_buttons = 0;
static volatile int32_t mouse_accum_x = 0;
static volatile int32_t mouse_accum_y = 0;
static volatile int32_t mouse_accum_wheel = 0;
static volatile bool mouse_dirty = false;

// Serial Packet Parser
enum parse_state_t {
  STATE_WAIT_CMD,
  STATE_WAIT_PAYLOAD,
};

static enum parse_state_t parse_state = STATE_WAIT_CMD;
static uint8_t current_cmd = 0;
static uint8_t payload_buf[4];
static uint8_t payload_idx = 0;
static uint32_t last_rx_time = 0;

static void handle_packet(uint8_t cmd, const uint8_t *payload) {
  if (cmd == 0x01) {
    // Keyboard packet: [0x01, modifiers, reserved, keycode, keycode2]
    hid_keyboard_report_t report = {0};
    report.modifier = payload[0];
    report.reserved = payload[1];
    report.keycode[0] = payload[2];
    report.keycode[1] = payload[3];
    kbd_queue_push(&report);
  } else if (cmd == 0x02) {
    // Mouse packet: [0x02, buttons, dx, dy, wheel]
    mouse_buttons = payload[0];
    mouse_accum_x += (int8_t)payload[1];
    mouse_accum_y += (int8_t)payload[2];
    mouse_accum_wheel += (int8_t)payload[3];
    mouse_dirty = true;
  }
}

static void process_serial_byte(uint8_t byte) {
  last_rx_time = to_ms_since_boot(get_absolute_time());

  if (parse_state == STATE_WAIT_CMD) {
    if (byte == 0x01 || byte == 0x02) {
      current_cmd = byte;
      payload_idx = 0;
      parse_state = STATE_WAIT_PAYLOAD;
    }
  } else if (parse_state == STATE_WAIT_PAYLOAD) {
    payload_buf[payload_idx++] = byte;
    if (payload_idx == 4) {
      handle_packet(current_cmd, payload_buf);
      parse_state = STATE_WAIT_CMD;
    }
  }
}

void core1_main(void) {
  sleep_ms(10);

  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  init_string_desc();
  usb_device = pio_usb_device_init(&config, &desc);

  while (true) {
    pio_usb_device_task();
  }
}

int main(void) {
  // default 125MHz is not appropriate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(180000, true);

  stdio_init_all();
  stdio_set_translate_crlf(&stdio_usb, false);

  multicore_reset_core1();
  // all PIO USB tasks run in core1
  multicore_launch_core1(core1_main);

  while (usb_device == NULL) {
    tight_loop_contents();
  }

  endpoint_t *ep_kb = pio_usb_get_endpoint(usb_device, 1);
  endpoint_t *ep_mouse = pio_usb_get_endpoint(usb_device, 2);

  uint32_t kb_last_tx_time = 0;
  uint32_t mouse_last_tx_time = 0;

  while (true) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Recover if endpoint transfer timed out (> 50ms)
    if (ep_kb && ep_kb->has_transfer && (now - kb_last_tx_time > 50)) {
      pio_usb_ll_transfer_complete(ep_kb, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
    if (ep_mouse && ep_mouse->has_transfer && (now - mouse_last_tx_time > 50)) {
      pio_usb_ll_transfer_complete(ep_mouse, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }

    // Reset parser if partial packet timed out (> 100ms)
    if (parse_state != STATE_WAIT_CMD && (now - last_rx_time > 100)) {
      parse_state = STATE_WAIT_CMD;
    }

    // Read all available bytes from USB CDC
    while (true) {
      int ch = getchar_timeout_us(0);
      if (ch < 0) {
        break;
      }
      process_serial_byte((uint8_t)ch);
    }

    // Forward keyboard events
    if (!kbd_queue_empty() && ep_kb && !ep_kb->has_transfer) {
      hid_keyboard_report_t r = kbd_queue_pop();
      if (pio_usb_set_out_data(ep_kb, (const uint8_t *)&r, sizeof(r)) == 0) {
        kb_last_tx_time = now;
      }
    }

    // Forward mouse events
    if (mouse_dirty && ep_mouse && !ep_mouse->has_transfer) {
      hid_mouse_report_t r = {0};
      r.buttons = mouse_buttons;

      int32_t send_x = mouse_accum_x;
      if (send_x > 127) send_x = 127;
      else if (send_x < -127) send_x = -127;
      r.x = (int8_t)send_x;
      mouse_accum_x -= send_x;

      int32_t send_y = mouse_accum_y;
      if (send_y > 127) send_y = 127;
      else if (send_y < -127) send_y = -127;
      r.y = (int8_t)send_y;
      mouse_accum_y -= send_y;

      int32_t send_wheel = mouse_accum_wheel;
      if (send_wheel > 127) send_wheel = 127;
      else if (send_wheel < -127) send_wheel = -127;
      r.wheel = (int8_t)send_wheel;
      mouse_accum_wheel -= send_wheel;

      if (mouse_accum_x == 0 && mouse_accum_y == 0 && mouse_accum_wheel == 0) {
        mouse_dirty = false;
      }

      if (pio_usb_set_out_data(ep_mouse, (const uint8_t *)&r, sizeof(r)) == 0) {
        mouse_last_tx_time = now;
      }
    }
  }
}