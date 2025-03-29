#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "bsp/board_api.h"
#include "hid_report_parser.h"


struct MountedGamepad {
    hid::BitField<hid::GamepadConfig::NUM_BUTTONS> buttons;
    hid::Int32Array<hid::GamepadConfig::NUM_AXES> axes;
    hid::SelectiveInputReportParser parser;
};


struct JoyStickData {
    int8_t x;
    int8_t y;
};


union ButtonsData {
    struct {
        uint16_t west: 1;
        uint16_t south: 1;
        uint16_t east: 1;
        uint16_t north: 1;
        uint16_t left_shoulder: 1;
        uint16_t right_shoulder: 1;
        uint16_t left_trigger: 1;
        uint16_t right_trigger: 1;
        uint16_t select: 1; // 8
        uint16_t start: 1; // 9
        uint16_t left_joystick: 1;
        uint16_t right_joystick: 1;
        uint16_t home: 1;
        uint16_t share: 1;
    };
    uint16_t raw;
};


struct GamepadData {
    struct JoyStickData left_joystick;
    struct JoyStickData right_joystick;
    union ButtonsData buttons;
    uint8_t left_trigger;
    uint8_t right_trigger;
    uint8_t dpad;
};


static uart_inst_t *UART_ID = uart1;

static uint16_t uart_interval_ms = 250;
static uint8_t gamepad_dev_addr = 0;
static uint8_t gamepad_idx = 0;
static std::unique_ptr<MountedGamepad> p = nullptr;
static struct GamepadData gamepad_data = { { 0, 0 }, { 0, 0 }, 0, 0, 0, 0};


static uint8_t crc8(uint8_t *data, uint8_t len) {
    const uint8_t CRC8_GENERATE_POLYNOMIAL = 0xD5;
    const uint8_t CRC8_INITIAL_VALUE = 0xFF;
    const uint8_t CRC8_FINAL_XOR = 0xFF;

    uint8_t crc = CRC8_INITIAL_VALUE;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (crc << 1) ^ CRC8_GENERATE_POLYNOMIAL;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc ^ CRC8_FINAL_XOR;
}


static void core1_main() {
    const uint8_t DATA_LEN = 9;

    const uint8_t SBTP_HEADER_BYTE = 0x55;
    const uint8_t SBTP_FOOTER_BYTE = 0xAA;
    const uint8_t SBTP_ESCAPE_BYTE = 0x5A;
    const uint8_t SBTP_XOR_BYTE = 0x42;

    while (true) {
        uint8_t data[DATA_LEN] = {
            gamepad_data.left_joystick.x,
            gamepad_data.left_joystick.y,
            gamepad_data.right_joystick.x,
            gamepad_data.right_joystick.y,
            (gamepad_data.buttons.raw & 0xFF00) >> 8,
            gamepad_data.buttons.raw & 0x00FF,
            gamepad_data.left_trigger,
            gamepad_data.right_trigger,
            gamepad_data.dpad
        };

        uint8_t payload[DATA_LEN * 2];
        uint8_t payload_len = 0;

        for (uint8_t i = 0; i < DATA_LEN; i++) {
            if (data[i] == SBTP_HEADER_BYTE || data[i] == SBTP_FOOTER_BYTE || data[i] == SBTP_ESCAPE_BYTE) {
                payload[payload_len] = SBTP_ESCAPE_BYTE;
                payload_len++;
                payload[payload_len] = data[i] ^ SBTP_XOR_BYTE;
                payload_len++;
            }
            else {
                payload[payload_len] = data[i];
                payload_len++;
            }
        }

        uint8_t frame[DATA_LEN * 2];
        frame[0] = SBTP_HEADER_BYTE;
        frame[1] = payload_len;
        for (uint8_t i = 0; i < payload_len; i++) {
            frame[2 + i] = payload[i];
        }
        frame[2 + payload_len] = crc8(payload, payload_len);
        frame[3 + payload_len] = SBTP_FOOTER_BYTE;

        uart_write_blocking(UART_ID, frame, 4 + payload_len);

        sleep_ms(uart_interval_ms);
    }
}


static void print_gamepad_data(struct GamepadData *data) {
    printf(
        "Left X: %4d Y: %4d, Right X: %4d Y: %4d \r\n",
        data->left_joystick.x,
        data->left_joystick.y,
        data->right_joystick.x,
        data->right_joystick.y
    );
    printf(
        "A: %d, B: %d, X: %d, Y: %d, Dpad %d \r\n",
        data->buttons.east,
        data->buttons.south,
        data->buttons.north,
        data->buttons.west,
        data->dpad
    );
    printf(
        "L1: %d, R1: %d, L2: %d, R2: %d, L3: %d, R3: %d \r\n",
        data->buttons.left_shoulder,
        data->buttons.right_shoulder,
        data->buttons.left_trigger,
        data->buttons.right_trigger,
        data->buttons.left_joystick,
        data->buttons.right_joystick
    );
    printf(
        "Trigger: Left %3d Right %3d \r\n",
        data->left_trigger,
        data->right_trigger
    );
    printf(
        "Select: %d, Start: %d, Share: %d, Home: %d \r\n",
        data->buttons.select,
        data->buttons.start,
        data->buttons.share,
        data->buttons.home
    );
    printf("\e[0;0H");
}


static void read_gamepad_task() {
    if (gamepad_dev_addr == 0) {
        return;
    }

    if (!tuh_hid_receive_ready(gamepad_dev_addr, gamepad_idx)) {
        return;
    }

    if (!tuh_hid_receive_report(gamepad_dev_addr, gamepad_idx)) {
        printf("Error: cannot request to receive report\r\n");
    }
}


static void print_gamepad_data_task() {
    const uint16_t PRINT_INTERVAL_MS = 100;

    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < PRINT_INTERVAL_MS) {
        return;
    }

    start_ms += PRINT_INTERVAL_MS;

    print_gamepad_data(&gamepad_data);
}


int main() {
    board_init();
    stdio_init_all();
    sleep_ms(1000);

    tuh_init(BOARD_TUH_RHPORT);
    printf("Info: TinyUSB Host initialized\r\n");

    const uint32_t UART_BAUD_RATE_BPS = 115200;
    const uint8_t UART_DATA_BITS = 8;
    const uint8_t UART_STOP_BITS = 1;
    const uart_parity_t UART_PARITY = UART_PARITY_NONE;
    const uint8_t UART_TX_PIN = 4;
    const uint8_t UART_RX_PIN = 5;

    uart_init(UART_ID, UART_BAUD_RATE_BPS);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
    uart_set_hw_flow(UART_ID, false, false);
    printf("Info: UART initialized\r\n");

    multicore_launch_core1(core1_main);
    printf("Info: Core1 running UART task\r\n");

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    printf("Info: Core0 running USB task\r\n");
    while (true) {
        tuh_task();
        read_gamepad_task();
        // print_gamepad_data_task();
    }
}


void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len) {
    if (desc_report == NULL && desc_len == 0) {
        printf("Error: Report descriptor is too big\r\n");
        return;
    }

    if (gamepad_dev_addr != 0) {
        printf("Error: Gamepad already mounted\r\n");
        return;
    }

    p = std::unique_ptr<MountedGamepad>(new MountedGamepad);
    hid::BitFieldRef buttons_ref = p->buttons.Ref();
    hid::Int32ArrayRef axes_ref = p->axes.Ref();
    hid::GamepadConfig cfg;
    hid::Collection *cfg_root = cfg.Init(&buttons_ref, &axes_ref);

    int result = p->parser.Init(cfg_root, desc_report, desc_len);

    if (result) {
        printf("Error: parser init failed: result=%s[%d] desc_size=%u\r\n", hid::str_error(result, "UNKNOWN"), result, desc_len);
        p.reset();
        return;
    }

    printf("Info: Gamepad mounted. address: 0x%02X, idx: %u\r\n", dev_addr, idx);

    uart_interval_ms = 4;
    gamepad_dev_addr = dev_addr;
    gamepad_idx = idx;
}


void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
    if (gamepad_dev_addr != dev_addr || gamepad_idx != idx) {
        return;
    }

    printf("Info: Gamepad unmounted. address: 0x%02X, idx: %u\r\n", dev_addr, idx);

    uart_interval_ms = 250;
    gamepad_dev_addr = 0;
    gamepad_idx = 0;
    p.reset();
    gamepad_data = { { 0, 0 }, { 0, 0 }, 0, 0, 0, 0};
}


void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const *report, uint16_t len) {
    if (gamepad_dev_addr != dev_addr || gamepad_idx != idx) { return; } // ゲームパッド以外のデバイス

    board_led_write(true);

    int result = p->parser.Parse(report, len);
    if (result) {
        printf("Error: parse failed: result=%s[%d] report_size=%u\r\n", hid::str_error(result, "UNKNOWN"), result, len);
        return;
    }

    int16_t lx = p->axes[hid::GamepadConfig::X];
    int16_t ly = p->axes[hid::GamepadConfig::Y];
    struct JoyStickData left_joystick = { lx - 128, ly - 128 };
    int16_t rx = p->axes[hid::GamepadConfig::Z];
    int16_t ry = p->axes[hid::GamepadConfig::RZ];
    struct JoyStickData right_joystick = { rx - 128, ry - 128 };

    union ButtonsData buttons;
    buttons.raw = p->buttons.Flags<uint16_t>(0);

    uint8_t left_trigger = p->axes[hid::GamepadConfig::RX];
    uint8_t right_trigger = p-> axes[hid::GamepadConfig::RY];

    uint8_t dpad = p->axes[hid::GamepadConfig::HAT_SWITCH];

    gamepad_data = { left_joystick, right_joystick, buttons, left_trigger, right_trigger, dpad };

    board_led_write(false);
}
