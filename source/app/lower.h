#ifndef _LOWER_H
#define _LOWER_H

enum lower_state_t {
    LOWER_DISABLE,
    LOWER_IDLE,
    LOWER_DOING_HAND_SHAKE,
    LOWER_CONNECTED,
    LOWER_READ,
    LOWER_WRITE,
    LOWER_DISCONNECTING,
};

enum device_type_t {
    AUDIO_DEVICE,
    HID_D_F_DEVICE,
    HID_E_G_DEVICE,
    MAX_DEVICE_TYPE
};

enum lower_state_t lower_get_state(void);
void lower_uart_tx_disconnect(void);
void lower_uart_tx_bin(void);
void lower_uart_reset(void);

#endif

