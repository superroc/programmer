#include <string.h>
#include "usart.h"
#include "ff.h"
#include "malloc.h"
#include "stm32f10x_type.h"
#include "led.h"
#include "lower.h"

void uart_tx(u8 *data, u16 len);
void uart2_tx(u8 c);

#define ENABLE_AUDIO_DEVICE     1

enum lower_state_t lower_state = LOWER_DISABLE;
#if ENABLE_AUDIO_DEVICE == 1
enum device_type_t device_type = AUDIO_DEVICE;
__packed struct boot_pkt_t
{
    u8 code;
    u8 addr_hh;
    u8 addr_hl;
    u8 addr_h;
    u8 addr_l;
    u8 len_hh;
    u8 len_hl;
    u8 len_h;
    u8 len_l;
};
#define SINGLE_PACKET_SIZE      1024
const u8 boot_conn_ack[] = {'b' + 100, 'l' + 100, 'u' + 100, 'e' + 100, 's' + 100, 'p' + 100, 'e' + 100, 'a' + 100, 'k' + 100};
#define CODE_READ           0x05
#define CODE_READ_ACK       0x10
#define CODE_WRITE          0x06
#define CODE_WRITE_ACK      0x20
#define CODE_DISCONN        0x09
#define CODE_DISCONN_ACK    0x90
#define CODE_READMEM        0x03
#define CODE_READMEM_ACK    0x30
#define CODE_WRITEMEM       0x04
#define CODE_WRITEMEM_ACK   0x40
#define CODE_ERROR          0xf0//not used
#define CODE_NO_EEPROM_ACK  0x0f//
#else
enum device_type_t device_type = HID_D_F_DEVICE;
__packed struct boot_pkt_t
{
    u32 addr_h: 4;
    u32 code: 4;
    u32 addr_l: 8;
    u32 len_h: 8;
    u32 len_l: 8;
};
#define SINGLE_PACKET_SIZE      256
const u8 boot_conn_ack[] = {'z' + 100, 'c' + 100, 'a' + 100, 'n' + 100, 'z' + 100, 'c' + 100, 'a' + 100, 'n' + 100, '0' + 100};
#define CODE_READ           0x01
#define CODE_READ_ACK       0x02
#define CODE_WRITE          0x03
#define CODE_WRITE_ACK      0x04
#define CODE_DISCONN        0x05
#define CODE_DISCONN_ACK    0x06
#define CODE_READMEM        0x07
#define CODE_READMEM_ACK    0x08
#define CODE_WRITEMEM       0x09
#define CODE_WRITEMEM_ACK   0x0a
#define CODE_ERROR          0x0e//not used
#define CODE_NO_EEPROM_ACK  0x0f//
#endif

__packed struct boot_mem_t{
	u32  address;
	u8    value;
};

const u8 boot_conn_req[] = {0x04, 0x0f, 0x04, 0x00, 0x01, 0x00, 0x00};
const u8 boot_conn_success[] = {0x04, 0x0f, 0x04, 0x01, 0x01, 0x00, 0x00};
const u8 bd_addr_offset[MAX_DEVICE_TYPE][2] = {{0x0e, 0x13}, {0x89, 0x8e}, {0x85, 0x8a}};

u32 current_received_count = 0;
//下位机作为烧写flash的发起端，在完成一次读写后所执行的动作由此回调函数执行
void (*lower_uart_finish_callback)() = lower_uart_tx_bin;
u32 current_tx_index = 0;
u8 *lower_tx_buffer = NULL;
u32 read_total_count = 0;
u8 *lower_rx_buffer = 0;
u8 read_wait_for_header = TRUE;

extern FIL *f_hid_bin;

void uart_tx(u8 *data, u16 len)
{
    u16 i;
    
    for(i=0; i<len; i++) {
        uart2_tx(*data++);
    }
}

enum lower_state_t lower_get_state(void)
{
    return lower_state;
}

void lower_set_state(enum lower_state_t state)
{
    lower_state = state;
}

void lower_uart_tx(u8 code, u32 addr, u32 len, u8 *data)
{
    struct boot_pkt_t header;
#if ENABLE_AUDIO_DEVICE == 1
    header.addr_l = addr & 0xff;
    header.addr_h = addr >> 8;
    header.addr_hl = addr >> 16;
    header.addr_hh = addr >> 24;
    header.len_l = len & 0xff;
    header.len_h = len >> 8;
    header.len_hl = len >>16;
    header.len_hh = len >> 24;
    header.code = code;
#else
    header.addr_l = addr & 0xff;
    header.addr_h = addr >> 8;
    header.len_l = len & 0xff;
    header.len_h = len >> 8;
    header.code = code;
#endif
    uart_tx((void *)&header, sizeof(struct boot_pkt_t));
    
    if(data) {
        uart_tx(data, len);
    }
}

extern u8 bd_addr[];
void lower_uart_tx_bin(void)
{
    static UINT read_count = 0;
    UINT tmp_addr;
    
    if(lower_tx_buffer == NULL) {
        lower_tx_buffer = mymalloc(SRAMIN, SINGLE_PACKET_SIZE);
    }

    if(current_tx_index == 0) {
        f_lseek(f_hid_bin, 0);
        f_read(f_hid_bin, lower_tx_buffer, SINGLE_PACKET_SIZE, &read_count);
    }

    if((current_tx_index <= bd_addr_offset[device_type][0])
        &&((current_tx_index+read_count) > bd_addr_offset[device_type][0])) {
        if((current_tx_index+read_count) > bd_addr_offset[device_type][1]) {
            memcpy(lower_tx_buffer+(bd_addr_offset[device_type][0]-current_tx_index), bd_addr, 6);
        }
        else {
            memcpy(lower_tx_buffer+(bd_addr_offset[device_type][0]-current_tx_index), bd_addr, 5+(current_tx_index+read_count)-bd_addr_offset[device_type][1]);
        }
    }
    else if(current_tx_index <= bd_addr_offset[device_type][1]) {
        memcpy(lower_tx_buffer, bd_addr+(current_tx_index+5-bd_addr_offset[device_type][1]), bd_addr_offset[device_type][1]-current_tx_index+1);
    }

    lower_state = LOWER_WRITE;
    lower_uart_tx(CODE_WRITE, current_tx_index, read_count, lower_tx_buffer);
    current_tx_index += read_count;
    if(current_tx_index >= f_hid_bin->fsize) {
        current_tx_index = 0;
        lower_uart_finish_callback = lower_uart_tx_disconnect;
        tmp_addr = bd_addr[0] | (bd_addr[1]<<8) | (bd_addr[2]<<16);
        tmp_addr += 1;
        bd_addr[0] = tmp_addr & 0xFF;
        bd_addr[1] = (tmp_addr>>8) & 0xFF;
        bd_addr[2] = (tmp_addr>>16) & 0xFF;
    }
    else {
        f_read(f_hid_bin, lower_tx_buffer, SINGLE_PACKET_SIZE, &read_count);
    }
}

void lower_uart_tx_disconnect(void)
{
    lower_state = LOWER_DISCONNECTING;
    lower_uart_finish_callback = lower_uart_reset;
    lower_uart_tx(CODE_DISCONN, 0, 0, NULL);
}

u8 boot_req_receive_buffer[sizeof(boot_conn_req)];
void lower_uart_rx_idle(u8 data)
{
    boot_req_receive_buffer[current_received_count++] = data;
    if(current_received_count == sizeof(boot_conn_req)) {
        if((memcmp(boot_req_receive_buffer, boot_conn_req, sizeof(boot_conn_req))==0)
            &&(f_hid_bin != NULL)) {
            lower_state = LOWER_DOING_HAND_SHAKE;
            current_received_count = 0;
            uart_tx((u8 *)boot_conn_ack, sizeof(boot_conn_ack));
        }
        else {
            memcpy(boot_req_receive_buffer, boot_req_receive_buffer+1, sizeof(boot_conn_req)-1);
            current_received_count--;
        }
    }
#if 0
    if(data == boot_conn_req[current_received_count]) {
        current_received_count++;
        if((current_received_count == sizeof(boot_conn_req))
            &&(f_hid_bin != NULL)) {
            lower_state = LOWER_DOING_HAND_SHAKE;
            current_received_count = 0;
            uart_tx((u8 *)boot_conn_ack, sizeof(boot_conn_ack));
        }
    }
    else {
        current_received_count = 0;
    }
#endif
}

void lower_uart_rx_hand_shake(u8 data)
{
    if(data == boot_conn_success[current_received_count]) {
        current_received_count++;
        if(current_received_count == sizeof(boot_conn_success)) {
            LED1 = LED_OFF;
            LED2 = LED_ON;
            lower_state = LOWER_CONNECTED;
            current_received_count = 0;
            if(lower_uart_finish_callback) {
                lower_uart_finish_callback();
            }
        }
    }
    else {
        lower_state = LOWER_IDLE;
        current_received_count = 0;
    }
}

void lower_uart_rx_ack(u8 data, u8 expect_code)
{
    static struct boot_pkt_t header;
    
    if(current_received_count < sizeof(struct boot_pkt_t)) {
        *((u8 *)(&header)+current_received_count) = data;
        current_received_count++;
    }

    if(current_received_count == sizeof(struct boot_pkt_t)) {
        if(header.code == expect_code) {
            current_received_count = 0;
            if(expect_code == CODE_READ_ACK) {
                read_wait_for_header = FALSE;
            }
            else if(lower_uart_finish_callback) {
                lower_uart_finish_callback();
            }
        }
        else {
            if(lower_state != LOWER_DISCONNECTING) {
                lower_uart_tx_disconnect();
            }
            else {
                lower_uart_reset();
            }
        }
    }
}

void lower_uart_rx_content(u8 data)
{
    if(lower_rx_buffer) {
        lower_rx_buffer[current_received_count] = data;
    }
    current_received_count++;
    if(current_received_count == read_total_count) {
        if(lower_uart_finish_callback) {
            lower_uart_finish_callback();
        }
    }
}

void lower_uart_reset(void)
{
    lower_state = LOWER_DISABLE;
    lower_uart_finish_callback = lower_uart_tx_bin;
    current_received_count = 0;
    current_tx_index = 0;
    LED1 = LED_ON;
    LED2 = LED_OFF;
}

void low_fix_trans_bug_read_byte_ack()
{
    current_received_count = 0;
    lower_uart_finish_callback = lower_uart_tx_bin;
    lower_uart_tx_bin();    
}

void low_fix_trans_bug_read_byte(void)
{
    lower_state = LOWER_READ;
    lower_uart_finish_callback = low_fix_trans_bug_read_byte_ack;
    read_total_count = 1;
    lower_uart_tx(CODE_READ, 0, 1, 0);
}

void low_fix_trans_bug(void)
{
#if ENABLE_AUDIO_DEVICE == 1
    delay_ms(100);
    lower_state = LOWER_DOING_HAND_SHAKE;
    current_received_count = 0;
    read_wait_for_header = TRUE;
    uart_tx((u8 *)boot_conn_ack, sizeof(boot_conn_ack));
    delay_ms(100);
    low_fix_trans_bug_read_byte();
#endif
}

void lower_uart_rx(u8 data)
{
    switch(lower_state) {
        case LOWER_DISABLE:
            break;
        case LOWER_IDLE:
            lower_uart_rx_idle(data);
            break;
        case LOWER_DOING_HAND_SHAKE:
            lower_uart_rx_hand_shake(data);
            break;
        case LOWER_READ:
            if(read_wait_for_header) {
                lower_uart_rx_ack(data, CODE_READ_ACK);
            }
            else {
                lower_uart_rx_content(data);
            }
            break;
        case LOWER_WRITE:
            lower_uart_rx_ack(data, CODE_WRITE_ACK);
            break;
        case LOWER_DISCONNECTING:
            lower_uart_rx_ack(data, CODE_DISCONN_ACK);
    }
}

