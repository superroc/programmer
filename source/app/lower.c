#include <string.h>
#include "usart.h"
#include "ff.h"
#include "malloc.h"
#include "stm32f10x_type.h"
#include "led.h"
#include "lower.h"

//audio device opcode
#define CODE_EEPROM_READ        0x01
#define CODE_EEPROM_WRITE       0x02
#define CODE_READMEM            0x03
#define CODE_WRITEMEM           0x04
#define CODE_FLASH_READ         0x05
#define CODE_FLASH_WRITE        0x06
#define CODE_FLASH_FAST_READ    0x07
#define CODE_FLASH_ERASE        0x08
#define CODE_DISCONN            0x09
#define CODE_REMAP              0x0a
#define BAUD_RATE_CHANGE        0x0b
#define RF_TONE                 0x0c
#define RF_CLBR                 0x0d

#define CODE_READ_ACK           0x10
#define CODE_WRITE_ACK          0x20
#define CODE_READMEM_ACK        0x30
#define CODE_WRITEMEM_ACK       0x40
#define CODE_ERASE_ACK          0x80
#define CODE_DISCONN_ACK        0x90
#define BAUD_RATE_ACK           0xa0
#define RF_TONE_ACK             0xb0
#define RF_CLBR_ACK             0xc0

//hid device opcode
#define CODE_HID_READ           0x01
#define CODE_HID_READ_ACK       0x02
#define CODE_HID_WRITE          0x03
#define CODE_HID_WRITE_ACK      0x04
#define CODE_HID_DISCONN        0x05
#define CODE_HID_DISCONN_ACK    0x06
#define CODE_HID_READMEM        0x07
#define CODE_HID_READMEM_ACK    0x08
#define CODE_HID_WRITEMEM       0x09
#define CODE_HID_WRITEMEM_ACK   0x0a
#define CODE_HID_ERROR          0x0e//not used
#define CODE_HID_NO_EEPROM_ACK  0x0f//

u8 code_write_opcode;
u8 code_disconn_opcode;
u8 code_read_ack_opcode;
u8 code_read_opcode;
u8 code_write_ack_opcode;
u8 code_disconn_ack_opcode;

#define SINGLE_PACKET_SIZE      256

void uart_tx(u8 *data, u16 len);
void uart2_tx(u8 c);

__packed struct boot_pkt_audio_t
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

__packed struct boot_pkt_hid_t
{
    u32 addr_h: 4;
    u32 code: 4;
    u32 addr_l: 8;
    u32 len_h: 8;
    u32 len_l: 8;
};

union boot_pkt_t {
    struct boot_pkt_audio_t audio_header;
    struct boot_pkt_hid_t hid_header;
};

__packed struct boot_mem_t{
	u32  address;
	u8    value;
};

const u8 boot_conn_req[] = {0x04, 0x0f, 0x04, 0x00, 0x01, 0x00, 0x00};
const u8 boot_conn_audio_ack[] = {'b' + 100, 'l' + 100, 'u' + 100, 'e' + 100, 's' + 100, 'p' + 100, 'e' + 100, 'a' + 100, 'k' + 100};
const u8 boot_conn_hid_ack[] = {'z' + 100, 'c' + 100, 'a' + 100, 'n' + 100, 'z' + 100, 'c' + 100, 'a' + 100, 'n' + 100, '0' + 100};
const u8 boot_conn_success[] = {0x04, 0x0f, 0x04, 0x01, 0x01, 0x00, 0x00};
const u8 bd_addr_offset[MAX_DEVICE_TYPE][2] = {{0x0e, 0x13}, {0x0e, 0x13}, {0x89, 0x8e}, {0x85, 0x8a}};

//�����Ѿ����յ�����Ӧheader���߶�ȡ�ָ�data�ĳ���
u32 current_received_count = 0;
//��λ����Ϊ��дflash�ķ���ˣ������һ�ζ�д����ִ�еĶ����ɴ˻ص�����ִ��
void (*lower_uart_finish_callback)() = lower_uart_tx_bin;
//�Ѿ����͵�bin���ݳ���
u32 current_tx_index = 0;
//������һ��Ҫ���͵�����֡�ĵ�ַ
u8 *lower_tx_buffer = NULL;
//Ӧ�ò�Ҫ�������������ݳ���
u32 read_total_count = 0;
//�����������ݱ��浽ʲô��ַ
u8 *lower_rx_buffer = 0;
//��ǰ���յ��Ƿ�Ϊ������ack��ͷ��(�ڶ�����ʱ��Ч)
u8 read_wait_for_header = TRUE;

enum lower_state_t lower_state = LOWER_DISABLE;
enum device_type_t device_type = AUDIO_DEVICE_FLASH;

extern FIL *f_bin;

void uart_tx(u8 *data, u16 len)
{
    u16 i;
    
    for(i=0; i<len; i++) {
        uart2_tx(*data++);
    }
}

void lower_assign_opcode()
{
    if(device_type == AUDIO_DEVICE_EEPROM) {
        code_write_opcode = CODE_EEPROM_WRITE;
        code_disconn_opcode = CODE_DISCONN;
        code_read_ack_opcode = CODE_READ_ACK;
        code_read_opcode = CODE_EEPROM_READ;
        code_write_ack_opcode = CODE_WRITE_ACK;
        code_disconn_ack_opcode = CODE_DISCONN_ACK;
    }
    else if(device_type == AUDIO_DEVICE_FLASH) {
        code_write_opcode = CODE_FLASH_WRITE;
        code_disconn_opcode = CODE_DISCONN;
        code_read_ack_opcode = CODE_READ_ACK;
        code_read_opcode = CODE_FLASH_READ;
        code_write_ack_opcode = CODE_WRITE_ACK;
        code_disconn_ack_opcode = CODE_DISCONN_ACK;
    }
    else if((device_type == HID_D_F_DEVICE)
        ||(device_type == HID_E_G_DEVICE)) {
        code_write_opcode = CODE_HID_WRITE;
        code_disconn_opcode = CODE_HID_DISCONN;
        code_read_ack_opcode = CODE_HID_READ_ACK;
        code_read_opcode = CODE_HID_READ;
        code_write_ack_opcode = CODE_HID_WRITE_ACK;
        code_disconn_ack_opcode = CODE_HID_DISCONN_ACK;
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
    union boot_pkt_t header;

    if((device_type == AUDIO_DEVICE_EEPROM)
        ||(device_type == AUDIO_DEVICE_FLASH)) {
        header.audio_header.addr_l = addr & 0xff;
        header.audio_header.addr_h = addr >> 8;
        header.audio_header.addr_hl = addr >> 16;
        header.audio_header.addr_hh = addr >> 24;
        header.audio_header.len_l = len & 0xff;
        header.audio_header.len_h = len >> 8;
        header.audio_header.len_hl = len >>16;
        header.audio_header.len_hh = len >> 24;
        header.audio_header.code = code;

        uart_tx((void *)&header, sizeof(struct boot_pkt_audio_t));
    }
    else if((device_type == HID_D_F_DEVICE)
        ||(device_type == HID_E_G_DEVICE)) {
        header.hid_header.addr_l = addr & 0xff;
        header.hid_header.addr_h = addr >> 8;
        header.hid_header.len_l = len & 0xff;
        header.hid_header.len_h = len >> 8;
        header.hid_header.code = code;

        uart_tx((void *)&header, sizeof(struct boot_pkt_hid_t));
    }    
    else {
        return;
    }
    
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
        f_lseek(f_bin, 0);
        f_read(f_bin, lower_tx_buffer, SINGLE_PACKET_SIZE, &read_count);
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
    lower_uart_tx(code_write_opcode, current_tx_index, read_count, lower_tx_buffer);
    current_tx_index += read_count;
    if(current_tx_index >= f_bin->fsize) {
        current_tx_index = 0;
        lower_uart_finish_callback = lower_uart_tx_disconnect;
        tmp_addr = bd_addr[0] | (bd_addr[1]<<8) | (bd_addr[2]<<16);
        tmp_addr += 1;
        bd_addr[0] = tmp_addr & 0xFF;
        bd_addr[1] = (tmp_addr>>8) & 0xFF;
        bd_addr[2] = (tmp_addr>>16) & 0xFF;
    }
    else {
        f_read(f_bin, lower_tx_buffer, SINGLE_PACKET_SIZE, &read_count);
    }
}

void lower_uart_tx_disconnect(void)
{
    lower_state = LOWER_DISCONNECTING;
    lower_uart_finish_callback = lower_uart_reset;
    lower_uart_tx(code_disconn_opcode, 0, 0, NULL);
}

void lower_uart_tx_boot_ack(void)
{
    if((device_type == AUDIO_DEVICE_EEPROM)
        ||(device_type == AUDIO_DEVICE_FLASH)) {
        uart_tx((u8 *)boot_conn_audio_ack, sizeof(boot_conn_audio_ack));
    }
    else if((device_type == HID_D_F_DEVICE)
        ||(device_type == HID_E_G_DEVICE)) {
        uart_tx((u8 *)boot_conn_hid_ack, sizeof(boot_conn_hid_ack));
    }
}

/*
 * ѭ�����մ������ݣ��ж��Ƿ��յ���һ��������boot_conn_req
 */
u8 boot_req_receive_buffer[sizeof(boot_conn_req)];
void lower_uart_rx_idle(u8 data)
{
    boot_req_receive_buffer[current_received_count++] = data;
    if(current_received_count == sizeof(boot_conn_req)) {
        if((memcmp(boot_req_receive_buffer, boot_conn_req, sizeof(boot_conn_req))==0)
            &&(f_bin != NULL)) {
            lower_state = LOWER_DOING_HAND_SHAKE;
            current_received_count = 0;
            lower_uart_tx_boot_ack();
        }
        else {
            memcpy(boot_req_receive_buffer, boot_req_receive_buffer+1, sizeof(boot_conn_req)-1);
            current_received_count--;
        }
    }
}

/*
 * ������boot_conn_ack�󣬵ȴ�����boot_conn_success�ź�
 */
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

/*
 * ��д����оƬ��������ָ���ͨ���ú�����ȡоƬ����Ӧ
 */
void lower_uart_rx_ack(u8 data, u8 expect_code)
{
    static union boot_pkt_t header;
    u8 header_size, recv_code;

    if(current_received_count < header_size) {
        *((u8 *)(&header)+current_received_count) = data;
        current_received_count++;
    }

    if((device_type == AUDIO_DEVICE_EEPROM)
        ||(device_type == AUDIO_DEVICE_FLASH)) {
        header_size = sizeof(struct boot_pkt_audio_t);
        recv_code = header.audio_header.code;
    }
    else if((device_type == HID_D_F_DEVICE)
        ||(device_type == HID_E_G_DEVICE)) {
        header_size = sizeof(struct boot_pkt_hid_t);
        recv_code = header.hid_header.code;
    }
    else {
        return;
    }

    //�ȴ����յ��㹻������
    if(current_received_count == header_size) {
        current_received_count = 0;
        if(recv_code == expect_code) {
            //�յ������Ľ�����Զ�������һ�����⴦��
            if(expect_code == code_read_ack_opcode) {
                read_wait_for_header = FALSE;
            }
            else if(lower_uart_finish_callback) {
                lower_uart_finish_callback();
            }
        }
        else {
            //�յ��˷Ƿ������ݣ��Ͽ����ӻ��߸�λ
            if(lower_state != LOWER_DISCONNECTING) {
                lower_uart_tx_disconnect();
            }
            else {
                lower_uart_reset();
            }
        }
    }
}

/*
 * ����Ҫ��ȡ�����ݵ����ݣ����Ӧ����lower_uart_rx_ack�н��ն�����ʱоƬ���ص�header
 */
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

/*
 * ��״̬���ָ�����ʼ״̬
 */
void lower_uart_reset(void)
{
    lower_state = LOWER_DISABLE;
    lower_uart_finish_callback = lower_uart_tx_bin;
    current_received_count = 0;
    current_tx_index = 0;
    LED1 = LED_ON;
    LED2 = LED_OFF;
}

/*
 * �յ���3180�Զ���������Ӧ��֮�������������ģʽ
 */
void low_fix_trans_bug_read_byte_ack()
{
    current_received_count = 0;
    lower_uart_finish_callback = lower_uart_tx_bin;
    lower_uart_tx_bin();    
}

/*
 * �����ȡһ��byte�Ĳ��������ڻ���3180�������л�����
 */
void low_fix_trans_bug_read_byte(void)
{
    lower_state = LOWER_READ;
    lower_uart_finish_callback = low_fix_trans_bug_read_byte_ack;
    read_total_count = 1;
    lower_uart_tx(code_read_opcode, 0, 1, 0);
}

/*
 * ��������3180���flashʱ��bug:3180�Ĵ��ڷ����ź��߸��õ���spi���������ϣ�
 * ���Ǵ�������bug����û���л����������������װ���յ���оƬ��������������
 * �ź�(boot_conn_req)��ֱ�ӻظ���boot_conn_ack���ݣ����Һ���Ӧ���յ�����
 * Ӧ�ź�boot_conn_successҲ��װ�յ���֮��ͨ������һ��������������оƬ����
 * �Ź����л������������յ��������Ľ��֮�����������д�������
 */
void low_fix_trans_bug(void)
{
    if(device_type == AUDIO_DEVICE_FLASH) {
        delay_ms(100);
        lower_state = LOWER_DOING_HAND_SHAKE;
        current_received_count = 0;
        read_wait_for_header = TRUE;
        lower_uart_tx_boot_ack();
        delay_ms(100);
        low_fix_trans_bug_read_byte();
    }
}

/*
 * �����ڽ������ݵ������
 */
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
                lower_uart_rx_ack(data, code_read_ack_opcode);
            }
            else {
                lower_uart_rx_content(data);
            }
            break;
        case LOWER_WRITE:
            lower_uart_rx_ack(data, code_write_ack_opcode);
            break;
        case LOWER_DISCONNECTING:
            lower_uart_rx_ack(data, code_disconn_ack_opcode);
    }
}

