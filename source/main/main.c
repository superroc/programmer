#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	 	 
#include "timer.h"
#include "flash.h"
#include "sram.h"
#include "malloc.h"
#include "string.h"
#include "mmc_sd.h"
#include "mass_mal.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "memory.h"	    
#include "usb_bot.h" 
#include "ff.h"
#include "exfuns.h"
#include "exti.h"
#include "power_control.h"
#include "lower.h"

u8 bd_addr[6];
FIL *f_hid_bin;

//ALIENTEK战舰STM32开发板实验50
//USB读卡器 实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司 

//设置USB 连接/断线
//enable:0,断开
//       1,允许连接	   
void usb_port_set(u8 enable)
{
    RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
    if(enable) {
        _SetCNTR(_GetCNTR()&(~(1<<1)));//退出断电模式
    }
    else {	  
        _SetCNTR(_GetCNTR()|(1<<1));  // 断电模式
        GPIOA->CRH&=0XFFF00FFF;
        GPIOA->CRH|=0X00033000;
        PAout(12)=0;	    		  
    }
}

int main(void)
{						  
    u8 offline_cnt=0;
    u8 tct=0;
    u8 USB_STA;
    u8 Divece_STA;
    DIR root_dir;
    static FILINFO file_info;
    UINT file_num;
    u8 res;
    FIL *f_bdaddr_bin;

    Stm32_Clock_Init(6);//系统时钟设置
    delay_init(72);		//延时初始化
    uart_init(36,115200); //串口1初始化
    LED_Init();         //LED初始化
    EXTIX_Init();
    power_control_pin_init();

    LED1 = LED_OFF;
    LED2 = LED_OFF;

    SPI_Flash_Init();
    if(SD_Initialize())	{//检测SD卡错误
    }
    else {//SD 卡正常														  
        Mass_Memory_Size[0]=(long long)SD_GetSectorCount()*512;//得到SD卡容量（字节），当SD卡容量超过4G的时候,需要用到两个u32来表示
        Mass_Block_Size[0] =512;//因为我们在Init里面设置了SD卡的操作字节为512个,所以这里一定是512个字节.
        Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
    }
    

    if(SPI_FLASH_TYPE!=W25Q64) {//检测SD卡错误
    }
    else {//SPI FLASH 正常															  
        Mass_Memory_Size[1]=1024*1024*6;//前6M字节
        Mass_Block_Size[1] =512;//因为我们在Init里面设置了SD卡的操作字节为512个,所以这里一定是512个字节.
        Mass_Block_Count[1]=Mass_Memory_Size[1]/Mass_Block_Size[1];
    }

    if(SPI_FLASH_TYPE!=0xEF12) {//检测SD卡错误
    }
    else {//SPI FLASH 正常															  
        Mass_Memory_Size[1]=1024*512;//512K
        Mass_Block_Size[1] =512;//因为我们在Init里面设置了SD卡的操作字节为512个,所以这里一定是512个字节.
        Mass_Block_Count[1]=Mass_Memory_Size[1]/Mass_Block_Size[1];
    }

    delay_ms(1000);
    //printf("error.\r\n");
		
    mem_init(SRAMIN);
    exfuns_init();
    f_mount(0, fs[0]);
    f_mount(1, fs[1]);
    
    delay_ms(1800);
    usb_port_set(0); 	//USB先断开
    delay_ms(300);
    usb_port_set(1);	//USB再次连接
    //USB配置
    USB_Interrupts_Config();
    Set_USBClock();
    USB_Init();
    delay_ms(1800);

    if(f_opendir(&root_dir, "1:")==FR_OK) {
        file_info.lfsize = _MAX_LFN*2+1;
	    file_info.lfname = mymalloc(SRAMIN, file_info.lfsize);
        while(1) {
            res = f_readdir(&root_dir, &file_info);
            if((res != FR_OK)
                ||(file_info.fname[0] == 0)) {
                break;
            }
        }
    }

#if 1
    f_hid_bin = (FIL *)mymalloc(SRAMIN,sizeof(FIL));
    if(f_open(f_hid_bin, (const TCHAR*)"1:/bd_addr.bin", FA_READ) == FR_OK) {
        f_read(f_hid_bin, bd_addr, 6, &file_num);
        f_close(f_hid_bin);
    }

    if(f_open(f_hid_bin, (const TCHAR*)"1:/hid.bin", FA_READ) == FR_OK) {
        
    }
    else {
        myfree(SRAMIN, (void *)f_hid_bin);
        f_hid_bin = NULL;
    }
#endif

    LED1 = LED_ON;
    LED2 = LED_OFF;

    while(1) {
        if(key1_pressed) {
            key1_pressed = 0;
            if(lower_get_state() == LOWER_DISABLE) {
#if 0
                power_control_pin_off();
                LED1 = LED_OFF;
                LED2 = LED_OFF;
                delay_ms(1000);
                delay_ms(1000);
                lower_set_state(LOWER_IDLE);
                power_control_pin_on();
#else
                LED1 = LED_OFF;
                LED2 = LED_OFF;
                lower_set_state(LOWER_IDLE);
                power_control_pin_off();
                delay_ms(10);
                power_control_pin_on();
#endif
            }
        }

        if(key2_pressed) {
            key2_pressed = 0;
            if(lower_get_state() != LOWER_DISABLE) {
                lower_uart_reset();
            }
            LED1 = LED_OFF;
            LED2 = LED_OFF;
            lower_set_state(LOWER_IDLE);
            power_control_pin_off();
            delay_ms(10);
            power_control_pin_on();
            low_fix_trans_bug();
        }
        
        if(key3_pressed) {
            key3_pressed = 0;
            if(lower_get_state() == LOWER_DISABLE) {
                if(f_hid_bin) {
                    f_close(f_hid_bin);
                }
                f_bdaddr_bin = (FIL *)mymalloc(SRAMIN,sizeof(FIL));
                if(f_open(f_bdaddr_bin, (const TCHAR*)"1:/bd_addr.bin", FA_WRITE) == FR_OK) {
                    f_write(f_bdaddr_bin, bd_addr, 6, &file_num);
                    f_close(f_bdaddr_bin);
                }
                myfree(SRAMIN, f_bdaddr_bin);
                if(f_hid_bin) {
                    f_open(f_hid_bin, (const TCHAR*)"1:/hid.bin", FA_READ);
                }
            }
        }
        delay_ms(1);				  
        if(USB_STA!=USB_STATUS_REG) {//状态改变了 
            if(USB_STATUS_REG&0x01) {//正在写
                //提示USB正在写入数据	 
            }
            if(USB_STATUS_REG&0x02) {//正在读
                //提示USB正在读出数据  		 
            }	 										  
            if(USB_STATUS_REG&0x04) {//提示写入错误
            }
            else {//清除显示	  
            }
            if(USB_STATUS_REG&0x08) {//提示读出错误
            }
            else {//清除显示    
            }
            USB_STA=USB_STATUS_REG;//记录最后的状态
        }
        if(Divece_STA!=bDeviceState) 
        {
            if(bDeviceState==CONFIGURED) {//提示USB连接已经建立
            }
            else {//提示USB被拔出了
            }
            Divece_STA=bDeviceState;
        }
        tct++;
        if(tct==200)
        {
            tct=0;
            //LED0=!LED0;//提示系统在运行
            if(USB_STATUS_REG&0x10) {
                offline_cnt=0;//USB连接了,则清除offline计数器
                bDeviceState=CONFIGURED;
            }else {//没有得到轮询 
                offline_cnt++;  
                if(offline_cnt>10)
                    bDeviceState=UNCONNECTED;//2s内没收到在线标记,代表USB被拔出了
            }
            USB_STATUS_REG=0;
        }
    };  										    			    
}

