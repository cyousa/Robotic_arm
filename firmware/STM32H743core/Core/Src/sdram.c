#include "sdram.h"

void delay_us(uint16_t us)
{
		uint16_t times=0;
		while(times<us)
		{
			TIM6->CNT=0;
			while (TIM6->CNT<240)
			{
		
			}
			times++;
		}
	
}

uint8_t sdram_send_cmd(uint32_t cmd , uint32_t refresh , uint32_t regval)
{

	
	FMC_SDRAM_CommandTypeDef command;
	
	command.CommandMode             =  cmd;
	
	command.CommandTarget           =   FMC_SDRAM_CMD_TARGET_BANK1;//目标储存区，次bank非sdram的bank要注意咯
	
	command.AutoRefreshNumber       =   refresh;//自刷新次数
	
	command.ModeRegisterDefinition  =   regval;//写入模式寄存器的值
	
	if(HAL_SDRAM_SendCommand(&hsdram1,&command,0x1000)==HAL_OK)
	{
			return 0;
	}
	return 1;

}

void SDRAM_initialization_sequence()
{
			__IO uint32_t mode_reg=0;
	
			sdram_send_cmd(FMC_SDRAM_CMD_CLK_ENABLE,1,0);//使能SDRAM时钟
	
			delay_us(500);
			
			sdram_send_cmd(FMC_SDRAM_CMD_PALL,1,0);//给SDRAM的所以BANK预充电
	
			sdram_send_cmd(FMC_SDRAM_CMD_AUTOREFRESH_MODE,8,0);//设置自刷新次数 8次
								/*设置突发长度：1(可以是1/2/4/8)
								设置突发类型：连续（可以是连续/交错
								设置CAS值：3(可以是2/3)
								设置操作模式：0，标准模式
								设置突发写模式：1，单点访问*/
			mode_reg = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1  				|
														SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL		|
														SDRAM_MODEREG_CAS_LATENCY_3						|
														SDRAM_MODEREG_OPERATING_MODE_STANDARD |
														SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
	
	  	sdram_send_cmd(FMC_SDRAM_CMD_LOAD_MODE,1,mode_reg);

}