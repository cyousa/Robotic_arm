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
	
	command.CommandTarget           =   FMC_SDRAM_CMD_TARGET_BANK1;//Ŀ�괢��������bank��sdram��bankҪע�⿩
	
	command.AutoRefreshNumber       =   refresh;//��ˢ�´���
	
	command.ModeRegisterDefinition  =   regval;//д��ģʽ�Ĵ�����ֵ
	
	if(HAL_SDRAM_SendCommand(&hsdram1,&command,0x1000)==HAL_OK)
	{
			return 0;
	}
	return 1;

}

void SDRAM_initialization_sequence()
{
			__IO uint32_t mode_reg=0;
	
			sdram_send_cmd(FMC_SDRAM_CMD_CLK_ENABLE,1,0);//ʹ��SDRAMʱ��
	
			delay_us(500);
			
			sdram_send_cmd(FMC_SDRAM_CMD_PALL,1,0);//��SDRAM������BANKԤ���
	
			sdram_send_cmd(FMC_SDRAM_CMD_AUTOREFRESH_MODE,8,0);//������ˢ�´��� 8��
								/*����ͻ�����ȣ�1(������1/2/4/8)
								����ͻ�����ͣ�����������������/����
								����CASֵ��3(������2/3)
								���ò���ģʽ��0����׼ģʽ
								����ͻ��дģʽ��1���������*/
			mode_reg = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1  				|
														SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL		|
														SDRAM_MODEREG_CAS_LATENCY_3						|
														SDRAM_MODEREG_OPERATING_MODE_STANDARD |
														SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
	
	  	sdram_send_cmd(FMC_SDRAM_CMD_LOAD_MODE,1,mode_reg);

}