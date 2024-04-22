#include "FocCtr.h"
uint8_t isready;
float  T=4096;

float K1, K;
float uq=0,ud=5;
float cos_zeta,sin_zeta;
float V_alpha,V_beta;
void Park_contrary_change()
{


	
	cos_zeta=arm_cos_f32(-pos_vel.radio);
	sin_zeta=arm_sin_f32(-pos_vel.radio);
	
	V_alpha=ud*cos_zeta - uq*sin_zeta;
	V_beta =uq*cos_zeta + ud*sin_zeta;
	

}
uint8_t Now_sector;
void sector_judg()
{
	
	Now_sector=1+((V_alpha>0.0))+((V_beta>0.0)<<1);
	
}


short PWM_A,PWM_B;
void SVPWM()
{  
	float Tx,Ty;
	int Ta,Tb,Tc,Td;
	int T1,T0;
	
		//Angle_Now-=0.02f;
	if(isready==1)
	{
//		K1=2.0f*PI/16384.0f;
//		Angle_Now=K1*(SPI_recive_data-SPI_recive_data_begin)*50.0f+314.159265f*laps;
		
			int16_t enc = SPI_recive_data - SPI_recive_data_begin;
			if(enc<0) enc += 16384;				
			MeasurePosVel(enc,&pos_vel);//Odrive计算当前角度以及速度
		
	}
	

	Park_contrary_change();
	sector_judg();
	K=T/Vbus;
	
	switch(Now_sector)
	{
		case 4://扇区一
			Tx=K*V_alpha;
			Ty=K*V_beta;
			T1=(T-Tx-Ty)/2;
			Ta=Tx+T1;
			Tb=T1;
			Tc=Tx+Ty+T1;
			Td=Tx+T1;
			break;
		case 3://扇区二
			Tx=K*(-V_alpha);
			Ty=K*V_beta;
			T1=(T-Tx-Ty)/2;
			Ta=T1;
			Tb=Tx+T1;
			Tc=Tx+Ty+T1;
			Td=Tx+T1;
			break;
		case 1://扇区三
			Tx=K*(-V_alpha);
			Ty=K*(-V_beta);
			T1=(T-Tx-Ty)/2;
			Ta=T1;
			Tb=Tx+T1;
			Tc=Tx+T1;
			Td=Tx+Ty+T1;
			break;
		case 2://扇区四
			Tx=K*(V_alpha);
			Ty=K*(-V_beta);
			T1=(T-Tx-Ty)/2;
			Ta=Tx+T1;
			Tb=T1;
			Tc=Tx+T1;
			Td=Tx+Ty+T1;
			break;		
	}

	
	PWM_A= (Ta-Tb);
	PWM_B= (Tc-Td);
	

	Set_DAC_Value( PWM_A, PWM_B);
	
}



float Target_angel=0;

float angel_P=1,angel_I=0.000,angel_out,angel_error,angel_error_sum;
void Position_loop()
{
		angel_error=Target_angel-pos_vel.pos;
    angel_out=angel_error*angel_P+angel_error_sum*angel_I;
		angel_error_sum+=angel_error;
	if(angel_error_sum>3000)
	{
	angel_error_sum=3000;
	}
		else if(angel_error_sum<-3000)
	{
	angel_error_sum=-3000;
	}
	
		if(angel_out>10)
	{
		
		angel_out=10;
		
	}
	else if(angel_out<-10)
	{
			angel_out=-10;
	}

}

float Target_speed=-0.01 ;
float Sp_P=0.04,Sp_I=0.0008,Sp_out,Sp_error,Sp_error_sum;//float Sp_P=0.005,Sp_I=0.0000001 
void speed_loop()
{
		Sp_error=(angel_out -pos_vel.vel);
		Sp_out+=Sp_error*Sp_P+Sp_error_sum*Sp_I/1000;
		Sp_error_sum+=Sp_error;
	if(Sp_error_sum>100000)
	{
		Sp_error_sum=100000.0;
	}
	else if(Sp_error_sum<-100000)
	{
	 Sp_error_sum=-100000.0;
	}
	if(Sp_out>7)
	{
		Sp_out=7.0f;
	}
	else if(Sp_out<-7)
	{
		Sp_out=-7.0f;
	}
	
	uq=Sp_out;
		my_data.DATA[4]=0.1;
		my_data.DATA[5]=pos_vel.vel;	
		my_data.DATA[6]=Target_angel;
		my_data.DATA[7]=pos_vel.pos;
		my_data.DATA[8]=Sp_out;


}

