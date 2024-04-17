#include "FocCtr.h"

struct PosVel pos_vel;


float Ksp;


float cos_value,sin_value;//一次PWM周期只算一次

float Ualpha,Ubata;

float Iq_Target=0,Id_Target=0;

float Id_ref ,Iq_ref;


float Uq=0,Ud=0.8; 
float Ia,Ib,Ic;






void Park_change_Contrary()//Park逆变换
{
	cos_value=arm_cos_f32(pos_vel.radio);
	sin_value=arm_sin_f32(pos_vel.radio);
	Ksp=7274.613f/Vbus;
	
	Ualpha=Ud*cos_value-Uq*sin_value;
	Ubata= Uq*cos_value+Ud*sin_value;
}


void Park_change(float I_alpha,float I_beta)//Park变换
{
		Id_ref =  I_alpha*cos_value + I_beta*sin_value;
		Iq_ref = -I_alpha*sin_value + I_beta*cos_value;
}


float I_ref_ahp,I_ref_beta;
void Klark_change()//克拉克变换
{
	
	Ia=(((uint16_t)hadc1.Instance->JDR1 - 2048) / 4096.f) / 20.f / 0.005f;
	Ib=(((uint16_t)hadc1.Instance->JDR2 - 2048) / 4096.f) / 20.f / 0.005f;
	Ic=(((uint16_t)hadc1.Instance->JDR3 - 2048) / 4096.f) / 20.f / 0.005f;
	I_ref_ahp=Ia-0.5f*(Ib+Ic);
	I_ref_beta=0.5774f*(Ib - Ic);	
	
	Park_change(I_ref_ahp,I_ref_beta);
}




float ta,tb,tc;
float X,Y,Z;
uint8_t sector;
void sector_judg()
{
	float u1,u2,u3;
	float t0,t1,t2,t3,t4,t5,t6,t7;
	u1=Ubata;
	u2=squrt3*Ualpha/2.f-Ubata/2.f;
	u3=-squrt3*Ualpha/2.f-Ubata/2.f;
		
	sector=(u1>0.0)+((u2>0.0)<<1)+((u3>0.0)<<2);	
	 X=Ksp*Ubata;
	 Y=Ksp*(-(squrt3/2.0f)*Ualpha-0.5f*Ubata);
	 Z=Ksp*((squrt3/2.0f)*Ualpha-0.5f*Ubata);
		

	if(sector==3)//第1扇区
	{
			t4=Z;
			t6=X;		
			t7=(PWM_Max-t4-t6)/2;
			ta=t4+t6+t7;
			tb=t6+t7;
			tc=t7;
	}
	else if(sector==1)//第2扇区
	{	
			t2=-Z;
			t6=-Y;
			t7=(PWM_Max-t2-t6)/2;
			ta=t6+t7;
			tb=t2+t6+t7;
			tc=t7;
	}
	else if(sector==5)//第3扇区
	{	
			t2=X;
			t3=Y;		
			t7=(PWM_Max-t2-t3)/2;
			ta=t7;
			tb=t2+t3+t7;
			tc=t3+t7;
	}
	else if(sector==4)//第4扇区
	{	
			t1=-X;
			t3=-Z;	
			t7=(PWM_Max-t1-t3)/2;
			ta=t7;
			tb=t3+t7;
			tc=t1+t3+t7;
	}
	else if(sector==6)//第5扇区
	{	
			t1=Y;
			t5=Z;	
			t7=(PWM_Max-t1-t5)/2;
			ta=t5+t7;
			tb=t7;
			tc=t1+t5+t7;
	}
	else if(sector==2)//第6扇区
	{	
			t4=-Y;
			t5=-X;	
			t7=(PWM_Max-t4-t5)/2;
			ta=t4+t5+t7;
			tb=t7;
			tc=t5+t7;
	}
	else 
	{
		ta=0;
		tb=0;
		tc=0;
	
	}

}

PID Iq_current,Id_current;
float Iq_current_loop()
{
		Iq_current.error=Iq_Target-Iq_ref;
		Iq_current.error_sum+=Iq_current.error;
		if(Iq_current.error_sum>20)
		{
				Iq_current.error_sum=20;
		}
		else if(Iq_current.error_sum<-20)
		{
				Iq_current.error_sum=-20;
		}
	  Iq_current.out = Iq_current.error * Iq_current.P + Iq_current.error_sum * Iq_current.I;
		return Iq_current.out;
}

float Id_current_loop()
{
		Id_current.error=Id_Target-Id_ref;
		Id_current.error_sum+=Id_current.error;
		if(Id_current.error_sum>5)
		{
				Id_current.error_sum=5;
		}
		else if(Id_current.error_sum<-5)
		{
				Id_current.error_sum=-5;
		}
	  Id_current.out = Id_current.error * Id_current.P + Id_current.error_sum * Id_current.I;
		
		return Id_current.out;
}

PID Speed_ctl;

float target_speed=5; 
float speed,speed_last;
float  speed_loop()
{
	
	Speed_ctl.error = target_speed-pos_vel.vel;
	Speed_ctl.error_sum += Speed_ctl.error;
	Speed_ctl.out=Speed_ctl.error * Speed_ctl.P + Speed_ctl.error_sum*Speed_ctl.I;
	return Speed_ctl.out;

}