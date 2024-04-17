#include "pll.h"

inline int mod(const int dividend, const int divisor){
   int r = dividend % divisor;
   if (r < 0) r += divisor;
   return r;
}

inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
        float intval = (float)round_int(x / y);
#else
        float intval = nearbyintf(x / y);
#endif
        return x - intval * y;
}
		
inline float fmodf_pos(float x, float y) {
  float res = wrap_pm(x, y);
  if (res < 0) res += y;
	return res;
}
		
inline float wrap_pm_pi(float x) {
        return wrap_pm(x, 2 * PI);
}



void MeasurePosVel(uint16_t cnt,struct PosVel *pv)
{

	static uint16_t shadow_count_;
	static uint16_t count_in_cpr_;
	static float interpolation_;
	static float pos_cpr_counts_;
	static float pos_estimate_counts_;
	static float vel_estimate_counts_;
	static float delta_pos_cpr_counts_;
	static float pll_kp_ = 2*1000;
	static float pll_ki_ ;
	static float pos_estimate_,vel_estimate_;
	static float pos_circular_;
	static float phase_,phase_vel_;
	pll_ki_ =0.25f*pll_kp_*pll_kp_;
	
	int delta_enc = 0;
	
	delta_enc = cnt - (int16_t)shadow_count_;
	
	shadow_count_ += delta_enc;
	count_in_cpr_ += delta_enc;
	count_in_cpr_  = mod(count_in_cpr_,16384);
	float pos_cpr_counts_last_ = pos_cpr_counts_;
	
	pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
  pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
	
  float delta_pos_counts      = (float)(shadow_count_ - floor(pos_estimate_counts_));
  float delta_pos_cpr_counts  = (float)(count_in_cpr_ - floor(pos_cpr_counts_));
  delta_pos_cpr_counts        = wrap_pm(delta_pos_cpr_counts,(float)(16384.f));	
	
	delta_pos_cpr_counts_+= 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_);
	
	pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
  pos_cpr_counts_      += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
  pos_cpr_counts_      =  fmodf_pos(pos_cpr_counts_,16384.f); 			//确保pos_cpr_counts_保持在CPR范围内。
  vel_estimate_counts_ += current_meas_period*pll_ki_*delta_pos_cpr_counts;
	
  // 检查速度估计值是否接近零。
  bool snap_to_zero_vel = false;
  float abs_vel_estimate_counts_;
	arm_abs_f32(&vel_estimate_counts_,&abs_vel_estimate_counts_,1);

  if(abs_vel_estimate_counts_ < 0.5f*current_meas_period*pll_ki_){
      vel_estimate_counts_ = 0.0f;
      snap_to_zero_vel = true;
  }
	
  pos_estimate_ = pos_estimate_counts_ / (float)16384;
  float vel_estimate = vel_estimate_counts_ / (float)16384;
  vel_estimate_ = vel_estimate;	
	
	float pos_circular = pos_circular_;
  pos_circular+= wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last_)/(16384.f),1.0f);
  pos_circular_=pos_circular;
	
	int32_t corrected_enc = count_in_cpr_ ;
	
  if (snap_to_zero_vel || !false) {
      interpolation_ = 0.5f; // 将插值因子设置为中点。
  } else if (delta_enc > 0) {
      interpolation_ = 0.0f; // 将插值因子设置为0。
  } else if (delta_enc < 0) {
      interpolation_ = 1.0f; // 将插值因子设置为1。
  } else {
      // 更新插值因子。
      interpolation_ += current_meas_period * vel_estimate_counts_;
      if (interpolation_ > 1.0f) interpolation_ = 1.0f;
      if (interpolation_ < 0.0f) interpolation_ = 0.0f;
  }

    // 执行线性插值以获取插值后的编码器计数。 
  float interpolated_enc = corrected_enc + interpolation_;
    // 计算电气角度。
  float elec_rad_per_enc = 14 * 2 * PI * (1.0f / (float)(16384.f));
  float ph = elec_rad_per_enc * (interpolated_enc);

  // 将电气角度限制在-pi到pi范围内，并调整方向。
  phase_ = wrap_pm_pi(ph) * -1.f;
  float vel = (2*PI) * vel_estimate_* 14;
  phase_vel_ = vel;
	
	pv->pos_in_one = pos_estimate_;
	pv->vel        = vel_estimate_;
	pv->radio      = phase_;
	pv->phase_vel  = phase_vel_;
	pv->pos        = pos_circular;

}