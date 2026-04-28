
#include "main.h"      


Gimbal_Motor_Handle_t Pitch_Motor;
Gimbal_Motor_Handle_t Yaw_Motor;
Gimbal_Debug_t Gimbal_Debug;

double T = 1.0f;


static void GPID_Init(Gimbal_PID_t *pid, double kp, double ki, double kd, double kk,double i_max, double out_max) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;pid->kk = kk;
    pid->I_Limit = i_max; pid->Max_Output = out_max;
    pid->Error = 0; pid->P_Out = 0; pid->I_Out = 0; pid->D_Out = 0; pid->Output = 0; pid->Last_Measure = 0;
}


static double GPID_Calc_Normal(Gimbal_PID_t *pid, double target, double measure) {
    pid->Error = target - measure;
    pid->target = target;
    
    pid->P_Out = pid->Kp * pid->Error;
     
    pid->I_Out += pid->Ki * pid->Error;
    if (pid->I_Out > pid->I_Limit) pid->I_Out = pid->I_Limit;
    else if (pid->I_Out < -pid->I_Limit) pid->I_Out = -pid->I_Limit;
    
    
    double	 diff = pid->Error - pid->Last_Error;
    pid->D_Out = (pid->Kd * diff*pid->D_filter + pid->last_D_out*(1 - pid->D_filter));
    pid->Last_Error = pid->Error;
    pid->last_D_out = pid->D_Out;

    pid->Output = pid->P_Out + pid->I_Out + pid->D_Out +pid->kk*(pid->target - pid->last_target);
    
    if (pid->Output > pid->Max_Output) pid->Output = pid->Max_Output;
    else if (pid->Output < -pid->Max_Output) pid->Output = -pid->Max_Output;
    pid ->last_target = pid ->target;
    return pid->Output;
}
static double GPID_Calc_1(Gimbal_PID_t *pid, double target, double measure) {
    pid->Error = target - measure;
    pid->target = target;
    
    pid->P_Out = pid->Kp * pid->Error;
    
    pid->I_Out += pid->Ki * pid->Error;
    if (pid->I_Out > pid->I_Limit) pid->I_Out = pid->I_Limit;
    else if (pid->I_Out < -pid->I_Limit) pid->I_Out = -pid->I_Limit;
    
    
    double diff = measure - pid->Last_Measure;
    pid->D_Out = -pid->Kd * diff;
    pid->Last_Measure = measure;
    

    pid->Output += pid->P_Out + pid->I_Out + pid->D_Out +pid->kk*(pid->target - pid->last_target);
    

    if (pid->Output > pid->Max_Output) pid->Output = pid->Max_Output;
    else if (pid->Output < -pid->Max_Output) pid->Output = -pid->Max_Output;
    pid ->last_target = pid ->target;
    return pid->Output;
}

static void Sync_Params(Gimbal_Motor_Handle_t *motor, int axis_id) {
    if (axis_id == 2) { 
        motor->Speed_PID.Kp = Gimbal_Debug.Pitch_Speed_Kp;
        motor->Speed_PID.Ki = Gimbal_Debug.Pitch_Speed_Ki;
        motor->Speed_PID.Kd = Gimbal_Debug.Pitch_Speed_Kd;
        motor->Angle_PID.Kp = Gimbal_Debug.Pitch_Angle_Kp;
        motor->Angle_PID.Ki = Gimbal_Debug.Pitch_Angle_Ki;
        motor->Angle_PID.Kd = Gimbal_Debug.Pitch_Angle_Kd;
    } else { // Yaw
        motor->Speed_PID.Kp = Gimbal_Debug.Yaw_Speed_Kp;
        motor->Speed_PID.Ki = Gimbal_Debug.Yaw_Speed_Ki;
        motor->Speed_PID.Kd = Gimbal_Debug.Yaw_Speed_Kd;
		motor->Speed_PID.kk = Gimbal_Debug.Yaw_Speed_KK;
        motor->Angle_PID.Kp = Gimbal_Debug.Yaw_Angle_Kp;
        motor->Angle_PID.Ki = Gimbal_Debug.Yaw_Angle_Ki;
        motor->Angle_PID.Kd = Gimbal_Debug.Yaw_Angle_Kd;
		motor->Angle_PID.kk = Gimbal_Debug.Yaw_Angle_KK;
        motor->Speed_PID.D_filter = Gimbal_Debug.Yaw_Speed_D_filter;
		
    }
}

void Gimbal_Init(void) {
    
    Gimbal_Debug.Control_Mode = DEBUG_ANGLE_LOOP;
    
    
    Gimbal_Debug.Pitch_Speed_Kp = 15.0; Gimbal_Debug.Pitch_Speed_Ki = 0.06;
    Gimbal_Debug.Pitch_Angle_Kp = 3.0;  Gimbal_Debug.Pitch_Angle_Ki = 0.1;
    Gimbal_Debug.Pitch_Speed_kk = 3.6; Gimbal_Debug.Pitch_Speed_D_filter = 0.3;
	Gimbal_Debug.Pitch_Speed_Kd = 1.5; Gimbal_Debug.Pitch_Angle_Kd = 0.00;
	Gimbal_Debug.Pitch_Angle_kk = 0.0;
    Gimbal_Debug.Yaw_Speed_Kp = 16.0;   Gimbal_Debug.Yaw_Speed_Ki = 0.1;
	Gimbal_Debug.Yaw_Speed_Kd = 1.6;   Gimbal_Debug.Yaw_Speed_KK = 2.0;
    Gimbal_Debug.Yaw_Angle_Kp = 3.0;    Gimbal_Debug.Yaw_Angle_Ki = 0.02;
	Gimbal_Debug.Yaw_Angle_Kd = 0.0;    Gimbal_Debug.Yaw_Speed_D_filter = 0.3;
	Gimbal_Debug.Yaw_Angle_KK = 0.0;
    Gimbal_Debug.Sweep_Freq = 10.0; 
    Gimbal_Debug.Sweep_Amp  = 100.0; 
    Gimbal_Debug.Sweep_Time = 0.0;
   
    GPID_Init(&Pitch_Motor.Angle_PID,3.0,0.1,0,0,0.5, 100);    // 角度环限幅
    GPID_Init(&Pitch_Motor.Speed_PID, 15,0.06,1.5,0.3,100, 1000); // 速度环限幅
    
    GPID_Init(&Yaw_Motor.Angle_PID, 3.0,0.02,0,0,1, 100);
    GPID_Init(&Yaw_Motor.Speed_PID, 16,0.1,1.6,2,600, 1000);
}

double Gimbal_Control_Loop(Gimbal_Motor_Handle_t *motor, int axis_id, double current_raw_enc, double current_speed) {
    
	
	
	
	
	Gimbal_Debug.Actual_Speed = current_speed;
    Sync_Params(motor, axis_id);

    
    double diff = current_raw_enc - motor->Last_Raw_Enc;
    if (diff > ENCODER_HALF_RANGE) motor->Round_Count--;
    else if (diff < -ENCODER_HALF_RANGE) motor->Round_Count++;
    motor->Last_Raw_Enc = current_raw_enc;
    motor->Angle_Total = (current_raw_enc / ENCODER_MAX_RANGE * 360.0) + (motor->Round_Count * 360.0);

    double final_iq = 0.0;
    
    switch (Gimbal_Debug.Control_Mode) {
        case DEBUG_STOP:
            motor->Speed_PID.I_Out = 0; 
            motor->Angle_PID.I_Out = 0;
            final_iq = 0;
            break;
            
        case DEBUG_SPEED_LOOP: // 正弦波摆动
            {
                static double t = 0.0;
                const double dt = 0.001; 
                t += dt;
                
                if (t > 5.0f) {
                    t -= 5.0f; 
                }
                
                Gimbal_Debug.sine_target_speed = 20.0* sin(2.0 * 3.14159265 * 0.2 * t);
				final_iq = GPID_Calc_Normal(&motor->Speed_PID, Gimbal_Debug.sine_target_speed, current_speed);
//				final_iq = GPID_Calc_Normal(&motor->Speed_PID, 0.0, imu_feedback_speed);
            }
            
            break;
            
        case DEBUG_ANGLE_LOOP: 
            {
                double current_imu_angle = 0.0;
                double current_target_angle = 0.0;
                if (axis_id == 2) {
//                    current_target_angle = Gimbal_Debug.Pitch_Target_Angle;
					current_target_angle = 0.0;
                    current_imu_angle = hi_imu_data.pitch;
                } else if (axis_id == 1) {
//                    current_target_angle = Gimbal_Debug.Yaw_Target_Angle;
					 current_target_angle = 0.0;
                    current_imu_angle = hi_imu_data.yaw;
                }
                
                
                double angle_error = current_target_angle - current_imu_angle;
                while (angle_error > 180.0)  angle_error -= 360.0;
                while (angle_error < -180.0) angle_error += 360.0;
                
                
                double target_spd = GPID_Calc_Normal(&motor->Angle_PID, angle_error, 0.0);

                
                final_iq = GPID_Calc_Normal(&motor->Speed_PID, target_spd, current_speed);
            }
            break;
            case DEBUG_SWEEP: 
        {
            
            const double dt = 0.001; 
            Gimbal_Debug.Sweep_Time += dt;
            
            
            if (Gimbal_Debug.Sweep_Time > 1000.0) {
                Gimbal_Debug.Sweep_Time = 0.0; 
            }

            final_iq = Gimbal_Debug.Sweep_Amp * sinf(2.0* 3.14159265 * Gimbal_Debug.Sweep_Freq * Gimbal_Debug.Sweep_Time);
            
            
            motor->Speed_PID.I_Out = 0; 
            motor->Angle_PID.I_Out = 0;
            motor->Speed_PID.Last_Measure = current_speed;
        }
            break;
		case DEBUG_POINTMODE: 
        {
            double current_imu_angle = 0.0;
            double current_target_angle = 0.0;
            const double dt = 0.001; 

            if (axis_id == 2) {
                
                current_target_angle = 0.0;
                current_imu_angle = hi_imu_data.pitch;
                
                Gimbal_Debug.Pitch_Target_Angle = current_target_angle; 
            } else if (axis_id == 1) {
                
                static double yaw_target = -2.05; 
                static int yaw_dir = 1; 
                
                
                yaw_target += yaw_dir * 1.23 * dt;
                
                
                if (yaw_target >= 2.05) {
                    yaw_target = 2.05;
                    yaw_dir = -1;
                } else if (yaw_target <= -2.05) {
                    yaw_target = -2.05;
                    yaw_dir = 1;
                }
                
                current_target_angle = yaw_target;
                current_imu_angle = hi_imu_data.yaw;
                
                // 同步给上位机查看目标波形
                Gimbal_Debug.Yaw_Target_Angle = current_target_angle;
            }
            
            
            double angle_error = current_target_angle - current_imu_angle;
            while (angle_error > 180.0)  angle_error -= 360.0;
            while (angle_error < -180.0) angle_error += 360.0;
            
            
            double target_spd = GPID_Calc_Normal(&motor->Angle_PID, angle_error, 0.0);

            
            final_iq = GPID_Calc_Normal(&motor->Speed_PID, target_spd, current_speed);
        }
        break;
    }
    return final_iq;
}
