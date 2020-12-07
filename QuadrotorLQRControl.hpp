#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/landing_gear.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/experiment_mode.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;


class QuadrotorLQRControl
{
public:
       QuadrotorLQRControl();
       
       void setEquilibriumPoint(Matrix<float,12,1> eqPoint);
       
       void setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos);
       
       void setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos);
       
       Matrix<float,4,1> LQRcontrol();
       
       void setAutoEqPointFlag(bool flag);
    
       void setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos); 

       bool getAutoEqPointFlag();

       Matrix<float,4,1> getLQRcontrols();
       
       void setCurrentState(Matrix<float,12,1> _state_estimate);

       Matrix<float,4,1> normalizationControlInputs(Matrix<float,4,1> _u);
       

private:
       
      
       
      Matrix<float,12,1> _eq_point;

      Matrix<float,4,12> _K;

      Matrix<float,12,12> _P;
      
      Matrix<float,12,1> _current_state;

      Matrix<float,12,1> _current_state_ekf;


      bool _auto_eq_point_flag;
	  float ff_thrust;
	  float ux;
	  float uy; 
	  float s1; 
	  float s2;
	  float s3; 
	  float s4;
	 
	 
	 //------ SMC - Ecuador --------
	 
	 float s_ecuad; 
	 float kd_z;
	 float kd_roll;
	 float kd_pitch;
	 float kd_yaw; 
	 
	 float lambda_z;
	 float lambda_roll;
	 float lambda_pitch;
	 float lambda_yaw;
	 
	 float delta_z;
	 float delta_roll;
	 float delta_pitch;
	 float delta_yaw;
	 
	 	  
	 
	 //---------------------------
	
	 float sx;
	 float sy;
	
	 float xd7;//roll 
	 float xd7_dot;
	 float xd7_ddot;
	 float xd7_old;
	 float xd7_dot_old;
	 
	 float xd8; //pitch 
	 float xd8_dot;
	 float xd8_ddot;
	 float xd8_old;
	 float xd8_dot_old;

	 float xd9; //yaw 
	 float xd9_dot;
	 float xd9_ddot;
	 float xd9_old;
	 float xd9_dot_old;
	 
	 float b1;
	 float b2;
	 float b3;
	 
	 float a1;
	 float a2;
	 float a3; 
	 
	 // Quadcopter Properties
	 float m = 0.8;
	 float Jx = 0.005;
	 float Jy = 0.005;
	 float Jz = 0.009;
	 float g = 9.81;
	 float l = 1; //0.33 / 2;

	 
	// For Altitude Z
	 float c1 = 68;
	 float k1 = 25;
	 float k2 = 10;
	 
	 // For X-axis
	 float c2 = 0.001;
	 float k3 = 0.001;
	 float k4 = 0.001;
	 
	 // For Y-axis
	 float c3 = 0.001;
	 float k5 = 0.001;
	 float k6 = 0.001;
   
	 // For phi - u2
	 float c4 = 5; //20
	 float k7 = 100; //5
	 float k8 = 50; //5
					 
	// For theta - u3
	 float c5 = 5;
	 float k9 = 100;
	 float k10 = 10;
   
   // For psi - u4
     float c6 = 5; //20
     float k11 = 100; //5
     float k12 = 10; //5

	   

      void writeStateOnFile(const char *filename, Matrix <float, 12, 1> vect, hrt_abstime t); 

      Matrix <float, 4, 12> readMatrixK(const char *filename);

      void writeInputOnFile(const char *filename, Matrix <float, 4, 1> vect, hrt_abstime t); 

      void writeLyapunovOnFile(const char *filename, float value, hrt_abstime t);

      Matrix <float, 12, 12> readMatrixP(const char *filename);

      float _past_time;

      Matrix<float,4,1> u_control;
	  
	 int sign(double v);

	 float sat(float s);
      

};