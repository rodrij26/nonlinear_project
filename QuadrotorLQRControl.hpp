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
	
	 float sx;
	 float sy;
	
	 float xd7;
	 float xd7_dot;
	 float xd7_ddot;
	 float xd7_old;
	 float xd7_dot_old;
	 
	 float xd8;
	 float xd8_dot;
	 float xd8_ddot;
	 float xd8_old;
	 float xd8_dot_old;
	 
		float m = 0.6;
	   float Ix = 0.0092;
	   float Iy = 0.0092;
	   float Iz = 0.0101;
	   float g = 9.81;
   
	   // For Altitude Z
	   float c1 = 68;
	   float k1 = 25;
	   float k2 = 25/0.3;
	   
	   // For X-axis
	   float c2 = 2;
	   float k3 = 0.1;
	   float k4 = 0.1/0.3;
	   
	   // For Y-axis
	   float c3 = 2;
	   float k5 = 0.1;
	   float k6 = 0.1/0.3;
	   
	   // For phi - u2
	   float c4 = 5;
	   float k7 = 100;
	   float k8 = 100/0.3;
	   
	   // For theta - u3
	   float c5 = 5;
	   float k9 = 100;
	   float k10 = 100/0.3;
	   
	   // For psi - u4
	   float c6 = 5;
	   float k11 = 100;
	   float k12 = 100/0.3;
	   

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
