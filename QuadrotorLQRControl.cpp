#include "QuadrotorLQRControl.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

//#include <eigen3/Eigen/Dense> // ++ Math operations with Matrices ++

/* -------------------- added part ---------------------*/
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>

using namespace matrix;
using namespace std;

QuadrotorLQRControl::QuadrotorLQRControl()
{

    for (int i=0;i<12;i++)
    {
       _current_state(i,0) = 0.0f;
       _eq_point(i,0) = 0.0f;
    }

     // _eq_point(1,0) =  0.0f; //x
     // _eq_point(3,0) =  0.0f; //y
     // _eq_point(5,0) = -1.0f; //z
	
	 // Sliding Mode Control 
	 _eq_point(1,0) = 0.0f; //x
     _eq_point(3,0) = 0.0f; //y
     _eq_point(5,0) = -5.0f; //z 
	
	 ux = 0.0f;
	 uy = 0.0f; 
	 s1 = 0.0f; 
	 s2 = 0.0f;
	 s3 = 0.0f; 
	 s4 = 0.0f;
	
	 sx = 0.0f;
	 sy = 0.0f;
	
	 xd7 = 0.0f;
	 xd7_dot = 0.0f;
	 xd7_ddot = 0.0f;
	 xd7_old = 0.0f;
	 xd7_dot_old = 0.0f;
	 
	 xd8 = 0.0f;
	 xd8_dot = 0.0f;
	 xd8_ddot = 0.0f;
	 xd8_old = 0.0f;
	 xd8_dot_old = 0.0f;
	

     u_control(0,0) = 0.0f;
     u_control(1,0) = 0.0f;
     u_control(2,0) = 0.0f;
     u_control(3,0) = 0.0f;

     //_K = readMatrixK("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/lqr_files/new_controller.txt");
	 //_K = readMatrixK("C:\PX4\home\px4_external\src\modules\mc_att_control\lqr_files\new_controller.txt");
     //_P = readMatrixP("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/lqr_files/new_pe.txt");
	 //_P = readMatrixP("C:\PX4\home\px4_external\src\modules\mc_att_control\lqr_files\new_pe.txt");

     // ff_thrust = 5.886f; // [N]

     _auto_eq_point_flag = true;

     ofstream outfile1;
     //outfile1.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/control_input.txt", std::ios::out);
	 outfile1.open("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/control_input.txt");
     outfile1.close();

        
     ofstream outfile3;
     //outfile3.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/state.txt", std::ios::out);
	 outfile3.open("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/state.txt");
     outfile3.close();

     
     ofstream outfile5;
     //outfile5.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", std::ios::out);
	 outfile5.open("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt");
     outfile5.close();

     ofstream outfile4;
     //outfile4.open("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/ekf.txt", std::ios::out);
	 outfile4.open("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/ekf.txt");
     outfile4.close();

     _past_time = hrt_absolute_time() * 1e-6;

}



Matrix <float, 4, 12>  QuadrotorLQRControl::readMatrixK(const char *filename)
    {

    static Matrix <float, 4, 12> result;
    static int rows = 4;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }
 

Matrix<float,4,1> QuadrotorLQRControl::LQRcontrol()
{
       
     
     //static Matrix<float,4,1> u_control;
     static Matrix<float,4,1> u_control_norm;
     static Matrix<float,12,1> delta_x;
     static Matrix<float, 1,12> v_b;
     static Matrix<float,1,12> delta_x_T;
     static Matrix<float,1,1> _lyap_fun;     
     const hrt_abstime now = hrt_absolute_time();

     float _current_time = now *1e-6;
     float dt = _current_time-_past_time;
	 
	 //-------- Sliding Mode Control -------
	 
	 xd7_dot = (xd7 - xd7_old)/dt; 
	 xd8_dot = (xd8 - xd8_old)/dt;
	 
	 xd7_ddot = (xd7_dot - xd7_dot_old)/dt; 
	 xd8_ddot = (xd8_dot - xd8_dot_old)/dt;
	 
	 xd7_old = xd7;
	 xd8_old = xd8;
	 xd7_dot_old = xd7_dot;
	 xd8_dot_old = xd8_dot;
     
     _past_time = _current_time;

     //delta_x   = _current_state - _eq_point;    
     //u_control = - _K*(delta_x); 
 
     //delta_x_T = delta_x.transpose();
    
     //v_b = delta_x_T*_P;
     //_lyap_fun = v_b*delta_x;
     //cout<< dt << "\t" << _P(0,0) << "\n";
     // !! IMPORTANT scale the control inputs.......
   
     //------------------------ Sliding Mode Control ---------------------------------------
   
	   
	   //Matlab State Order  ---------- 
	   // x1 - pos_x    		| _current_state(1,0)
	   // x2 - pos_y    		| _current_state(3,0)
	   // x3 - pos_z    		| _current_state(5,0)
	   // x4 - x_dot 			| _current_state(0,0)
	   // x5 - y_dot			| _current_state(2,0)
	   // x6 - z_dot			| _current_state(4,0)
	   // x7 - roll-phi 		| _current_state(7,0)
	   // x8 - pitch-theta 		| _current_state(9,0)
	   // x9 - yaw - psi		| _current_state(11,0)
	   // x10 - roll_dot		| _current_state(6,0)
	   // x11 - pitch_dot		| _current_state(8,0)
	   // x12 - psi_dot 		| _current_state(10,0)
	   
	   b1 = l/Jx;
	   b2 = l/Jy;
	   b3 = 1/Jz;
	   a1 = (Jx-Jy)/Jx;
	   a2 = (Jz-Jx)/Jy;
	   a3 = (Jx-Jy)/Jz;
	 

	   // sx = (_eq_point(0,0) - _current_state(0,0)) + c2*(_eq_point(1,0)-_current_state(1,0));
	   // sy = (_eq_point(4,0) - _current_state(2,0)) + c3*(_eq_point(3,0)-_current_state(3,0));
	   //u_control(0,0) = -(m/(cos(x(7))*cos(x(8))))*(k1*sign(s1)+k2*s1+g+c1*(r(6)-x(6)));
	    
	   
	   //motion control about x and y axis
	   // ux = (m/u_control(0,0))*(k3*sign(sx) + k4*sx + c2*(_eq_point(0,0) -_current_state(0,0)));
	   // uy = (m/u_control(0,0)*(k5*sign(sy) + k6*sy + c3*(_eq_point(2,0)-_current_state(2,0))));
	   
	   // xd7 = (asin(-uy-floor(-uy))) + M_PI*floor(-uy)/2; // desired as functions 
	   // xd8 = (asin(ux/cos(xd7) - floor(ux/cos(xd7))))+ M_PI*floor(ux/cos(xd7))/2;
	   
	   //angular motion control 
	   // s2 = (xd7_dot-_current_state(6,0)) + c4*(xd7-_current_state(7,0));
	   // s3 = (xd8_dot-_current_state(8,0)) + c5*(xd8-_current_state(9,0));

       s1 = (_eq_point(4, 0) - _current_state(4, 0)) + c1 * (_eq_point(5, 0) - _current_state(5, 0));

       u_control(0, 0) = -(m / (cos(_current_state(7, 0)) * cos(_current_state(9, 0)))) * (k1 * sat(s1) + k2 * s1 + g + c1 * (_eq_point(4, 0) - _current_state(4, 0)));
	   
	   s2 = (0 -_current_state(6,0)) + c4*(0 -_current_state(7,0)); //-roll
	   s3 = (0 -_current_state(8,0)) + c5*(0 -_current_state(9,0)); //-pitch
	   s4 = (0- _current_state(10,0)) + c6*(0 -_current_state(11,0)); //-yaw
	   
	   // u_control(1,0) = ix*(k8*sign(s2)+k9*s2 + xd7_ddot + c4*(xd7_dot - _current_state(6,0)));
	   // u_control(2,0) = iy*(k9*sign(s3)+k10*s3 + xd8_ddot + c5*(xd8_dot - _current_state(8,0)));
	   // u_control(3,0) = iz*(k11*sign(s4)+k12*s4 + c6*(0 -_current_state(10,0)));
	   
   	   u_control(1,0) = (1/b1)*(k8* sat(s2)+k9*s2 - a1*_current_state(8,0)*_current_state(10,0) + c4*(0 - _current_state(6,0))); //roll
	   u_control(2,0) = (1/b2)*(k9* sat(s3)+k10*s3 - a2*_current_state(6,0)*_current_state(10,0) + c5*(0 - _current_state(8,0))); //pitch
	   u_control(3,0) = (1/b3)*(k11* sat(s4)+k12*s4 - a3*_current_state(6,0)*_current_state(9,0) + c6*(0 -_current_state(10,0))); //yaw
	   
	   //----------------------------- Sliding Mode Control 2.0 Ecuador ----------------------
	   
 
	  
	   
	   
	   //-----------------------------------------------------------------------------------------


		u_control_norm(1,0) = fmin(fmax((u_control(1,0))/(0.1080f*4.0f), -1.0f), 1.0f);  //u2 - roll
		u_control_norm(2,0) = fmin(fmax((u_control(2,0))/(0.1080f*4.0f),  -1.0f), 1.0f); //u3 - pitch
		u_control_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1080f*4.0f), -1.0f), 1.0f);	 //u4 - yaw
		//u_control_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust)/16.0f, 0.0f), 1.0f);  //u1 - thrust 
	    u_control_norm(0,0) = fmin(fmax((u_control(0,0))/16.0f, 0.0f), 1.0f);  //u1 - thrust 


	   // not normalized control inputs
		 u_control(0,0) = u_control_norm(0,0)*16.0f;
		 u_control(1,0) = u_control_norm(1,0)*4.0f;
		 u_control(2,0) = u_control_norm(2,0)*4.0f;
		 u_control(3,0) = u_control_norm(3,0)*4.0f;
		 
		//"\t" <<  u_control(0,0)+ff_thrust << "\n";
			 /* Save data*/
		// writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
		// writeInputOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control_norm, now); 
		// writeLyapunovOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", _lyap_fun(0,0), now); 
		// writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/ekf.txt", _current_state_ekf, now);
		
		//------------- Javier-----------------------
		writeStateOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
		writeInputOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control_norm, now); 
		writeLyapunovOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_fileslyapunov.txt", _lyap_fun(0,0), now); 
		writeStateOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_filesekf.txt", _current_state_ekf, now);
		
		return u_control_norm;    

}

Matrix<float,4,1> QuadrotorLQRControl::normalizationControlInputs(Matrix<float,4,1> _u)
{
   Matrix<float,4,1> _u_norm;
   _u_norm(0,0) = _u(0,0)*16.0f;
   _u_norm(1,0) = _u(1,0)*(0.1080f*4.0f);
   _u_norm(2,0) = _u(2,0)*(0.1080f*4.0f);
   _u_norm(3,0) = _u(3,0)*(0.1f*1.0f); 

return _u_norm;
}

Matrix<float,4,1> QuadrotorLQRControl::getLQRcontrols()
{

return u_control;

}

void QuadrotorLQRControl::setCurrentState(Matrix<float,12,1> _state_estimate)
{

      _current_state = _state_estimate;

}



void QuadrotorLQRControl::setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state(0,0) = _v_local_pos.vx; //xdot
      _current_state(1,0) = _v_local_pos.x;  //x 
      _current_state(2,0) = _v_local_pos.vy; //ydot
      _current_state(3,0) = _v_local_pos.y; //y
      _current_state(4,0) = _v_local_pos.vz; //zdot
      _current_state(5,0) = _v_local_pos.z; //z
    
      _current_state(6,0)  = _v_att.rollspeed;                 //roll dot
      _current_state(7,0)  = Eulerf(Quatf(_v_att.q)).phi();    //roll - phi
      _current_state(8,0)  = _v_att.pitchspeed;                //pitch dot
      _current_state(9,0)  = Eulerf(Quatf(_v_att.q)).theta();  //pitch - theta
      _current_state(10,0) = _v_att.yawspeed;	               // yawdot
      _current_state(11,0) = Eulerf(Quatf(_v_att.q)).psi();    //yaw - psi

}

void QuadrotorLQRControl::setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state_ekf(0,0) = _v_local_pos.vx;
      _current_state_ekf(1,0) = _v_local_pos.x;
      _current_state_ekf(2,0) = _v_local_pos.vy;
      _current_state_ekf(3,0) = _v_local_pos.y;
      _current_state_ekf(4,0) = _v_local_pos.vz;
      _current_state_ekf(5,0) = _v_local_pos.z;
    
      _current_state_ekf(6,0)  = _v_att.rollspeed;
      _current_state_ekf(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state_ekf(8,0)  = _v_att.pitchspeed;
      _current_state_ekf(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state_ekf(10,0) = _v_att.yawspeed;	
      _current_state_ekf(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}


void QuadrotorLQRControl::setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _eq_point(0,0) = _v_local_pos.vx;
      _eq_point(1,0) = _v_local_pos.x;
      _eq_point(2,0) = _v_local_pos.vy;
      _eq_point(3,0) = _v_local_pos.y;
      _eq_point(4,0) = _v_local_pos.vz;
      _eq_point(5,0) = _v_local_pos.z;
    
      _eq_point(6,0)  = _v_att.rollspeed;
      _eq_point(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _eq_point(8,0)  = _v_att.pitchspeed;
      _eq_point(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _eq_point(10,0) = _v_att.yawspeed;	
      _eq_point(11,0) = Eulerf(Quatf(_v_att.q)).psi();


}

void QuadrotorLQRControl::setEquilibriumPoint(Matrix<float,12,1> eqPoint)
{

      _eq_point(0,0) = eqPoint(0,0);
      _eq_point(1,0) = eqPoint(1,0);
      _eq_point(2,0) = eqPoint(2,0);
      _eq_point(3,0) = eqPoint(3,0);
      _eq_point(4,0) = eqPoint(4,0);
      _eq_point(5,0) = eqPoint(5,0);
    
      _eq_point(6,0)  = eqPoint(6,0);
      _eq_point(7,0)  = eqPoint(7,0);
      _eq_point(8,0)  = eqPoint(8,0);
      _eq_point(9,0)  = eqPoint(9,0);
      _eq_point(10,0) = eqPoint(10,0);	
      _eq_point(11,0) = eqPoint(11,0);

    
      

}

void QuadrotorLQRControl::setAutoEqPointFlag(bool flag)
{

   _auto_eq_point_flag = flag;
}

bool QuadrotorLQRControl::getAutoEqPointFlag()
{

   return _auto_eq_point_flag;
}

/* Save data on files */

void QuadrotorLQRControl::writeStateOnFile(const char *filename, Matrix <float, 12, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
       
	for(int i=0;i<12;i++){
		if(i==11){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}


void QuadrotorLQRControl::writeInputOnFile(const char *filename, Matrix <float, 4, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
        
	for(int i=0;i<4;i++){
		if(i==3){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeLyapunovOnFile(const char *filename, float value, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t" << value << "\n";   
	outfile.close();
	return;
}

Matrix <float, 12, 12>  QuadrotorLQRControl::readMatrixP(const char *filename)
    {

    static Matrix <float, 12, 12> result;
    static int rows = 12;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file";
    return result;

 }
 
 int QuadrotorLQRControl::sign(double v) 
 {
  return (v > 0) - (v < 0);
}

 float QuadrotorLQRControl::sat(float s)
 {
     if (fabs(s) < 1)
         return s;
     else if (fabs(s) > 1)
         return sign(s);
 }

