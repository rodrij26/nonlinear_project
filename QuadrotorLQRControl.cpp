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

	for (int i = 0; i < 12; i++)
	{
		_current_state(i, 0) = 0.0f;
		_eq_point(i, 0) = 0.0f;
	}

	for (int i = 0; i < 3; i++)
	{
		_ref_points(i, 0) = 0.0f;
		_t_norm(i, 0) = 0.0f;
		_t(i, 0) = 0.0f;
	}

	_eq_point(1, 0) = -1.0f; //x
	_eq_point(3, 0) = -1.0f; //y
	_eq_point(5, 0) = -1.0f; //z

   // Sliding Mode Control 


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

	old_time_ref = 0.0f;
	time_ref = 0.0f;

	psi_des = 0.0f;


	ddxc = 0.0f;
	ddyc = 0.0f;
	ddzc = 0.0f;


	u_control(0, 0) = 0.0f;
	u_control(1, 0) = 0.0f;
	u_control(2, 0) = 0.0f;
	u_control(3, 0) = 0.0f;


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

	ofstream outfile6;
	outfile6.open("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/ref.txt");
	outfile6.close();


	_past_time = hrt_absolute_time() * 1e-6;

}



Matrix <float, 4, 12>  QuadrotorLQRControl::readMatrixK(const char* filename)
{

	static Matrix <float, 4, 12> result;
	static int rows = 4;
	static int cols = 12;
	ifstream infile;
	infile.open(filename);
	if (infile.is_open()) {
		for (int i = 0; i < rows; i++) {
			string line;
			getline(infile, line);
			stringstream stream(line);
			for (int j = 0; j < cols; j++) {
				stream >> result(i, j);
			}

		}
		infile.close();
	}
	else cout << "Unable to open file";
	return result;

}


Matrix<float, 4, 1> QuadrotorLQRControl::LQRcontrol()
{


	//static Matrix<float,4,1> u_control;
	static Matrix<float, 4, 1> u_control_norm;
	static Matrix<float, 12, 1> delta_x;
	static Matrix<float, 1, 12> v_b;
	static Matrix<float, 1, 12> delta_x_T;
	static Matrix<float, 1, 1> _lyap_fun;
	const hrt_abstime now = hrt_absolute_time();

	float _current_time = now * 1e-6;
	float dt = _current_time - _past_time;
	float dt_ref = dt; 

	if (dt_ref > 0.010)
		dt_ref = 0.004;


	//---------------------------- reference generator ---------------------------------

	ref_x = _ref_points(0, 0);
	_eq_point(1, 0) = ref_x;

	ref_y = _ref_points(1, 0);
	_eq_point(3, 0) = ref_y;

	ref_z = _ref_points(2, 0);
	_eq_point(5, 0) = ref_z;

	cout << "time: " << time_ref <<", ref_x: " << ref_x << ", ref_y: " << ref_y << ", ref_z: " << ref_z << "\n";
    cout << "-------------------------------------------------------------" << "\n \n";

	ref_xdot = (ref_x - ref_x_old) / dt;
	_eq_point(0, 0) = ref_xdot;

	ref_ydot = (ref_y - ref_y_old) / dt;
	_eq_point(2, 0) = ref_ydot;

	ref_zdot = (ref_z - ref_z_old) / dt;
	_eq_point(4, 0) = ref_zdot;

	ref_xddot = (ref_xdot - ref_xdot_old) / dt; //x
	ref_yddot = (ref_ydot - ref_ydot_old) / dt; //y
	ref_zddot = (ref_zdot - ref_zdot_old) / dt; //z

	ref_x_old = ref_x;
	ref_y_old = ref_y;
	ref_z_old = ref_z;

	ref_xdot_old = ref_xdot;
	ref_ydot_old = ref_ydot;
	ref_zdot_old = ref_zdot;

	//------------------------------------------------------

	xd7_dot = (xd7 - xd7_old) / dt;
	xd8_dot = (xd8 - xd8_old) / dt;
	xd9_dot = (xd9 - xd9_old) / dt;

	xd7_ddot = (xd7_dot - xd7_dot_old) / dt;
	xd8_ddot = (xd8_dot - xd8_dot_old) / dt;
	xd9_ddot = (xd9_dot - xd9_dot_old) / dt;

	xd7_old = xd7;
	xd8_old = xd8;
	xd9_old = xd9;
	xd7_dot_old = xd7_dot;
	xd8_dot_old = xd8_dot;
	xd9_dot_old = xd9_dot;

	_past_time = _current_time;


	time_ref = old_time_ref + dt_ref;
	old_time_ref = time_ref;

	generateRef(time_ref);

	//delta_x   = _current_state - _eq_point;    
	//u_control = - _K*(delta_x); 

	//delta_x_T = delta_x.transpose();

	//v_b = delta_x_T*_P;
	//_lyap_fun = v_b*delta_x;
	//cout<< dt << "\t" << _P(0,0) << "\n";
	// !! IMPORTANT scale the control inputs.......

	//------------------------ Sliding Mode Control ---------------------------------------
 

	//Matlab State Order  ---------- 
	// x(1) - pos_x    		| _current_state(1,0)
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

	//a1 = (Ix - Iy) / Ix;
	//a2 = (Iz - Ix) / Iy;
	//a3 = (Ix - Iy) / Iz;
	//b1 = 1 / Ix;
	//b2 = 1 / Iy;
	//b3 = 1 / Iz;

	

	//sx = (_eq_point(0, 0) - _current_state(0, 0)) + c2 * (_eq_point(1, 0) - _current_state(1, 0));
	//sy = (_eq_point(4, 0) - _current_state(2, 0)) + c3 * (_eq_point(3, 0) - _current_state(3, 0));
	////cout << "s1: " << s1 << ", sx: " << sx << ", sy: " << sy << "\n";
	//
	////motion control about x and y axis
	//u_control(0, 0) = -(m / (cos(_current_state(7, 0)) * cos(_current_state(9, 0)))) * (k1 * sat(s1) + k2 * s1 + g + c1 * (_eq_point(4, 0) - _current_state(4, 0)));

	//ux = (m / u_control(0, 0)) * (k3 * sat(sx) + k4 * sx + c2 * (_eq_point(0, 0) - _current_state(0, 0)));
	//uy = (m / u_control(0, 0)) * (k5 * sat(sy) + k6 * sy + c3 * (_eq_point(2, 0) - _current_state(2, 0)));

	//xd7 = asin(sat(uy));
	//xd8 = asin(sat(-ux / cos(xd7)));
	////xd9 = atan((_eq_point(1, 0) - _current_state(1, 0))/ (_eq_point(3, 0) - _current_state(3, 0)));
	//xd9 = 0;

	////angular motion control --- s = e_dot + c*e
	//s2 = (xd7_dot - _current_state(6, 0)) + c4 * (xd7 - _current_state(7, 0));
	//s3 = (xd8_dot - _current_state(8, 0)) + c5 * (xd8 - _current_state(9, 0));
	//s4 = (xd9_dot - _current_state(10, 0)) + c6 * (xd9 - _current_state(11, 0));


	////cout << "s2: " << s2 << ", s3: " << s3 << ", s4: " << s4 << "\n";

	//u_control(1, 0) = (1 / b1) * (k7 * sat(s2) + k8 * s2 + xd7_ddot - a1 * _current_state(8, 0) * _current_state(10, 0) + c4 * (xd7_dot - _current_state(6, 0))); //roll
	//u_control(2, 0) = (1 / b2) * (k9 * sat(s3) + k10 * s3 + xd8_ddot - a2 * _current_state(6, 0) * _current_state(10, 0) + c5 * (xd8_dot - _current_state(8, 0))); //pitch
	//u_control(3, 0) = (1 / b3) * (k11 * sat(s4) + k12 * s4 + xd9_ddot - a3 * _current_state(6, 0) * _current_state(8, 0) + c6 * (xd9_dot - _current_state(10, 0))); //yaw

	//-------------------------- MATLAB Approach --------------------------------------------- //


	a1 = (Iy - Iz) / Ix;
	a2 = (Iz - Ix) / Iy;
	a3 = (Ix - Iy) / Iz;
	b1 = 1 / Ix;
	b2 = 1 / Iy;
	b3 = 1 / Iz;

	ddxc = ref_xddot + Kdx * (ref_xdot - _current_state(0, 0)) + Kpx * (ref_x - _current_state(1, 0));
	ddyc = ref_yddot + Kdy * (ref_ydot - _current_state(2, 0)) + Kpz * (ref_y - _current_state(3, 0));
	ddzc = ref_zddot + Kdz * (ref_zdot - _current_state(4, 0)) + Kpx * (ref_z - _current_state(5, 0));

	_t(0, 0) = ddxc;
	_t(1, 0) = ddyc;
	_t(2, 0) = ddzc;

	float norm_t = pow(pow(ddxc, 2) + pow(ddyc, 2) + pow(ddzc + m*g, 2), 0.5);

	_t_norm = _t / norm_t; // verify that you can do this

	//position control

	float sinTheta = cos(psi_des) * _t_norm(0, 0) + sin(psi_des) * _t_norm(1, 0);
	xd8 = asin(sinTheta);
	float sinPhi =  (sin(psi_des) * _t_norm(0,0) - cos(psi_des) * _t_norm(1,0)) / cos(xd8);
	xd7 = asin(sinPhi);
	xd9 = psi_des; 


	//sliding surface
	s1 = (_eq_point(4, 0) - _current_state(4, 0)) + c1 * (_eq_point(5, 0) - _current_state(5, 0));
	s2 = (xd7_dot - _current_state(6, 0)) + c4 * (xd7 - _current_state(7, 0));
	s3 = (xd8_dot - _current_state(8, 0)) + c5 * (xd8 - _current_state(9, 0));
	s4 = (xd9_dot - _current_state(10, 0)) + c6 * (xd9 - _current_state(11, 0));


	//Sliding Mode Control 
	u_control(0, 0) = -(m / (cos(_current_state(7, 0)) * cos(_current_state(9, 0)))) * (k1 * sat(s1) + k2 * s1 + g + c1 * (_eq_point(4, 0) - _current_state(4, 0)));
	u_control(1, 0) = (1/b1) * (-((a1 * _current_state(10, 0) * _current_state(8, 0)) + 5 * _current_state(6, 0)) - 1 * sat(s2)); //roll
	u_control(2, 0) = (1/b2) * (-((a2 * _current_state(10, 0) * _current_state(6, 0)) + 5 * _current_state(8, 0)) - 1 * sat(s3)); //pitch
	u_control(3, 0) = (1/b3) * (-((a3 * _current_state(10, 0) * _current_state(8, 0)) + 5 * _current_state(10, 0)) - 1 * sat(s4)); //yaw


	//-----------------------------------------------------------------------------------------


	u_control_norm(1, 0) = fmin(fmax((u_control(1, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f);
	u_control_norm(2, 0) = fmin(fmax((u_control(2, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f);
	u_control_norm(3, 0) = fmin(fmax((u_control(3, 0)) / (0.1f * 1.0f), -1.0f), 1.0f);
	u_control_norm(0, 0) = fmin(fmax((u_control(0, 0)) / 16.0f, 0.0f), 1.0f);

	// not normalized control inputs
	u_control(0, 0) = u_control_norm(0, 0) * 16.0f;
	u_control(1, 0) = u_control_norm(1, 0) * 4.0f;
	u_control(2, 0) = u_control_norm(2, 0) * 4.0f;
	u_control(3, 0) = u_control_norm(3, 0) * 0.05f;

	//"\t" <<  u_control(0,0)+ff_thrust << "\n";
		 /* Save data*/
	// writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
	// writeInputOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control_norm, now); 
	// writeLyapunovOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/lyapunov.txt", _lyap_fun(0,0), now); 
	// writeStateOnFile("/home/raffaele/PX4/Firmware/src/modules/mc_att_control/output_files/ekf.txt", _current_state_ekf, now);

	//------------- Siddartha-----------------------
	writeRefOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/reference.txt", _ref_points, now);
	writeStateOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/state.txt", _current_state, now);
	writeInputOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_files/control_input.txt", u_control_norm, now);
	writeLyapunovOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_fileslyapunov.txt", _lyap_fun(0, 0), now);
	writeStateOnFile("C:/PX4/home/Firmware/src/modules/mc_att_control/output_filesekf.txt", _current_state_ekf, now);

	return u_control_norm;

}

Matrix<float, 4, 1> QuadrotorLQRControl::normalizationControlInputs(Matrix<float, 4, 1> _u)
{
	Matrix<float, 4, 1> _u_norm;
	_u_norm(0, 0) = _u(0, 0) * 16.0f;
	_u_norm(1, 0) = _u(1, 0) * (0.1080f * 4.0f);
	_u_norm(2, 0) = _u(2, 0) * (0.1080f * 4.0f);
	_u_norm(3, 0) = _u(3, 0) * (0.1f * 1.0f);

	return _u_norm;
}

Matrix<float, 4, 1> QuadrotorLQRControl::getLQRcontrols()
{

	return u_control;

}

void QuadrotorLQRControl::setCurrentState(Matrix<float, 12, 1> _state_estimate)
{

	_current_state = _state_estimate;

}



void QuadrotorLQRControl::setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

	_current_state(0, 0) = _v_local_pos.vx; //xdot
	_current_state(1, 0) = _v_local_pos.x;  //x 
	_current_state(2, 0) = _v_local_pos.vy; //ydot
	_current_state(3, 0) = _v_local_pos.y; //y
	_current_state(4, 0) = _v_local_pos.vz; //zdot
	_current_state(5, 0) = _v_local_pos.z; //z

	_current_state(6, 0) = _v_att.rollspeed;                 //roll dot
	_current_state(7, 0) = Eulerf(Quatf(_v_att.q)).phi();    //roll - phi
	_current_state(8, 0) = _v_att.pitchspeed;                //pitch dot
	_current_state(9, 0) = Eulerf(Quatf(_v_att.q)).theta();  //pitch - theta
	_current_state(10, 0) = _v_att.yawspeed;	               // yawdot
	_current_state(11, 0) = Eulerf(Quatf(_v_att.q)).psi();    //yaw - psi

}

void QuadrotorLQRControl::setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

	_current_state_ekf(0, 0) = _v_local_pos.vx;
	_current_state_ekf(1, 0) = _v_local_pos.x;
	_current_state_ekf(2, 0) = _v_local_pos.vy;
	_current_state_ekf(3, 0) = _v_local_pos.y;
	_current_state_ekf(4, 0) = _v_local_pos.vz;
	_current_state_ekf(5, 0) = _v_local_pos.z;

	_current_state_ekf(6, 0) = _v_att.rollspeed;
	_current_state_ekf(7, 0) = Eulerf(Quatf(_v_att.q)).phi();
	_current_state_ekf(8, 0) = _v_att.pitchspeed;
	_current_state_ekf(9, 0) = Eulerf(Quatf(_v_att.q)).theta();
	_current_state_ekf(10, 0) = _v_att.yawspeed;
	_current_state_ekf(11, 0) = Eulerf(Quatf(_v_att.q)).psi();

}


void QuadrotorLQRControl::setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

	_eq_point(0, 0) = _v_local_pos.vx;
	_eq_point(1, 0) = _v_local_pos.x;
	_eq_point(2, 0) = _v_local_pos.vy;
	_eq_point(3, 0) = _v_local_pos.y;
	_eq_point(4, 0) = _v_local_pos.vz;
	_eq_point(5, 0) = _v_local_pos.z;

	_eq_point(6, 0) = _v_att.rollspeed;
	_eq_point(7, 0) = Eulerf(Quatf(_v_att.q)).phi();
	_eq_point(8, 0) = _v_att.pitchspeed;
	_eq_point(9, 0) = Eulerf(Quatf(_v_att.q)).theta();
	_eq_point(10, 0) = _v_att.yawspeed;
	_eq_point(11, 0) = Eulerf(Quatf(_v_att.q)).psi();


}

void QuadrotorLQRControl::setEquilibriumPoint(Matrix<float, 12, 1> eqPoint)
{

	_eq_point(0, 0) = eqPoint(0, 0);
	_eq_point(1, 0) = eqPoint(1, 0);
	_eq_point(2, 0) = eqPoint(2, 0);
	_eq_point(3, 0) = eqPoint(3, 0);
	_eq_point(4, 0) = eqPoint(4, 0);
	_eq_point(5, 0) = eqPoint(5, 0);

	_eq_point(6, 0) = eqPoint(6, 0);
	_eq_point(7, 0) = eqPoint(7, 0);
	_eq_point(8, 0) = eqPoint(8, 0);
	_eq_point(9, 0) = eqPoint(9, 0);
	_eq_point(10, 0) = eqPoint(10, 0);
	_eq_point(11, 0) = eqPoint(11, 0);



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

void QuadrotorLQRControl::writeStateOnFile(const char* filename, Matrix <float, 12, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);

	outfile << t << "\t";   // time

	for (int i = 0; i < 12; i++) {
		if (i == 11) {
			outfile << vect(i, 0) << "\n";
		}
		else {
			outfile << vect(i, 0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeRefOnFile(const char* filename, Matrix <float, 3, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);

	outfile << t << "\t";   // time

	for (int i = 0; i < 3; i++) {
		if (i == 2) {
			outfile << vect(i, 0) << "\n";
		}
		else {
			outfile << vect(i, 0) << "\t";
		}
	}
	outfile.close();
	return;
}


void QuadrotorLQRControl::writeInputOnFile(const char* filename, Matrix <float, 4, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);

	outfile << t << "\t";   // time

	for (int i = 0; i < 4; i++) {
		if (i == 3) {
			outfile << vect(i, 0) << "\n";
		}
		else {
			outfile << vect(i, 0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeLyapunovOnFile(const char* filename, float value, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);

	outfile << t << "\t" << value << "\n";
	outfile.close();
	return;
}

Matrix <float, 12, 12>  QuadrotorLQRControl::readMatrixP(const char* filename)
{

	static Matrix <float, 12, 12> result;
	static int rows = 12;
	static int cols = 12;
	ifstream infile;
	infile.open(filename);
	if (infile.is_open()) {
		for (int i = 0; i < rows; i++) {
			string line;
			getline(infile, line);
			stringstream stream(line);
			for (int j = 0; j < cols; j++) {
				stream >> result(i, j);
			}

		}
		infile.close();
	}
	else cout << "Unable to open file";
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

Matrix<float, 3, 1> QuadrotorLQRControl::generateRef(float time)
{
	//cout << "time ref:" << time << endl; 
	float r = 2;
	float h = 10;

	float time_stop = 2;

	if (time < time_stop) {
		_ref_points(0, 0) = 0;
		_ref_points(1, 0) = 0;
		_ref_points(2, 0) = -1;

		cout << "rising" << endl;
	}
	else {
		cout << "helix" << endl;
		_ref_points(0, 0) = r * cos(time_constant*(time-time_stop)) - r;
		_ref_points(1, 0) = r * sin(time_constant*(time-time_stop));
		//_ref_points(2, 0) = -1 - time_constant * (time-time_stop);
		_ref_points(2, 0) = -1;

		//_ref_points(0, 0) = -2;
		//_ref_points(1, 0) = -2;
		//_ref_points(2, 0) = -2;

	}



}