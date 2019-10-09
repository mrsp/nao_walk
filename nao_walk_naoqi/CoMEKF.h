/*
 * humanoid_state_estimation - a complete state estimation scheme for humanoid robots
 *
 * Copyright 2017-2018 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef __COMEKF_H__
#define __COMEKF_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "butterworthLPF.h"
using namespace Eigen;
using namespace std;

class CoMEKF {

private:

	Matrix<float, 9, 9> F, P, I, Q, Fd;

	Vector3f COP, fN, L, COP_p, fN_p, L_p, Acc;

	Matrix<float, 6, 9> H;

	Matrix<float, 9, 6> K;

	Matrix<float, 6, 6> R, S;

	butterworthLPF *DCMx_LPF, *DCMy_LPF;
	Matrix<float, 6, 1> z;
	
	float tmp;
	

	void euler(Vector3f COP_, Vector3f fN_, Vector3f L_);
	Matrix<float,9,1> computeDyn(Matrix<float,9,1> x_, Vector3f COP_, Vector3f fN_, Vector3f L_);
	Matrix<float,9,9> computeTrans(Matrix<float,9,1> x_,  Vector3f COP_, Vector3f fN_, Vector3f L_);
	void RK4(Vector3f COP_, Vector3f fN_, Vector3f L_, Vector3f COP0, Vector3f fN0, Vector3f L0);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Matrix<float, 9, 1> x, f;

	float comd_q, com_q, fd_q, com_r, comdd_r;

	float dt, m, g, I_xx,I_yy;

    float bias_fx, bias_fy, bias_fz;
	bool firstrun;
	bool useEuler;
	void init();
	void updateVars();


    Vector2f DCM,DCMdot,ZMP, DCM_lp;
	void setdt(float dtt) {
		dt = dtt;
		DCMx_LPF->init("DCMx",1.0/dt,4.0);
		DCMy_LPF->init("DCMy",1.0/dt,4.0);
	}

	void setParams(float m_, float I_xx_, float I_yy_, float g_)
	{
		m = m_;
		I_xx = I_xx_;
		I_yy = I_yy_;
		g = g_;

	}

	void setCoMPos(Vector3f pos) {
		x(0) = pos(0);
		x(1) = pos(1);
		x(2) = pos(2);
	}
	void setCoMExternalForce(Vector3f force) {
		x(6) = force(0);
		x(7) = force(1);
		x(8) = force(2);
	}

	void predict(Vector3f COP_, Vector3f fN_, Vector3f L_);
	void update(Vector3f Acc_, Vector3f Pos_, Vector3f Gyro_, Vector3f Gyrodot_);

	float comX, comY, comZ, velX, velY, velZ, fX,
			fY, fZ, accX, accY, accZ;

};

#endif