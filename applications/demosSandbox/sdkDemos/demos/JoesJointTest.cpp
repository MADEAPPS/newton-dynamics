/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"
#include <dCustomBallAndSocket.h>

#define N_PI dFloat(3.1415926535897932384626433832795)






/////////////////////////////////////////


#define ZERO_MASS_BASES					// static bodies at the base so stuff is fixed   

//#define DBG_FORCE_LIMIT_MOTOR			// force moter on for all joints

//#define JULY							// to compare with older version of Newton



/////////////////////////////////////////


//#define AVERAGE_CONSTRAINT_SPACE		// constraint space is average rotation of both parent and child (enable limit visualization to see how it rotates)
										// intended to avoid gimbal lock issues, e.g. twist mapped to swing if angle is 90 degrees.
										// but it does not work - arc gets twisted - fixed this but still causes the effect of external force due to wobbeling limit... still thinking of it but you can ignore this.

//#define ENABLE_ERROR_ALIGNED_P2P		// align point to point constraint space to displacement error, but no difference. ignore it.


static NewtonBody* CreateBox (DemoEntityManager* const scene, const dVector& location, const dVector& size, const dFloat mass = 1)
{
    NewtonWorld* const world = scene->GetNewton();
    int materialID =  NewtonMaterialGetDefaultGroupID (world);
    NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
   	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");
    
    dMatrix matrix (dGetIdentityMatrix());
    matrix.m_posit = location;
    matrix.m_posit.m_w = 1;
    NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

    geometry->Release();
    NewtonDestroyCollision(collision);
    return body;
}



class JoesRagdollJoint: public dCustomBallAndSocket
{
	struct Vis
	{
		static void Line (dDebugDisplay* const debugDisplay, dVector p0, dVector p1, dFloat r, dFloat g, dFloat b, dMatrix *frame = 0)
		{
			if (frame)
			{
				p0 = frame->TransformVector (p0);
				p1 = frame->TransformVector (p1);
			}
			dVector color (r,g,b,1);
	#ifdef JULY
			debugDisplay->SetColor(0, color);
			debugDisplay->DrawLine (0, p0, p1);
	#else
			debugDisplay->SetColor(color);
			debugDisplay->DrawLine (p0, p1);
	#endif
		}

		static void Vector (dDebugDisplay* const debugDisplay, dVector o, dVector v, dFloat r, dFloat g, dFloat b, dMatrix *frame = 0)
		{
			dVector p0 = o;
			dVector p1 = o + v;
			Line (debugDisplay, p0, p1, r,g,b, frame);
		}

		static dVector ToCartesian (dVector s)
		{
			dFloat r = s.m_x;
			dFloat p = s.m_y;
			dFloat t = s.m_z;

			return dVector (
				r*dCos(p) * dSin(t),
				r*dSin(p) * dSin(t),
				r*dCos(t), 
				0);
		}
	};

	public:
	dQuaternion m_target; // relative target rotation to reach at next timestep

	dFloat m_maxTorque;

	// motor:
	dFloat m_reduceError;
	dFloat m_pin_length;
	dFloat m_stiffness;

	dFloat m_anim_speed;
	dFloat m_anim_offset;
	dFloat m_anim_time;

	// limits:
	bool m_isLimitJoint;
	bool m_isMotorizedLimitJoint;

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_coneAngle;
	dFloat m_arcAngle;
	dFloat m_arcAngleCos;
	dFloat m_arcAngleSin;

//	const dFloat E = dFloat(1.0E-16);
//	const dFloat E_angle = dFloat(1.0e-10);

	#define E		dFloat(1.0E-16)
	#define E_angle dFloat(1.0e-10)

	
	JoesRagdollJoint(NewtonBody* child, NewtonBody* parent, const dMatrix &localMatrix0, const dMatrix &localMatrix1, NewtonWorld *world, bool isLimitJoint = false, bool isMotorizedLimitJoint = false)
		:dCustomBallAndSocket(localMatrix0, localMatrix1, child, parent)
	{
		m_localMatrix0 = localMatrix0;
		m_localMatrix1 = localMatrix1;

		m_target = dQuaternion(dVector(1, 0, 0), 0);
		m_reduceError = dFloat(0.95); // amount of error to reduce per timestep (more -> oszillation)
		m_stiffness = dFloat(0.9f);
		m_maxTorque = dFloat(1000);

		//dAssert (m_maxTorque >= m_maxMotorTorque);

		m_anim_speed = 0;
		m_anim_offset = 0;
		m_anim_time = 0;
		
		m_isLimitJoint = isLimitJoint || isMotorizedLimitJoint;
		m_isMotorizedLimitJoint = isMotorizedLimitJoint;

#ifdef DBG_FORCE_LIMIT_MOTOR
		m_isLimitJoint = 1;
		m_isMotorizedLimitJoint = 1;
#endif
		
		m_coneAngle = dFloat(2.8);
#ifdef AVERAGE_CONSTRAINT_SPACE
		m_coneAngle /= 2;
		m_arcAngle /= 2;
#endif
		m_arcAngle = -1; // negative number: simple cone and no arc 
		m_arcAngleCos = dCos (0);
		m_arcAngleSin = dSin (0);
		m_minTwistAngle = 1; m_maxTwistAngle = -1; // unrestricted
	}


	void SetTwistSwingLimits (const dFloat coneAngle, const dFloat arcAngle, const dFloat minTwistAngle, const dFloat maxTwistAngle)
	{
		dFloat const maxAng = dFloat(2.8); // to prevent flipping on the pole on the backside

		m_coneAngle = dMin (maxAng, coneAngle);
		m_arcAngle = dMax (dFloat(0), dMin (maxAng, arcAngle + m_coneAngle) - m_coneAngle);
#ifdef AVERAGE_CONSTRAINT_SPACE
		m_coneAngle /= 2;
		m_arcAngle /= 2;
#endif
		m_arcAngleCos = dCos (m_arcAngle);
		m_arcAngleSin = dSin (m_arcAngle);

		m_minTwistAngle = minTwistAngle;
		m_maxTwistAngle = maxTwistAngle;
	}

	dVector BodyGetPointVelocity (const NewtonBody* const body, const dVector &point) const
	{
		dMatrix matrix;
		dVector v(0,0,0,0);
		dVector w(0,0,0,0);
		dVector c(0,0,0,0);
		NewtonBodyGetVelocity(body, &v[0]);
		NewtonBodyGetOmega(body, &w[0]);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		c = matrix.m_posit;									// TODO: Does not handle COM offset !!!!!!!!!!!!!!!!!!!!!!
		return v + w.CrossProduct(point - c);
	}

	dFloat XAngle (const dQuaternion &q0, const dQuaternion &q1) const
	{
		dFloat *q0p = (dFloat*)&q0;
		dFloat *q1p = (dFloat*)&q1;
		// factor rotation about x axis between quat0 and quat1. Code is an optimization of this: qt = q0.Inversed() * q1; halfTwistAngle = atan (qt.x / qt.w);
		return 2 * dAtan (
			( ( ( (q0p[0] * q1p[1]) + (-q0p[1] * q1p[0]) ) + (-q0p[2] * q1p[3]) ) - (-q0p[3] * q1p[2]) ) /
			( ( ( (q0p[0] * q1p[0]) - (-q0p[1] * q1p[1]) ) - (-q0p[2] * q1p[2]) ) - (-q0p[3] * q1p[3]) ) );
	}




	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		SubmitConstraints(timestep, threadIndex, 0);
	}


	void SubmitConstraints(dFloat timestep, int threadIndex, dDebugDisplay* const debugDisplay)
	{
		const bool vis = (debugDisplay != 0);

		dFloat invTimestep = 1 / timestep;

		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix(matrix0, matrix1);

		dQuaternion q0(matrix0);
		dQuaternion q1(matrix1);      

#ifdef AVERAGE_CONSTRAINT_SPACE
		dQuaternion qAV;// = q0.Slerp(q1, dFloat(0.5f) ); // lerp instead, requires no epsilon:
		{
			dFloat t (0.5f);
			dFloat s (1.0f - 0.5f);
			if (q0.DotProduct(q1) < 0) s *= -1;
			qAV.m_w = t*q1.m_w + s*q0.m_w;
			qAV.m_x = t*q1.m_x + s*q0.m_x;
			qAV.m_y = t*q1.m_y + s*q0.m_y;
			qAV.m_z = t*q1.m_z + s*q0.m_z;
			qAV.Normalize();

			//dFloat unTwistAngle = XAngle (q0, qAV);
			//dQuaternion unRot (dVector(1,0,0,0), unTwistAngle);
			//qAV = unRot * qAV; // prevent arc from twisting
		}
		dMatrix matrixCS (qAV, dVector(0,0,0,1));
#else
		dMatrix &matrixCS = matrix0;
#endif

		// constrain position
		{
#ifdef ENABLE_ERROR_ALIGNED_P2P
			const dVector& p0 = matrix0.m_posit;
			const dVector& p1 = matrix1.m_posit;

			dMatrix matrixCS_P2P;
			dVector alignTo = p0-p1;
			if ((alignTo.DotProduct3(alignTo)) < E) matrixCS_P2P = matrix0;
			else matrixCS_P2P = dGrammSchmidt(alignTo);

			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrixCS_P2P.m_front[0]);
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrixCS_P2P.m_up[0]);
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrixCS_P2P.m_right[0]);
#else
			//dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
			const dVector& p0 = matrix0.m_posit;
			const dVector& p1 = matrix1.m_posit;
			if (!vis) for (int i = 0; i < 3; i++) {
				NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1[i][0]);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			}
#endif
		}



		// calculate motor
		dVector angAcc (0,0,0,0);
		dVector angAcc2 (0,0,0,0);
		dVector errorAxis = matrix0.m_front;
		{
#if 1
			if (m_anim_speed != 0) // some animation to illustrate purpose
			{
				m_anim_time += timestep * m_anim_speed;
				dFloat a0 = dSin(m_anim_time);
				dFloat a1 = m_anim_offset * N_PI + m_anim_time / 3 + N_PI/4;
				dVector axis(dSin(a1), 0, dCos(a1));
				m_target = dQuaternion(axis, a0 / 2);
			}
#elif 1
			{ // test twist
				m_anim_time += timestep * 2;
				dFloat a0 = dSin(m_anim_time);
				dVector axis(1,0,0);
				m_target = dQuaternion(axis, a0 * 2);
			}
#elif 1
			{ // test swing
				m_anim_time += timestep * 2;
				dFloat a0 = dSin(m_anim_time);
				dVector axis(0,1,0);
				m_target = dQuaternion(axis, a0 * 2);
			}
#endif

			// measure error
			dQuaternion qt0 = m_target * q1;
			dQuaternion qErr = ((q0.DotProduct(qt0) < 0)	
				? dQuaternion(-q0.m_w, q0.m_x, q0.m_y, q0.m_z) 
				: dQuaternion(q0.m_w, -q0.m_x, -q0.m_y, -q0.m_z)) * qt0;
			qErr.Normalize();

			dFloat errorAngle = 2 * dAcos(dClamp (qErr.m_w, dFloat(-1), dFloat(1)));

			if (errorAngle > E_angle) 
			{
				errorAxis = dVector (qErr.m_x, qErr.m_y, qErr.m_z, 0);
				errorAxis = errorAxis.Scale(1 / dSqrt(errorAxis.DotProduct3(errorAxis)));
			} 

			dVector angVel0(0,0,0,0);
			dVector angVel1(0,0,0,0);
			NewtonBodyGetOmega(m_body0, (dFloat*)&angVel0);
			NewtonBodyGetOmega(m_body1, (dFloat*)&angVel1);

			angAcc = (errorAxis.Scale(m_reduceError * errorAngle * invTimestep) - (angVel0 - angVel1)).Scale(invTimestep);
			angAcc2 = (errorAxis.Scale(m_reduceError * errorAngle * invTimestep)).Scale(invTimestep);
		}

if (vis) Vis::Vector (debugDisplay, matrix0.m_posit, errorAxis, 1,1,1);


		if (m_isLimitJoint)
		{
			


			const dVector& coneDir0 = matrixCS.m_front;
			const dVector& coneDir1 = matrix1.m_front;
			dFloat dot = coneDir0.DotProduct3(coneDir1);
			if (dot < dFloat(-0.999)) return; // todo: Assuming it's ok to not submit any rows in this case


			// claculate swing
			dVector swingAxis;
			dFloat swingErrorAngle = 0;
			{
				if (m_arcAngle <= 0) // simple cone limit
				{

					swingAxis = (coneDir0.CrossProduct(coneDir1));
					swingErrorAngle = dAcos (dClamp(dot, dFloat(-1), dFloat(1))) - m_coneAngle;

				}
				else // cone on arc limit - think of an piece of pizza (arc) and an allowed max distance from it (cone):
				{	

					// project current axis to the arc plane (y)
					dVector d = matrix1.UnrotateVector (matrixCS.m_front);
					dVector cone = d; cone.m_y = 0; 
					dFloat sql = cone.DotProduct3(cone);
					cone = (sql > E) ? cone.Scale (1 / dSqrt (sql)) : dVector(1, 0, 0, 0);

					// clamp the result to be within the arc angle
					if (cone.m_x < m_arcAngleCos)
						cone = dVector ( m_arcAngleCos, 0, ( (cone.m_z < 0) ? -m_arcAngleSin : m_arcAngleSin));
				
					// do a regular cone constraint from that
					swingErrorAngle = dAcos (dClamp(d.DotProduct3(cone), dFloat(-1), dFloat(1))) - m_coneAngle;

					swingAxis = matrix1.RotateVector(d.CrossProduct(cone));
				}
				dFloat sql = swingAxis.DotProduct3(swingAxis);
				if (sql > E) swingAxis =  swingAxis.Scale (1 / dSqrt (sql));
				else swingAxis = matrixCS.m_up;

				if (swingErrorAngle < 0) swingErrorAngle = 0;
			}

if (vis) Vis::Vector (debugDisplay, matrix0.m_posit, swingAxis, 1, swingErrorAngle==0, 0);

			// twist
			dVector twistAxis = coneDir0; // this is the only real change from the last version, which used (coneDir0+coneDir1).Unit() resulting in non orthogonal twist/swing axis
			
			// but still, sometimes it fails (not sure why) - force orthogonal:
			{
				twistAxis -= swingAxis.Scale(swingAxis.DotProduct3(twistAxis));
				dFloat sql = twistAxis.DotProduct3(twistAxis);
				if (sql > E) twistAxis = twistAxis.Scale (1 / dSqrt (sql));
				else twistAxis = coneDir0; // should never happen
			}

if (vis) Vis::Vector (debugDisplay, matrix0.m_posit, twistAxis, 0,1,1);

			{
				dFloat twistAngle = 0;
				{
#if 1
					// this does not account for a orthogonal fix, but still works best:

					twistAngle = XAngle (q0, q1);

#else
					// try to account for orthogonal fix (rotationAxis may not be exactly (1,0,0)), but just worse - i still don't understand what causes unorthogonality - ignore this

					dQuaternion qt = q1 * q0.Inverse();
					dVector rotationAxis (qt.m_x, qt.m_y, qt.m_z);
					dFloat rotationAngle = 2 * dAcos(dClamp(qt.m_w, dFloat(-1), dFloat(1)));
					dFloat sql = rotationAxis.DotProduct3(rotationAxis);
					if (sql > E)
					{
						rotationAxis = rotationAxis.Scale (1 / dSqrt (sql));
						twistAngle = rotationAngle * twistAxis.DotProduct3 (rotationAxis);
					}
#endif
				}

				dFloat minTourque = 0;
				dFloat maxTourque = 0;
				dFloat errorAngle = 0;
				if (1 && m_maxTwistAngle >= m_minTwistAngle) // twist restricted?
				{
					if (m_maxTwistAngle == m_minTwistAngle) // no freedom for any twist
					{
						errorAngle = twistAngle - m_maxTwistAngle;
						minTourque = -m_maxTorque;
						maxTourque = m_maxTorque;
					}
					else if (twistAngle > m_maxTwistAngle)
					{
						errorAngle = twistAngle - m_maxTwistAngle;
						maxTourque = m_maxTorque;
					}
					else if (twistAngle < m_minTwistAngle)
					{
						errorAngle = twistAngle - m_minTwistAngle;
						minTourque = -m_maxTorque;
					}
				}

				dVector &axis = twistAxis;
				if (m_isMotorizedLimitJoint)
				{
					if (!vis) {
						NewtonUserJointAddAngularRow(m_joint, 0, &axis[0]);

						dFloat restAcc = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
						dFloat limitAcc = dFloat(0.5) * errorAngle * invTimestep;
						dFloat motorAcc = angAcc2.DotProduct3(axis);
						if ((limitAcc>0 && motorAcc<0) || (limitAcc<0 && motorAcc>0)) motorAcc = 0; // turn off motor if working against limit
						dFloat targetAcc = restAcc + limitAcc + motorAcc;
						
						NewtonUserJointSetRowAcceleration(m_joint, targetAcc);
						NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
						NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);					
					}
				}
				else
				{
					if (!vis) {
						NewtonUserJointAddAngularRow (m_joint, errorAngle, (dFloat*)&axis);
						NewtonUserJointSetRowMinimumFriction (m_joint, minTourque);
						NewtonUserJointSetRowMaximumFriction (m_joint, maxTourque);
					}
				}
					if (!vis) NewtonUserJointSetRowStiffness(m_joint, m_stiffness);


			}

			// apply swing
			{


				if (m_isMotorizedLimitJoint)
				{

					dVector &axis = swingAxis;
					if (!vis) {
						NewtonUserJointAddAngularRow(m_joint, 0, &axis[0]);

						dFloat restAcc = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
						dFloat limitAcc = dFloat(0.5) * swingErrorAngle * invTimestep;
						dFloat motorAcc = angAcc2.DotProduct3(axis);
						if ((limitAcc>0 && motorAcc<0) || (limitAcc<0 && motorAcc>0)) motorAcc = 0; // turn off motor if working against limit
						dFloat targetAcc = restAcc + limitAcc + motorAcc;
						
						NewtonUserJointSetRowAcceleration(m_joint, targetAcc);
						NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
						NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);
					}
				}
				else
				{
					dFloat maxTourque = 0;	
					if (swingErrorAngle > 0) maxTourque = m_maxTorque;
					if (!vis) {
						NewtonUserJointAddAngularRow (m_joint, swingErrorAngle, (dFloat*)&swingAxis);
						NewtonUserJointSetRowMinimumFriction (m_joint, 0);
						NewtonUserJointSetRowMaximumFriction (m_joint, maxTourque);	
					}
				}
					if (!vis) NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			}

			dAssert (dAbs(twistAxis.DotProduct3(swingAxis)) < dFloat(0.001)); // todo: in theory it could happen swingAxis == twistAxis





			// final row if motor
			
if (vis) Vis::Vector (debugDisplay, matrix0.m_posit, swingAxis.CrossProduct(twistAxis), 0.5,1,0);

			// todo: 
			//
			// submitting this row only if motor turned on, otherwise only 5 rows but should be 6; create one more with no effect
			// 
			// problem: What's the limit of this 2nd perpendicular axis for the motor?
			// correct solution would bu to intersect the arc limit? probably not necessary; using simple cone limit:

			if (m_isMotorizedLimitJoint) 
			{
				dVector axis = swingAxis.CrossProduct(twistAxis);
				if (!vis) {
					NewtonUserJointAddAngularRow(m_joint, 0, &axis[0]);

					dFloat restAcc = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
					dFloat limitAcc = dFloat(0.5) * swingErrorAngle * invTimestep;
					dFloat motorAcc = angAcc2.DotProduct3(axis);
					if ((limitAcc>0 && motorAcc<0) || (limitAcc<0 && motorAcc>0)) motorAcc = 0; // turn off motor if working against limit
					dFloat targetAcc = restAcc + limitAcc + motorAcc;
						
					NewtonUserJointSetRowAcceleration(m_joint, targetAcc);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);
				}
			}
		}
		else // no limits but motor (reference approach i use for my ragdoll - not suited for inverse dynamics)
		{
			dMatrix basis = dGrammSchmidt(errorAxis);
			for (int n = 0; n < 3; n++) {
				// calculate the desired acceleration
				dVector &axis = basis[n];
				dFloat relAccel = angAcc.DotProduct3(axis);

				if (!vis) {
					NewtonUserJointAddAngularRow(m_joint, 0, &axis[0]);
					NewtonUserJointSetRowAcceleration(m_joint, relAccel);
					NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
					NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);
					NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
				}
			}
		}
		
	}

	void Debug(dDebugDisplay* const debugDisplay) const
	{
		//dCustomJoint::Debug(debugDisplay);



		if (1) // debug
		{
			((JoesRagdollJoint*)this)->SubmitConstraints(dFloat(1/60), -1, debugDisplay);
		}


		
		if (1) // vis limits
		{
			dMatrix matrix0;
			dMatrix matrix1;
			CalculateGlobalMatrix(matrix0, matrix1);

			dMatrix matrixCS;
			dMatrix matrixVIS;
			{

#ifdef AVERAGE_CONSTRAINT_SPACE
				dQuaternion q0(matrix0);
				dQuaternion q1(matrix1);      

				dQuaternion qAV;// = q0.Slerp(q1, dFloat(0.5f) ); // lerp instead, requires no epsilon:
				{
					dFloat t (0.5f);
					dFloat s (1.0f - 0.5f);
					if (q0.DotProduct(q1) < 0) s *= -1;
					qAV.m_w = t*q1.m_w + s*q0.m_w;
					qAV.m_x = t*q1.m_x + s*q0.m_x;
					qAV.m_y = t*q1.m_y + s*q0.m_y;
					qAV.m_z = t*q1.m_z + s*q0.m_z;
					qAV.Normalize();

					dFloat unTwistAngle = XAngle (q0, qAV);
					dQuaternion unRot (dVector(1,0,0,0), unTwistAngle);
					qAV = unRot * qAV; // prevent arc from twisting
				}
				matrixCS = dMatrix (qAV, matrix0.m_posit);
				matrixVIS = matrixCS;
#else
				matrixCS = matrix0;
				matrixVIS = matrix1;
#endif
			}



			int const subdiv = 36;
			dFloat const radius = 0.4f;

			dFloat yAngle = m_coneAngle;
			dFloat xAngle = dMax(m_arcAngle,dFloat(0));
			dFloat minTAngle = m_minTwistAngle;
			dFloat maxTAngle = m_maxTwistAngle;

			dFloat minXAngle = -xAngle; 
			dFloat maxXAngle = xAngle; 
			int i;

			// vis swing limit

			dVector ot = matrix1.m_posit;
			dVector ob = matrix1.m_posit;
			dMatrix ortho = dYawMatrix(N_PI * 0.5f - minXAngle) * matrixVIS;
			for (i=0; i<=subdiv/4; i++)
			{
				dFloat t = dFloat(i) / dFloat(subdiv);
				dVector c = Vis::ToCartesian (dVector (radius, N_PI * 2.0f * t, yAngle));
				dVector nt = ortho.TransformVector (c); Vis::Line (debugDisplay, nt, ot, 1,1,1); ot = nt;
				c[1] *= -1;
				dVector nb = ortho.TransformVector (c); Vis::Line (debugDisplay, nb, ob, 1,1,1); ob = nb;
			}

			ot = matrix1[3];
			ob = matrix1[3];
			ortho = dYawMatrix(N_PI * 0.5f - maxXAngle) * matrixVIS;
			for (i=subdiv/2; i>=subdiv/4; i--)
			{
				dFloat t = dFloat(i) / dFloat(subdiv);
				dVector c = Vis::ToCartesian (dVector (radius, N_PI * 2.0f * t, yAngle));
				dVector nt = ortho.TransformVector (c); Vis::Line (debugDisplay, nt, ot, 1,1,1); ot = nt;
				c[1] *= -1;
				dVector nb = ortho.TransformVector (c); Vis::Line (debugDisplay, nb, ob, 1,1,1); ob = nb;
			}	

			ot = matrix1[3];
			ob = matrix1[3];
			ortho = dPitchMatrix(N_PI * 0.5f) * dYawMatrix(-minXAngle) * matrixVIS;//matrix1 * sMat4::rotationY(-minXAngle) * sMat4::rotationX(N_PI * 0.5);
			dFloat angDiff = maxXAngle - minXAngle;
			int subdiv2 = 1 + int ((angDiff / N_PI * 180.0f / (360.0f / dFloat(subdiv))));
			for (i=0; i<=subdiv2; i++)
			{
				dFloat t = dFloat(i) / dFloat(subdiv2);
				dVector c = Vis::ToCartesian (dVector (radius, angDiff * t, N_PI * 0.5f - yAngle));
				dVector nt = ortho.TransformVector (c); Vis::Line (debugDisplay, nt, ot, 1,1,1); ot = nt;
				c[2] *= -1;
				dVector nb = ortho.TransformVector (c); Vis::Line (debugDisplay, nb, ob, 1,1,1); ob = nb;
			}	
			Vis::Line (debugDisplay, matrix1.m_posit, ot, 1,1,1);
			Vis::Line (debugDisplay, matrix1.m_posit, ob, 1,1,1);


			// vis twist limit

			ot = matrix1.m_posit;
			ortho = dYawMatrix(N_PI * -0.5f) * dPitchMatrix(N_PI * -0.5f) * matrix1;//matrix1 * sMat4::rotationX(N_PI * -0.5) * sMat4::rotationY(N_PI * -0.5);
			angDiff = maxTAngle - minTAngle;
			subdiv2 = 1 + int ((angDiff / N_PI * 180.0f / (360.f / dFloat(subdiv))));
			for (i=0; i<=subdiv2; i++)
			{
				dFloat t = dFloat(i) / dFloat(subdiv2);
				dVector c = Vis::ToCartesian (dVector (radius, angDiff * t + minTAngle, N_PI * 0.5f));
				dVector nt = ortho.TransformVector (c); Vis::Line (debugDisplay, nt, ot, 1,1,0); ot = nt;
			}	
			Vis::Line (debugDisplay, matrix1[3], ot, 1,1,0);

			dQuaternion quat0(matrix0), quat1(matrix1);      
			dFloat *q0 = (dFloat*)&quat0;
			dFloat *q1 = (dFloat*)&quat1;
			dFloat twistAngle = 2.0f * dFloat (atan (
				( ( ( (q0[0] * q1[1]) + (-q0[1] * q1[0]) ) + (-q0[2] * q1[3]) ) - (-q0[3] * q1[2]) ) /
				( ( ( (q0[0] * q1[0]) - (-q0[1] * q1[1]) ) - (-q0[2] * q1[2]) ) - (-q0[3] * q1[3]) ) ));

			Vis::Vector (debugDisplay, matrix1[3], matrix1.RotateVector(dVector(0,1,0).Scale(radius)), 0,1,0); // zero twist
			Vis::Vector (debugDisplay, matrix1[3], matrix1.RotateVector(dVector(0, dCos(twistAngle), dSin(-twistAngle)).Scale(radius)), 1,0,0); // current twist
			Vis::Vector (debugDisplay, matrix0[3], matrix0[0].Scale(radius), 0.5f,0.5f,1); // current pin
		}
	}
};

static inline dFloat randF (unsigned int time)
{
	time *= 1664525;
	time ^= (time << 16);
	time *= 16807;
	return dFloat(time & 0xFFFFFFFFu) / dFloat(0xFFFFFFFFu);
}

void AddJoesPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin, const dFloat animSpeed, const int numSegments,
	const int numArms = 1,
	const dFloat torsoHeight = 1, 
	const dFloat torsoWidth = 4.0f, 
	const dFloat randomness = 0, 
	const dFloat armHeight = 1, 
	const dFloat armWidth = 0.5f,
	const int pickMe = -1)
{
    dFloat height = torsoHeight;
    dFloat width = torsoWidth;

    dVector size (width, height, width);
    NewtonBody* torso = CreateBox (scene, origin + dVector (0,  0.5f, 0, 0), size
#ifdef ZERO_MASS_BASES
	,0
#endif
	);
	dMatrix torsoMatrix; 
	NewtonBodyGetMatrix (torso, (dFloat*) &torsoMatrix);

	int bodyIndex = 0;
	NewtonBody* pickBody = 0;
	for (int j=0; j < numArms; j++)
	{
		dFloat angle = dFloat(j) / dFloat(numArms) * N_PI*2.0f;
		dMatrix armRotation = dPitchMatrix(angle);
		dMatrix armTransform = armRotation * torsoMatrix;
		
		NewtonBody* parent = torso;

		int numBodies = numSegments;
		if (randomness > 0) numBodies += int (randF(j) * dFloat(numSegments) + 0.5f);
		for (int i=0; i < numBodies; i++)
		{
			dFloat height = armHeight;
			dFloat width = armWidth;

			dVector size (width, height, width);
			dVector pos (0,  height * dFloat(i + (numArms>1 ? 2 : 1)), 0, 0);
			NewtonBody* child = CreateBox (scene, pos, size);
			
			dMatrix bodyMatrix; 
			NewtonBodyGetMatrix (child, (dFloat*) &bodyMatrix);
			bodyMatrix = bodyMatrix * armTransform;
			NewtonBodySetMatrix (child, (dFloat*) &bodyMatrix);

			dMatrix matrix0 = dGetIdentityMatrix(); matrix0.m_posit = dVector (0, height*-0.5f, 0, 1);
			dMatrix matrix1 = dGetIdentityMatrix(); matrix1.m_posit = dVector (0, height*0.5f, 0, 1);
			if (parent == torso) 
			{
				matrix1.m_posit.m_y += height;
				matrix1 = matrix1 * armRotation;
			}
			if (randomness > 0)
			{
				dMatrix rotation =  dPitchMatrix(randF(bodyIndex*3+0) * N_PI * 0.25f * randomness);
				rotation = rotation * dYawMatrix(randF(bodyIndex*3+1) * N_PI * 0.25f * randomness);
				rotation = rotation * dYawMatrix(randF(bodyIndex*3+2) * N_PI * 0.25f * randomness);
				matrix0 = matrix0 * rotation;
			}
			JoesRagdollJoint* joint = new JoesRagdollJoint (child, parent, matrix0, matrix1, scene->GetNewton(), animSpeed >= 0, animSpeed > 0);

			if (animSpeed != 0) {
				joint->m_anim_speed = dAbs(animSpeed), joint->m_anim_offset = dFloat(i) / dFloat(numBodies); // animated      
			}

			parent = child;
			if (bodyIndex == pickMe) {
				pickBody = child;
			}
			bodyIndex++;
		}
	}

	if (pickBody)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(pickBody, &matrix[0][0]);
		new dCustomBallAndSocket(matrix, pickBody);
	}
}

void AddJoesLimitJoint (DemoEntityManager* const scene, const dVector& origin, const int testCase, const dFloat animSpeed = 0)
{
	NewtonBody* shoulder =	CreateBox (scene, origin + dVector (-0.5f,  2.0f, 0, 0), dVector (0.5f,  0.5f, 0.5f, 0)
#ifdef ZERO_MASS_BASES
	,0
#endif
	);
	NewtonBody* bicep =		CreateBox (scene, origin + dVector ( 0,  2.0f, 0, 0), dVector (1,  0.25f, 0.25f, 0));


	dMatrix localMatShoulder0 = dGetIdentityMatrix(); 
	if (testCase == -1) localMatShoulder0 = dPitchMatrix(N_PI*-0.5f) * dYawMatrix(N_PI*-0.25f) * dRollMatrix(N_PI*-0.15f); 
	localMatShoulder0.m_posit = dVector (0.25f, 0, 0, 1);

	dMatrix localMatShoulder1 = dGetIdentityMatrix(); 
	localMatShoulder1.m_posit = dVector (-0.5f, 0, 0, 1);

	dMatrix localMatBicep0 = dYawMatrix(N_PI*-0.5f);   
	localMatBicep0.m_posit = dVector (0.5f, 0, 0, 1);

	dMatrix localMatBicep1 = dGetIdentityMatrix();
	localMatBicep1.m_posit = dVector (-0.5f, 0, 0, 1);

	

	JoesRagdollJoint* shoulderJoint = new JoesRagdollJoint (bicep, shoulder, localMatShoulder1, localMatShoulder0, scene->GetNewton(), true, animSpeed > 0);
	shoulderJoint->m_anim_speed = animSpeed;

	switch (testCase)
	{
		case 1: 
			shoulderJoint->SetTwistSwingLimits (0, N_PI/2, 0, 0); // just arc (hinge; robust for knees and elbows, unsure if traditional approach is better)
			break;
		case 2: 
			shoulderJoint->SetTwistSwingLimits (N_PI/4*3, 0, 0, 0); // large swing cone to show robust twist (the larger the swing angle becomes, the more rotation happens around its axis: Drag bicep back and then around to see) 
			break;
		case 3: 
			shoulderJoint->SetTwistSwingLimits (N_PI/4, 0, 0, 0); // small swing cone
			break;
		case 4: 
			shoulderJoint->SetTwistSwingLimits (0, 0, -N_PI/2, N_PI/2); // no swing (hinge, but neither practical nor robust)
			break;
		default:
			shoulderJoint->SetTwistSwingLimits (N_PI*dFloat(0.2), N_PI*dFloat(0.1), -N_PI*dFloat(0.5), N_PI*dFloat(0.2)); // natural limit
	}

	if (testCase == -1)
	{
		NewtonBody* arm = CreateBox (scene, origin + dVector (1,2,0,0), dVector (1, dFloat(0.2f), dFloat(0.2f), 0));
		JoesRagdollJoint* ellbowJoint = new JoesRagdollJoint (arm, bicep, localMatBicep1, localMatBicep0, scene->GetNewton(), true);
		ellbowJoint->SetTwistSwingLimits (0, N_PI*dFloat(0.4), 0, 0);
	}

}



void JoesJointTest (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (dGetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "flatPlane1.ngd", 1);

    dVector location (0,0,0,0);
    dVector size (1.5f, 2.0f, 2.0f, 0);

	dFloat animSpeed = 4;

#if 1	
	AddJoesPoweredRagDoll(scene, dVector(-30.0f, 2.0f, -2.0f), -animSpeed, 4); // old motor with error aligned constraint space
	AddJoesPoweredRagDoll(scene, dVector(-30.0f, 2.0f, -10.0f), animSpeed, 4); // new motor with fixed constraint space suitable for inverse dynamics
#endif

// limits
#if 1
	AddJoesLimitJoint (scene, dVector(-40.0f, 0,  2.5f), -1);
	AddJoesLimitJoint (scene, dVector(-40.0f, 0, -0), 0);
	AddJoesLimitJoint (scene, dVector(-40.0f, 0, -2.5f), 1);
	AddJoesLimitJoint (scene, dVector(-40.0f, 0, -5.0f), 2);
	AddJoesLimitJoint (scene, dVector(-40.0f, 0, -7.5f), 3);
	AddJoesLimitJoint (scene, dVector(-40.0f, 0, -10.f), 4);
#endif

// limits and motors (does not look cool but it's stable)
#if 1
	AddJoesLimitJoint (scene, dVector(-20.0f, 0,  2.5f), -1, animSpeed);
	AddJoesLimitJoint (scene, dVector(-20.0f, 0, -0), 0, animSpeed);
	AddJoesLimitJoint (scene, dVector(-20.0f, 0, -2.5f), 1, animSpeed);
	AddJoesLimitJoint (scene, dVector(-20.0f, 0, -5.0f), 2, animSpeed);
	AddJoesLimitJoint (scene, dVector(-20.0f, 0, -7.5f), 3, animSpeed);
	AddJoesLimitJoint (scene, dVector(-20.0f, 0, -10.f), 4, animSpeed);
#endif

    // place camera into position
    dMatrix camMatrix (dGetIdentityMatrix());
    dQuaternion rot (camMatrix);
    dVector origin (-50.0f, 5.0f, -4.0f, 0);
    scene->SetCameraMatrix(rot, origin);
}
