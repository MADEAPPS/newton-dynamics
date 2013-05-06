/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple 4d vector class
//********************************************************************

#include <toolbox_stdafx.h>
#include "RenderPrimitive.h"
#include "Custom6DOF.h"
#include "CustomGear.h"
#include "CustomHinge.h"
#include "CustomPulley.h"
#include "CustomSlider.h"
#include "CustomWormGear.h"
#include "CustomUniversal.h"
#include "CustomCorkScrew.h"
#include "CustomBallAndSocket.h"
#include "JointLibrary.h"

#include "../NewtonDemos.h"
#include "../PhysicsUtils.h"
#include "../ToolBox/MousePick.h"
#include "../ToolBox/OpenGlUtil.h"
#include "../ToolBox/DebugDisplay.h"
#include "../ToolBox/LevelPrimitive.h"
#include "../ToolBox/PlaneCollision.h"
#include "../ToolBox/HeightFieldPrimitive.h"
#include "../ToolBox/UserHeightFieldCollision.h"



#define CHIKEN_LENGTH		2.5f
#define CHIKEN_HEIGHT		0.5f
#define CHIKEN_WIDTH		1.0f
#define MIN_JOINT_PIN_LENGTH 50.0f

class SixLegedRobot : public RenderPrimitive
{

	class SixLegedRobotCalf: public RenderPrimitive
	{

		class ThighEngine: public CustomHinge
		{
			public:
			ThighEngine(SixLegedRobotCalf* me, const dMatrix& pinAndPivox, NewtonBody* child, NewtonBody* parent, dFloat side)
				:CustomHinge(pinAndPivox, child, parent)
			{
				dFloat sinAngle;
				dFloat cosAngle;
				dMatrix matrix0;
				dMatrix matrix1;

				m_parent = me;
				// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
				CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

				
				// the joint angle can be determine by getting the angle between any two non parallel vectors
				sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
				cosAngle = matrix0.m_up % matrix1.m_up;
				m_baseAngle = dAtan2 (sinAngle, cosAngle) - side * 60.0f * 3.1416f / 180.0f;
			}

			virtual void SubmitConstrainst (dFloat timestep, int threadIndex)
			{
				dFloat angle;
				dFloat sinAngle;
				dFloat cosAngle;
				dFloat relAngle;
				dMatrix matrix0;
				dMatrix matrix1;

				// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
				CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

				// Restrict the movement on the pivot point along all tree orthonormal direction
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
				
				// get a point along the pin axis at some reasonable large distance from the pivot
				dVector q0 (matrix0.m_posit + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
				dVector q1 (matrix1.m_posit + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

				// two constraints row perpendicular to the pin vector
 				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

					
				// the joint angle can be determine by getting the angle between any two non parallel vectors
				sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
				cosAngle = matrix0.m_up % matrix1.m_up;
				angle = dAtan2 (sinAngle, cosAngle);

				relAngle = angle - m_baseAngle - m_parent->m_jointAngle;
		
				// tell joint error will minimize the exceeded angle error
				NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);
			}

			dFloat m_baseAngle;
			SixLegedRobotCalf *m_parent;
		};

		public:
		SixLegedRobotCalf (dSceneNode* parent, NewtonWorld* nWorld, const NewtonBody* parentBody, const dMatrix& matrix, dFloat radius, dFloat length, dFloat side)
			:RenderPrimitive (parent, matrix, NULL)
		{
			// crate the physics for this vehicle
			NewtonBody* body;
			NewtonCollision* collision;
			dGeometry* meshInstance;

			collision = NewtonCreateCapsule (nWorld, radius, length, NULL);

			meshInstance = new dGeometry (collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
			SetGeometry(meshInstance);
			meshInstance->Release();

			dFloat mass = 10.0f;
			dVector size (radius * 2, radius * 2, length);
			dFloat Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
			dFloat Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
			dFloat Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

			// ////////////////////////////////////////////////////////////////
			//
			//create the rigid body

			body = NewtonCreateBody (nWorld, collision);

			// make sure body parts do not collide
			int defaultID;
			defaultID = NewtonMaterialGetDefaultGroupID (nWorld);
			NewtonBodySetMaterialGroupID (body, defaultID);


			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (body, this);

			// set the transform call back function
			NewtonBodySetTransformCallback (body, SetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);

			// set the mass matrix
			NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (body, &GetMatrix()[0][0]);

			// release the collision 
			NewtonReleaseCollision (nWorld, collision);	


	//		dVector pin (matrix.m_right);
	//		dVector pivot (matrix.m_posit - matrix.m_front.Scale(side * length * 0.5f));
			dMatrix pinAndPivot (matrix);
			pinAndPivot.m_front = matrix.m_right;
			pinAndPivot.m_up = matrix.m_up;
			pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
			pinAndPivot.m_posit = matrix.m_posit - matrix.m_front.Scale(side * length * 0.5f);
			m_engine = new ThighEngine (this, pinAndPivot, body, (NewtonBody*)parentBody, side);
		}

		~SixLegedRobotCalf()
		{
		}

		// Set the vehicle matrix and all tire matrices
		static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr, int threadIndex)
		{
			SixLegedRobot* vehicle;

			// get the graphic object form the rigid body
			vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

			// set the transformation matrix for this rigid body
			dMatrix& matrix = *((dMatrix*)matrixPtr);
			vehicle->SetMatrix (matrix);
		}

		void SetAngle(dFloat angle)
		{
			m_jointAngle = angle;
		}

		dFloat m_jointAngle;
		ThighEngine* m_engine; 
	};


	class SixLegedRobotThight: public RenderPrimitive
	{
		class ThighEngine: public CustomHinge
		{
			public:
			ThighEngine (SixLegedRobotThight* me, const dMatrix& pinAndPivox, NewtonBody* child, NewtonBody* parent, dFloat side)
				:CustomHinge (pinAndPivox, child, parent)
			{
				dFloat sinAngle;
				dFloat cosAngle;
				dMatrix matrix0;
				dMatrix matrix1;

				m_parent = me;
				// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
				CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

				
				// the joint angle can be determine by getting the angle between any two non parallel vectors
				sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
				cosAngle = matrix0.m_up % matrix1.m_up;
				m_baseAngle = dAtan2 (sinAngle, cosAngle) + side * 90.0f * 3.1416f / 180.0f;
			}

			virtual void SubmitConstrainst (dFloat timestep, int threadIndex)
			{
				dFloat angle;
				dFloat sinAngle;
				dFloat cosAngle;
				dFloat relAngle;
				dMatrix matrix0;
				dMatrix matrix1;

				// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
				CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

				// Restrict the movement on the pivot point along all tree orthonormal direction
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
				NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
				
				// get a point along the pin axis at some reasonable large distance from the pivot
				dVector q0 (matrix0.m_posit + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
				dVector q1 (matrix1.m_posit + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

				// two constraints row perpendicular to the pin vector
 				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

					
				// the joint angle can be determine by getting the angle between any two non parallel vectors
				sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
				cosAngle = matrix0.m_up % matrix1.m_up;
				angle = dAtan2 (sinAngle, cosAngle);

				//relAngle = angle - m_baseAngle;
				relAngle = angle + m_parent->m_jointAngle - m_baseAngle ;
		
				// tell joint error will minimize the exceeded angle error
				NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);
			}

			dFloat m_baseAngle;
			SixLegedRobotThight* m_parent;
		};


		public:
		SixLegedRobotThight (dSceneNode* parent, NewtonWorld* nWorld, const NewtonBody* parentBody, const dMatrix& matrix, dFloat radius, dFloat length, dFloat side)
			:RenderPrimitive (parent, matrix, NULL)
		{
			// crate the physics for this vehicle
			NewtonBody* body;
			NewtonCollision* collision;
			dGeometry* meshInstance;

			collision = NewtonCreateCapsule (nWorld, radius, length, NULL);

			meshInstance = new dGeometry (collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
			SetGeometry(meshInstance);
			meshInstance->Release();

			m_side = side;
			m_jointAngle = 0.0f;
			dFloat mass = 10.0f;
			dVector size (radius * 2, radius * 2, length);
			dFloat Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
			dFloat Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
			dFloat Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

			// ////////////////////////////////////////////////////////////////
			//
			//create the rigid body

			body = NewtonCreateBody (nWorld, collision);

			// make sure body parts do not collide
			int defaultID;
			defaultID = NewtonMaterialGetDefaultGroupID (nWorld);
			NewtonBodySetMaterialGroupID (body, defaultID);

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (body, this);

			// set the transform call back function
			NewtonBodySetTransformCallback (body, SetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);

			// set the mass matrix
			NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (body, &GetMatrix()[0][0]);

			// release the collision 
			NewtonReleaseCollision (nWorld, collision);	


	//		dVector pin (matrix.m_right);
	//		dVector pivot (matrix.m_posit - matrix.m_front.Scale(side * length * 0.5f));
			dMatrix pinAndPivot (matrix);
			pinAndPivot.m_front = matrix.m_right;
			pinAndPivot.m_up = matrix.m_up;
			pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
			pinAndPivot.m_posit = matrix.m_posit - matrix.m_front.Scale(side * length * 0.5f);
			m_engine = new ThighEngine (this, pinAndPivot, body, (NewtonBody*)parentBody, side);

			//
			dMatrix matrix1 (matrix);
			matrix1.m_posit += matrix.m_front.Scale (side * length * 0.5f);

			matrix1 = dRollMatrix(-side * ((0.5f + 0.15f) * 3.1416f)) * matrix1;

			length *= 1.0f;
			matrix1.m_posit += matrix1.m_front.Scale (side * length * 0.5f);

			m_calf = new SixLegedRobotCalf (parent, nWorld, body, matrix1, radius, length, side); 

		}

		~SixLegedRobotThight()
		{
		}

		void SetAngle(dFloat angle)
		{
			m_jointAngle = angle + 0.25f * 3.1416f * m_side;
			m_calf->SetAngle (angle * 0.5f + 0.25f * 3.1416f * m_side);
		}

		// Set the vehicle matrix and all tire matrices
		static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr, int threadIndex)
		{
			SixLegedRobot* vehicle;

			// get the graphic object form the rigid body
			vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

			// set the transformation matrix for this rigid body
			dMatrix& matrix = *((dMatrix*)matrixPtr);
			vehicle->SetMatrix (matrix);
		}

		dFloat m_side;
		dFloat m_jointAngle;
		ThighEngine* m_engine; 
		SixLegedRobotCalf *m_calf;
	};


	class SixLegedRobotLeg: public RenderPrimitive
	{
		class LegEngine: public CustomUniversal
		{
			public:
			LegEngine(SixLegedRobotLeg* me, const dMatrix& pinAndPivot, NewtonBody* child, NewtonBody* parent)
				:CustomUniversal (pinAndPivot, child, parent)
			{
				m_lift = 0.0f;
				m_angle = 0.0f;
				m_parent = me;
			}

			virtual void SubmitConstrainst (dFloat timestep, int threadIndex)
			{
				dFloat angle;
				dFloat relAngle;
				dFloat sinAngle;
				dFloat cosAngle;
				dMatrix matrix0;
				dMatrix matrix1;

				// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
				CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

				// get the pin fixed to the first body
				const dVector& dir0 = matrix0.m_front;
				// get the pin fixed to the second body
				const dVector& dir1 = matrix1.m_up;

				// construct an orthogonal coordinate system with these two vectors
				dVector dir2 (dir0 * dir1);
				dVector dir3 (dir2 * dir0);
				dir3 = dir3.Scale (1.0f / dSqrt (dir3 % dir3));

				const dVector& p0 = matrix0.m_posit;
				const dVector& p1 = matrix1.m_posit;

				dVector q0 (p0 + dir3.Scale(MIN_JOINT_PIN_LENGTH));
				dVector q1 (p1 + dir1.Scale(MIN_JOINT_PIN_LENGTH));

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir0[0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir1[0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir2[0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &dir0[0]);
				NewtonUserJointSetRowStiffness (m_joint, 1.0f);

				dFloat relOmega;
				dVector omega0 (0.0f, 0.0f, 0.0f);
				dVector omega1 (0.0f, 0.0f, 0.0f);
				// get relative angular velocity
				NewtonBodyGetOmega(m_body0, &omega0[0]);
				NewtonBodyGetOmega(m_body1, &omega1[0]);
				{
					// calculate the angle error
					sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
					cosAngle = matrix0.m_front % matrix1.m_front;
					angle = dAtan2 (sinAngle, cosAngle);
					relAngle = angle - m_angle;
					// calculate the angle error derivative
					relOmega = (omega0 - omega1) % matrix1.m_up;
					NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);
				}

				{
					sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
					cosAngle = matrix0.m_up % matrix1.m_up;
					angle = dAtan2 (sinAngle, cosAngle);

					// save the angle relative angle into the thigh
					m_parent->m_thight->SetAngle (angle);
					relAngle = angle - m_lift;
					// calculate the angle error derivative
					relOmega = (omega0 - omega1) % matrix0.m_front;
					NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);
				}
			 }

			 void SetTargetAngle (dFloat angle, dFloat lift)
			 {
				m_angle = angle;
				m_lift = lift;
			 }

			 dFloat m_lift;
			 dFloat m_angle;
			 SixLegedRobotLeg* m_parent;
		};


		public:
		SixLegedRobotLeg (dSceneNode* nodeParent, NewtonWorld* nWorld, SixLegedRobot* parent, const dMatrix& localMatrix, dFloat radius, dFloat length, dFloat side)
			:RenderPrimitive (nodeParent, GetIdentityMatrix(), NULL)
		{
//			NewtonBody* body;
			dGeometry* meshInstance;
			NewtonCollision* collision;

			int defaultID;
			defaultID = NewtonMaterialGetDefaultGroupID (nWorld);

			collision = NewtonCreateCapsule (nWorld, radius, length, NULL);

			meshInstance = new dGeometry (collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
			SetGeometry(meshInstance);
			meshInstance->Release();

			// reposition the tire to  pivot around the tire local axis
			dMatrix matrix (dYawMatrix (0.5f * 3.1416f) * localMatrix * parent->GetMatrix());
			SetMatrix (matrix);

//return;
_ASSERTE (0);
/*
			m_side = side;
			// crate the physics for this vehicle

			dFloat mass = 10.0f;
			dVector size (radius * 2, radius * 2, length);
			dFloat Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
			dFloat Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
			dFloat Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

			// ////////////////////////////////////////////////////////////////
			//
			//create the rigid body
			
			body = NewtonCreateBody (nWorld, collision);

			// set the material group id for vehicle
			NewtonBodySetMaterialGroupID (body, defaultID);

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (body, this);

			// set a destructor for this rigid body
	//		NewtonBodySetDestructorCallback (body, DestroyVehicle);

			// set the transform call back function
			NewtonBodySetTransformCallback (body, SetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);

			// set the mass matrix
			NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (body, &GetMatrix()[0][0]);

			// release the collision 
			NewtonReleaseCollision (nWorld, collision);	


			dMatrix pinAndPivot (matrix);
			pinAndPivot.m_front = matrix.m_right;
			pinAndPivot.m_up = matrix.m_up;
			pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
			m_engine = new LegEngine (this, pinAndPivot, body, parent->m_vehicleBody);

			matrix.m_posit += matrix.m_front.Scale (side * length * 0.5f);
			matrix = dRollMatrix(side * 0.5f * 3.1416f) * matrix;
			length *= 2.0f;
			matrix.m_posit += matrix.m_front.Scale (side * length * 0.5f);
 			m_thight = new SixLegedRobotThight (nodeParent, nWorld, body, matrix, radius, length, side); 
*/
		}

		~SixLegedRobotLeg()
		{
		}


		// Set the vehicle matrix and all tire matrices
		static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr, int threadIndex)
		{
			SixLegedRobot* vehicle;

			// get the graphic object form the rigid body
			vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

			// set the transformation matrix for this rigid body
			dMatrix& matrix = *((dMatrix*)matrixPtr);
			vehicle->SetMatrix (matrix);
		}

		void Render() const
		{
			dMatrix identMatrix (GetIdentityMatrix());
			glLoadMatrix (&identMatrix[0][0]);
			RenderPrimitive::Render ();
		}

		dFloat m_side;
		LegEngine* m_engine;
		SixLegedRobotThight *m_thight; 
	};




	public:
	SixLegedRobot(dSceneNode* parent, NewtonWorld* nWorld, dVector position)
		:RenderPrimitive(parent, GetIdentityMatrix(), NULL)
	{
		static dFloat TankBody[][4] =
		{
			{  CHIKEN_LENGTH,  CHIKEN_HEIGHT, CHIKEN_WIDTH, 0.0f},
			{  CHIKEN_LENGTH,  CHIKEN_HEIGHT,-CHIKEN_WIDTH, 0.0f},
			{ -CHIKEN_LENGTH,  CHIKEN_HEIGHT,-CHIKEN_WIDTH, 0.0f},
			{ -CHIKEN_LENGTH,  CHIKEN_HEIGHT, CHIKEN_WIDTH, 0.0f},
						 							
			{  CHIKEN_LENGTH, -CHIKEN_HEIGHT, CHIKEN_WIDTH, 0.0f},
			{  CHIKEN_LENGTH, -CHIKEN_HEIGHT,-CHIKEN_WIDTH, 0.0f},
			{ -CHIKEN_LENGTH, -CHIKEN_HEIGHT,-CHIKEN_WIDTH, 0.0f},
			{ -CHIKEN_LENGTH, -CHIKEN_HEIGHT, CHIKEN_WIDTH, 0.0f},

			{ CHIKEN_LENGTH * 1.2f,  CHIKEN_HEIGHT * 0.8f,  CHIKEN_WIDTH * 0.8f, 0.0f},
			{ CHIKEN_LENGTH * 1.2f,  CHIKEN_HEIGHT * 0.8f, -CHIKEN_WIDTH * 0.8f, 0.0f},
			{-CHIKEN_LENGTH * 1.2f,  CHIKEN_HEIGHT * 0.8f, -CHIKEN_WIDTH * 0.8f, 0.0f},
			{-CHIKEN_LENGTH * 1.2f,  CHIKEN_HEIGHT * 0.8f,  CHIKEN_WIDTH * 0.8f, 0.0f},
			 					   										
			{ CHIKEN_LENGTH * 1.2f, -CHIKEN_HEIGHT * 0.8f,  CHIKEN_WIDTH * 0.8f, 0.0f},
			{ CHIKEN_LENGTH * 1.2f, -CHIKEN_HEIGHT * 0.8f, -CHIKEN_WIDTH * 0.8f, 0.0f},
			{-CHIKEN_LENGTH * 1.2f, -CHIKEN_HEIGHT * 0.8f, -CHIKEN_WIDTH * 0.8f, 0.0f},
			{-CHIKEN_LENGTH * 1.2f, -CHIKEN_HEIGHT * 0.8f,  CHIKEN_WIDTH * 0.8f, 0.0f},
		};

		int i;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		NewtonCollision* vehicleCollision;

		dMatrix location (GetIdentityMatrix());
		location.m_posit = position;
		location.m_posit.m_w = 1.0f;
		location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f;
		SetMatrix (location);


		m_angle = 0.0f;
		m_omega = 4.0f;
		m_amplitud = 2.0f;
		m_turnAmp = 0.0f;
		m_radius = 1.0f / (2.0f * m_amplitud) ;

		int defaultID;
		defaultID = NewtonMaterialGetDefaultGroupID (nWorld);


		// create the vehicle body from the hull
		int count = sizeof (TankBody) / (4 * sizeof (float));

		vehicleCollision = NewtonCreateConvexHull (nWorld, count, &TankBody[0][0], 4 * sizeof (dFloat), 0.0f, NULL);

		dGeometry* meshInstance;
		meshInstance = new dGeometry (vehicleCollision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		SetGeometry(meshInstance);
		meshInstance->Release();


		// calculate the bbox
		dVector minBox (1.0e10f, 1.0e10f, 1.0e10f); 
		dVector maxBox (-1.0e10f, -1.0e10f, -1.0e10f); 
		for (i = 0; i < 16; i ++) {
			 dFloat x = TankBody[i][0];
			 dFloat y = TankBody[i][1];
			 dFloat z = TankBody[i][2];
		
			 minBox.m_x = min (minBox.m_x, x);
			 minBox.m_y = min (minBox.m_y, y);
			 minBox.m_z = min (minBox.m_z, z);
			 maxBox.m_x = max (maxBox.m_x, x);
			 maxBox.m_y = max (maxBox.m_y, y);
			 maxBox.m_z = max (maxBox.m_z, z);
		}


		m_boxSize = maxBox - minBox;
		m_boxOrigin = (maxBox + minBox).Scale (0.5f);

		// get the vehicle size and collision origin
		m_boxSize = maxBox - minBox;
		m_boxOrigin = (maxBox + minBox).Scale (0.5f);


		// crate the physics for this vehicle
		mass = 100.0f;
		dVector size (m_boxSize);
		Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
		Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
		Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

		// ////////////////////////////////////////////////////////////////
		//
		//create the rigid body
		m_vehicleBody = NewtonCreateBody (nWorld, vehicleCollision);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (m_vehicleBody, this);

		// set the material group id for vehicle
		NewtonBodySetMaterialGroupID(m_vehicleBody, defaultID);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (m_vehicleBody, DestroyVehicle);

		// set the transform call back function
		NewtonBodySetTransformCallback (m_vehicleBody, SetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (m_vehicleBody, Update);

		// set the mass matrix
		NewtonBodySetMassMatrix (m_vehicleBody, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (m_vehicleBody, &GetMatrix()[0][0]);


		// release the collision 
		NewtonReleaseCollision (nWorld, vehicleCollision);	


		// get initial value for the tank tire placement
		dMatrix localMatrix (GetIdentityMatrix());
		dFloat lenght = 1.7 * CHIKEN_LENGTH;

		float radius = 0.3f;
		float length = radius * 2.5f;


		// procedurally place tires on the right side of the tank
		localMatrix.m_posit.m_z = -CHIKEN_WIDTH * 1.2f;
		localMatrix.m_posit.m_y = -CHIKEN_HEIGHT;

		localMatrix.m_posit.m_x = - lenght / 2;
		m_leftLegs[0] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, 1.0f);

		localMatrix.m_posit.m_x = lenght / 2;
		m_leftLegs[1] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, 1.0f);

		localMatrix.m_posit.m_x = 0;
		m_leftLegs[2] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, 1.0f);


		// procedurally place tires on the left side of the tank
		localMatrix.m_posit.m_z = CHIKEN_WIDTH * 1.2f;

		localMatrix.m_posit.m_x = - lenght / 2;
		m_rightLegs[0] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, -1.0f);

		localMatrix.m_posit.m_x = lenght / 2;
		m_rightLegs[1] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, -1.0f);

		localMatrix.m_posit.m_x = 0.0f;
		m_rightLegs[2] = new SixLegedRobotLeg (parent, nWorld, this, localMatrix, radius, length, -1.0f);
	}

	~SixLegedRobot()
	{
 		for (int i = 0; i < int (sizeof (m_rightLegs) / sizeof (SixLegedRobotLeg*)); i ++) {
			delete m_leftLegs[i];
			delete m_rightLegs[i];
		}
	}


	void CalculateLegPosition(dFloat time)
	{
		dFloat rightAngle;
		dFloat leftAngle;

		dFloat rightLiftAngle;
		dFloat leftLiftAngle;

		NewtonWorld* world;

		world = NewtonBodyGetWorld (m_vehicleBody);
		m_angle = dMod (m_angle + m_omega * time, 2.0f * 3.1416);

		dFloat rightAmp = 1.0f;
		dFloat leftAmp = 1.0f;


		 dFloat liftMinAngle = 45.0f * 3.1516f / 180.0f;

		// calculate desired angles for the left side
		leftAngle = leftAmp * dAsin (m_radius * (m_amplitud + m_turnAmp) * dSin (m_angle));
		leftLiftAngle = dAsin (m_radius * (m_amplitud + m_turnAmp) * dCos (m_angle));
		m_leftLegs[0]->m_engine->SetTargetAngle (leftAngle, leftLiftAngle + liftMinAngle);
		m_leftLegs[1]->m_engine->SetTargetAngle (leftAngle, leftLiftAngle + liftMinAngle);

		leftAngle = leftAmp * dAsin (m_radius * (m_amplitud + m_turnAmp) * dSin (m_angle + 3.1416));
		leftLiftAngle = dAsin (m_radius * (m_amplitud + m_turnAmp) * dCos (m_angle + 3.1416));
		m_leftLegs[2]->m_engine->SetTargetAngle (leftAngle, leftLiftAngle + liftMinAngle);



		// calculate desired angles for the right side
		rightAngle = rightAmp * dAsin (m_radius * (m_amplitud - m_turnAmp) * dSin (m_angle + 3.1416));
		rightLiftAngle = dAsin (m_radius * (m_amplitud - m_turnAmp) * dCos (m_angle + 3.1416));
		m_rightLegs[2]->m_engine->SetTargetAngle (rightAngle, rightLiftAngle - liftMinAngle);

		rightAngle = rightAmp * dAsin (m_radius * (m_amplitud - m_turnAmp) * dSin (m_angle));
		rightLiftAngle = dAsin (m_radius * (m_amplitud - m_turnAmp) * dCos (m_angle));

		m_rightLegs[0]->m_engine->SetTargetAngle (rightAngle, rightLiftAngle - liftMinAngle);
		m_rightLegs[1]->m_engine->SetTargetAngle (rightAngle, rightLiftAngle - liftMinAngle);
	 }



	// rigid body destructor
	static void DestroyVehicle (const NewtonBody* body)
	{
		SixLegedRobot* vehicle;

		// get the graphic object form the rigid body
		vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

		// destroy the graphic object
		delete vehicle;
	}


	// Set the vehicle matrix and all tire matrices
	static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr, int threadIndex)
	{
		SixLegedRobot* vehicle;

		// get the graphic object form the rigid body
		vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

		// set the transformation matrix for this rigid body
		dMatrix& matrix = *((dMatrix*)matrixPtr);
		vehicle->SetMatrix (matrix);
	}

	static void Update (const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		SixLegedRobot* vehicle;

		// get the graphic object form the rigid body
		vehicle = (SixLegedRobot*) NewtonBodyGetUserData (body);

		// update external forces
		PhysicsApplyGravityForce(body, timestep, threadIndex); 

//		vehicle->CalculateLegPosition(timestep);
	}


	dVector m_boxSize;
	dVector m_boxOrigin;
	NewtonBody* m_vehicleBody;

	dFloat m_angle;
	dFloat m_omega;
	dFloat m_amplitud;
	dFloat m_turnAmp;
	dFloat m_radius;

	SixLegedRobotLeg* m_leftLegs[3];
	SixLegedRobotLeg* m_rightLegs[3];
	friend class SixLegedRobotLeg;
};



static void SetDemoCallbacks (DemosSystem& system, dSceneNode* scene)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
	system.m_scene = scene;
}


static NewtonBody* BuildFloorAndSceneRoot (DemosSystem& system)
{
	NewtonWorld* world;
	NewtonBody* floorBody;
	dSceneNode* scene;

	world = system.m_world;

	// create the sky box,
	scene = new SkyBoxPrimitive (NULL);


	HeightFieldPrimitive* level = new HeightFieldPrimitive (scene, world);
	floorBody = level->GetRigidBody();


	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);


	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

//	NewtonMaterialSetSurfaceThickness(world, materialID, materialID, 0.1f);
//	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);


	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);


	// save th3 callback
	SetDemoCallbacks (system, scene);
	return floorBody;
}



void MotorizedRobots (DemosSystem& system)
{
	NewtonWorld* world;
	NewtonBody* floor; 
	world = system.m_world;

	// create the sky box and the floor,
	floor = BuildFloorAndSceneRoot (system);

	cameraEyepoint.m_y = FindFloor (system.m_world, cameraEyepoint.m_x, cameraEyepoint.m_z) + 4.0f;

	for (int x = 0; x < 1; x ++) {
		for (int z = 0; z < 1; z ++) {
			dVector point (cameraEyepoint + dVector (x * 12.0f, 0.0f, z * 8.0f, 1.0f));
			point.m_z -= 40.0f;
			point.m_w = 1.0f;
			new SixLegedRobot (system.m_scene, system.m_world, point);
		}
	}
}



