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


// CustomJoint.h: interface for the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_NEWTON_CUSTOM_JOINT_H_
#define D_NEWTON_CUSTOM_JOINT_H_

#include "CustomJointLibraryStdAfx.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "CustomAlloc.h"

struct NewtonUserJoint;
typedef void (*JointUserDestructorCallback) (const NewtonUserJoint* const me);	
typedef void (*JointUserSubmitConstraintCallback) (const NewtonUserJoint* const me, dFloat timestep, int threadIndex);


// this is the base class to implement custom joints, it is not a joint it just provide functionality
// for the user to implement it own joints
class CustomJoint: public CustomAlloc  
{
	public:
	struct AngularIntegration
	{
		AngularIntegration()
		{
			m_angle = 0.0f;
		}

		dFloat CalculateJointAngle (dFloat newAngleCos, dFloat newAngleSin)
		{
			// the joint angle can be determine by getting the angle between any two non parallel vectors
			dFloat sinJointAngle = dSin (m_angle);
			dFloat cosJointAngle = dCos (m_angle);

			dFloat sin_da = newAngleSin * cosJointAngle - newAngleCos * sinJointAngle; 
			dFloat cos_da = newAngleCos * cosJointAngle + newAngleSin * sinJointAngle; 

			m_angle += dAtan2 (sin_da, cos_da);
			return  m_angle;
		}
		dFloat m_angle;
	};

	CUSTOM_JOINTS_API CustomJoint();
	CUSTOM_JOINTS_API CustomJoint(int maxDOF, NewtonBody* const body0, NewtonBody* const body1);
	CUSTOM_JOINTS_API virtual ~CustomJoint();

	CUSTOM_JOINTS_API void SetBodiesCollisionState (int state);
	CUSTOM_JOINTS_API int GetBodiesCollisionState () const;

	CUSTOM_JOINTS_API NewtonBody* GetBody0 () const;
	CUSTOM_JOINTS_API NewtonBody* GetBody1 () const;
	CUSTOM_JOINTS_API NewtonJoint* GetJoint () const;

	CUSTOM_JOINTS_API void JointSetSolverMode (bool mode, int maxContactJoints) const;


	// the application needs to implement this function for serialization
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;
	CUSTOM_JOINTS_API virtual void ProjectError () const;

	// these member function are only used by the C interface or for hooking callback to customize a particular 
	// joint without deriving a new one
	// note: this is not a extension of a virtual function, DO NOT CALL the base class SubmitConstraints!! 
	CUSTOM_JOINTS_API void SetUserData (void* userData) {m_userData = userData;}
	CUSTOM_JOINTS_API void* GetUserData () const {return m_userData;}
	CUSTOM_JOINTS_API void SetUserDestructorCallback (JointUserDestructorCallback callback) {m_userDestructor = callback;}
	CUSTOM_JOINTS_API void SetUserSubmintConstraintCallback (JointUserSubmitConstraintCallback callback) {m_userConstrationCallback = callback;}

	private:
	// this are the callback needed to have transparent c++ method interfaces 
	static void Destructor (const NewtonJoint* me);	
	static void SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex);
	static void GetInfo (const NewtonJoint* const me, NewtonJointRecord* info);
	

	protected:
	bool TestIdentityMatrix(const dMatrix& matrix) const 
	{
		const dMatrix& identity = GetIdentityMatrix();

		bool isIdentity = true;
		for (int i = 0; isIdentity && (i < 3); i ++) {
			isIdentity &= dAbs (matrix[3][i]) < 1.0e-4f;
			for (int j = i; isIdentity && (j < 3); j ++) {
				isIdentity &= dAbs (matrix[i][j]-identity[i][j]) < 1.0e-4f;
			}
		}
		return isIdentity;
	}

	CUSTOM_JOINTS_API void Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1);

	// the application needs to implement this function for each derived joint. See examples for more detail
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API void CalculateGlobalMatrix (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& matrix0, dMatrix& matrix1) const;
	CUSTOM_JOINTS_API void CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;


	void* m_userData;
	NewtonBody* m_body0;
	NewtonBody* m_body1;
	NewtonJoint* m_joint;
	NewtonWorld* m_world;
	JointUserDestructorCallback m_userDestructor;
	JointUserSubmitConstraintCallback m_userConstrationCallback;
	int m_maxDof;
	int m_autoDestroy;
};



#endif 

