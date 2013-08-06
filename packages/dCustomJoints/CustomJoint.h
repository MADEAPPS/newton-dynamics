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

	NEWTON_API CustomJoint();
	NEWTON_API CustomJoint(int maxDOF, NewtonBody* const body0, NewtonBody* const body1);
	NEWTON_API virtual ~CustomJoint();

	NEWTON_API void SetBodiesCollisionState (int state);
	NEWTON_API int GetBodiesCollisionState () const;

	NEWTON_API NewtonBody* GetBody0 () const;
	NEWTON_API NewtonBody* GetBody1 () const;
	NEWTON_API NewtonJoint* GetJoint () const;


	// the application needs to implement this function for serialization
	NEWTON_API virtual void GetInfo (NewtonJointRecord* const info) const;




	// these member function are only used by the C interface or for hooking callback to customize a particular 
	// joint without deriving a new one
	// note: this is not a extension of a virtual function, DO NOT CALL the base class SubmitConstraints!! 
	NEWTON_API void SetUserData (void* userData) {m_userData = userData;}
	NEWTON_API void* GetUserData () const {return m_userData;}
	NEWTON_API void SetUserDestructorCallback (JointUserDestructorCallback callback) {m_userDestructor = callback;}
	NEWTON_API void SetUserSubmintConstraintCallback (JointUserSubmitConstraintCallback callback) {m_userConstrationCallback = callback;}

	private:
	// this are the callback needed to have transparent c++ method interfaces 
	static void Destructor (const NewtonJoint* me);	
	static void SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex);
	static void GetInfo (const NewtonJoint* const me, NewtonJointRecord* info);
	

	protected:
	NEWTON_API void Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1);
	NEWTON_API virtual void ProjectError () const;

	// the application needs to implement this function for each derived joint. See examples for more detail
	NEWTON_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	NEWTON_API void CalculateGlobalMatrix (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& matrix0, dMatrix& matrix1) const;
	NEWTON_API void CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;


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

