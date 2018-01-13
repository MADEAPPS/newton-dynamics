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



// dCustom6DOF.h: interface for the dCustom6DOF class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _CUSTOM_6DOF_H_
#define _CUSTOM_6DOF_H_

#include "dCustomJoint.h"

class dCustom6DOF: public dCustomJoint  
{
	class dAngleData
	{
		public:
		dAngleData()
			:m_currentAngle()
			,m_minAngle(-0.0f)
			,m_maxAngle(0.0f)
		{
		}

		dAngularIntegration m_currentAngle;
		dFloat m_minAngle;
		dFloat m_maxAngle;
	};

	public:
	CUSTOM_JOINTS_API dCustom6DOF (const dMatrix& pinsAndPivotChildFrame, const dMatrix& pinsAndPivotParentFrame, NewtonBody* const child, NewtonBody* const parent = NULL);
	CUSTOM_JOINTS_API virtual ~dCustom6DOF();

	CUSTOM_JOINTS_API void GetLinearLimits (dVector& minLinearLimits, dVector& maxLinearLimits) const;
	CUSTOM_JOINTS_API void SetLinearLimits (const dVector& minLinearLimits, const dVector& maxLinearLimits);

	CUSTOM_JOINTS_API void SetYawLimits(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void SetRollLimits(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void SetPitchLimits(dFloat minAngle, dFloat maxAngle);

	CUSTOM_JOINTS_API void GetYawLimits(dFloat& minAngle, dFloat& maxAngle) const;
	CUSTOM_JOINTS_API void GetRollLimits(dFloat& minAngle, dFloat& maxAngle) const;
	CUSTOM_JOINTS_API void GetPitchLimits(dFloat& minAngle, dFloat& maxAngle) const;

	CUSTOM_JOINTS_API dFloat GetYaw() const { return m_yaw.m_currentAngle.GetAngle();}
	CUSTOM_JOINTS_API dFloat GetRoll() const { return m_roll.m_currentAngle.GetAngle();}
	CUSTOM_JOINTS_API dFloat GetPitch() const { return m_pitch.m_currentAngle.GetAngle();}

	protected:
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const;

	private:
	void CalculateJointAngles(const dMatrix& matrix0, const dMatrix& matrix1);
	
	protected:
	dVector m_minLinearLimits;
	dVector m_maxLinearLimits;
	dAngleData m_yaw;
	dAngleData m_roll;
	dAngleData m_pitch;
	union
	{
		int m_mask;
		struct 
		{
			int m_xAxis : 1;
			int m_yAxis : 1;
			int m_zAxis : 1;
			int m_yawAxis : 1;
			int m_rollAxis : 1;
			int m_pitchAxis : 1;
		};
	};


	DECLARE_CUSTOM_JOINT(dCustom6DOF, dCustomJoint)
};

#endif // !defined(AFX_Custom6DOF_H__B631F556_B7D7_F85ECF3E9ADE_H)

