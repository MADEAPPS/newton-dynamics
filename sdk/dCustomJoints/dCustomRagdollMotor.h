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


// dCustomBallAndSocket.h: interface for the dCustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#ifndef _D_CUSTOM_RAG_DOLL_MOTOR_H_
#define _D_CUSTOM_RAG_DOLL_MOTOR_H_

#include "dCustomJoint.h"
#include "dCustomBallAndSocket.h"


// this joint is for controlling rag dolls muscles
class dCustomRagdollMotor: public dCustomBallAndSocket
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor();

	CUSTOM_JOINTS_API dFloat GetJointTorque() const;
	CUSTOM_JOINTS_API void SetJointTorque(dFloat torque);
	
	protected:
	//CUSTOM_JOINTS_API dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_torque;

	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor, dCustomBallAndSocket)
};


class dCustomRagdollMotor_1dof: public dCustomRagdollMotor
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);

	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	//CUSTOM_JOINTS_API dCustomRagdollMotor_1dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	protected:
	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_1dof, dCustomRagdollMotor)
};


class dCustomRagdollMotor_2dof: public dCustomRagdollMotor
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_2dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);

	CUSTOM_JOINTS_API void SetConeAngle(dFloat angle);
	CUSTOM_JOINTS_API dFloat GetConeAngle() const;

	protected:
	//CUSTOM_JOINTS_API dCustomRagdollMotor_2dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	private:
	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_coneAngle;
	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_2dof, dCustomRagdollMotor)
};


class dCustomRagdollMotor_3dof: public dCustomRagdollMotor
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_3dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);

	CUSTOM_JOINTS_API void SetConeAngle(dFloat angle);
	CUSTOM_JOINTS_API dFloat GetConeAngle() const;

	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	//CUSTOM_JOINTS_API dCustomRagdollMotor_3dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	private:
	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_3dof, dCustomRagdollMotor)
};


class dCustomRagdollMotor_EndEffector: public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_EndEffector(NewtonBody* const body, const dMatrix& attachmentPointInGlobalSpace);
	CUSTOM_JOINTS_API dCustomRagdollMotor_EndEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentPointInGlobalSpace);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor_EndEffector();

	CUSTOM_JOINTS_API void SetAsSixdof (); 
	CUSTOM_JOINTS_API void SetAsThreedof (); 

	CUSTOM_JOINTS_API void SetLinearSpeed(dFloat speed);
	CUSTOM_JOINTS_API void SetAngularSpeed(dFloat speed);

	CUSTOM_JOINTS_API void SetMaxLinearFriction(dFloat friction);
	CUSTOM_JOINTS_API void SetMaxAngularFriction(dFloat friction);

	CUSTOM_JOINTS_API void SetTargetPosit(const dVector& posit);
	CUSTOM_JOINTS_API void SetTargetRotation(const dQuaternion& rotation);
	
	CUSTOM_JOINTS_API dMatrix GetBodyMatrix() const;
	CUSTOM_JOINTS_API dMatrix GetTargetMatrix() const;
	CUSTOM_JOINTS_API void SetTargetMatrix(const dMatrix& matrix);

	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	protected:
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;

	CUSTOM_JOINTS_API virtual void Deserialize (NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;

	dMatrix m_targetMatrix;	
	dFloat m_linearSpeed;
	dFloat m_angularSpeed;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;
	bool m_isSixdof;

	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_EndEffector, dCustomJoint)
};


class dEffectorTreeInterface
{
	public:
	class dEffectorTransform
	{
		public:
		dVector m_posit;
		dQuaternion m_rotation;
		dCustomRagdollMotor_EndEffector* m_effector;
	};

	class dEffectorPose: public dList<dEffectorTransform>
	{
		public:
		dEffectorPose(): dList<dEffectorTransform>(), m_childNode(NULL)	{}
		dEffectorTreeInterface* m_childNode;
	};

	dEffectorTreeInterface(NewtonBody* const rootBody)
		:m_rootBody(rootBody)
	{
	}

	virtual ~dEffectorTreeInterface()
	{
	}

	virtual void Evaluate(dEffectorPose& output)
	{
	}

	NewtonBody* m_rootBody;
};


class dEffectorTreeRoot: public dEffectorTreeInterface
{
	public:
	dEffectorTreeRoot(NewtonBody* const rootBody, dEffectorTreeInterface* const childNode)
		:dEffectorTreeInterface(rootBody)
	{
		m_pose.m_childNode = childNode;
	}
	CUSTOM_JOINTS_API virtual ~dEffectorTreeRoot();

	dEffectorPose& GetPose() {return m_pose;}

	CUSTOM_JOINTS_API void Update();
	CUSTOM_JOINTS_API void Evaluate(dEffectorPose& output);

	protected:
	dEffectorPose m_pose;
};

class dEffectorTreePose: public dEffectorTreeInterface
{
	public:
	dEffectorTreePose(NewtonBody* const rootBody)
		:dEffectorTreeInterface(rootBody)
	{
	}

	virtual void Evaluate(dEffectorPose& output)
	{
		dAssert(0);
	}
};


class dEffectorTreeFixPose: public dEffectorTreePose
{
	public:
	dEffectorTreeFixPose(NewtonBody* const rootBody)
		:dEffectorTreePose(rootBody)
	{
	}

	dEffectorPose& GetPose() {return m_pose;}

	protected:
	CUSTOM_JOINTS_API void Evaluate(dEffectorPose& output);
	dEffectorPose m_pose;
};


#endif 

