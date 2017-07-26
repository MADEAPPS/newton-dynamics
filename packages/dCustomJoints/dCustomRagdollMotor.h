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
	class dSaveLoad
	{
		public:
		dSaveLoad(NewtonWorld* const world) : m_world(world) {}
		virtual ~dSaveLoad() {}
		virtual const char* GetBodyUniqueName(const NewtonBody* const body) const = 0;
		virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const = 0;

		virtual NewtonBody* Load(const char* const name);
		virtual void Save(const char* const name, NewtonBody* const rootbody);

		private:
		dSaveLoad() : m_world(NULL) {}

		class BodyJointPair
		{
			public:
			BodyJointPair(NewtonBody* const body, dCustomRagdollMotor* const joint)
				:m_body(body)
				,m_joint(joint)
			{
			}

			NewtonBody* m_body;
			dCustomRagdollMotor* m_joint;
		};
		
		NewtonCollision* ParseCollisonShape(FILE* const file);
		void GetBodyList (dList<BodyJointPair>& list, NewtonBody* const rootbody);
		void ParseRigidBody(FILE* const file, dTree<NewtonBody*, const dString>& bodyMap);
		void ParseJoint(FILE* const file, const dTree<NewtonBody*, const dString>& bodyMap);

		NewtonWorld* m_world;
	};

	CUSTOM_JOINTS_API dCustomRagdollMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor();

	CUSTOM_JOINTS_API dFloat GetJointTorque() const;
	CUSTOM_JOINTS_API void SetJointTorque(dFloat torque);

	CUSTOM_JOINTS_API bool GetMode() const;
	CUSTOM_JOINTS_API void SetMode(bool ragDollOrMotor);
	
	
	protected:
	CUSTOM_JOINTS_API dCustomRagdollMotor(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_torque;
	int m_motorMode;

	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor, dCustomBallAndSocket)
};


class dCustomRagdollMotor_1dof: public dCustomRagdollMotor
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_1dof(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent);

	CUSTOM_JOINTS_API void SetTwistAngle(dFloat minAngle, dFloat maxAngle);
	CUSTOM_JOINTS_API void GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const;

	protected:
	CUSTOM_JOINTS_API dCustomRagdollMotor_1dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	private:
	virtual void SubmitConstraints(dFloat timestep, int threadIndex);

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
	CUSTOM_JOINTS_API dCustomRagdollMotor_2dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	private:
	virtual void SubmitConstraints(dFloat timestep, int threadIndex);

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
	CUSTOM_JOINTS_API dCustomRagdollMotor_3dof(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	private:
	virtual void SubmitConstraints(dFloat timestep, int threadIndex);

	dFloat m_coneAngle;
	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_3dof, dCustomRagdollMotor)
};


class dCustomRagdollMotor_EndEffector: public dCustomJoint
{
	public:
	CUSTOM_JOINTS_API dCustomRagdollMotor_EndEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentPointInGlobalSpace);
	CUSTOM_JOINTS_API virtual ~dCustomRagdollMotor_EndEffector();

	CUSTOM_JOINTS_API void SetMaxLinearFriction(dFloat accel);
	CUSTOM_JOINTS_API void SetMaxAngularFriction(dFloat alpha);

	CUSTOM_JOINTS_API void SetTargetPosit(const dVector& posit);
	CUSTOM_JOINTS_API void SetTargetRotation(const dQuaternion& rotation);
	
	CUSTOM_JOINTS_API dMatrix GetTargetMatrix() const;
	CUSTOM_JOINTS_API void SetTargetMatrix(const dMatrix& matrix);

	protected:
	CUSTOM_JOINTS_API dCustomRagdollMotor_EndEffector(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual void SubmitConstraints(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void Serialize(NewtonSerializeCallback callback, void* const userData) const { dAssert(0); }

	dVector m_localHandle;
	dVector m_targetPosit;
	dQuaternion m_targetRot;
	dFloat m_maxLinearFriction;
	dFloat m_maxAngularFriction;

	DECLARE_CUSTOM_JOINT(dCustomRagdollMotor_EndEffector, dCustomJoint)
};


#endif 

