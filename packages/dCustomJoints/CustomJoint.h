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

#ifndef __D_NEWTON_CUSTOM_JOINT_H__
#define __D_NEWTON_CUSTOM_JOINT_H__

#include "CustomJointLibraryStdAfx.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "CustomAlloc.h"

struct NewtonUserJoint;
typedef void (*JointUserDestructorCallback) (const NewtonUserJoint* const me);	
typedef void (*JointUserSubmitConstraintCallback) (const NewtonUserJoint* const me, dFloat timestep, int threadIndex);


#define D_CUSTOM_LARGE_VALUE		dFloat (1.0e20f)

#define DECLARE_CUSTOM_JOINT(className,baseClass)																			\
	public:																													\
	virtual dCRCTYPE GetSerializeKey() const { return dCRC64(#className); }													\
	virtual const char* GetTypeName() const { return #className; }															\
	class SerializeMetaData: public baseClass::SerializeMetaData															\
	{																														\
		public:																												\
		SerializeMetaData(const char* const name)																			\
			:baseClass::SerializeMetaData(name)																				\
		{																													\
		}																													\
		virtual void SerializeJoint (CustomJoint* const joint, NewtonSerializeCallback callback, void* const userData)		\
		{																													\
			joint->Serialize(callback, userData);																			\
		}																													\
		virtual CustomJoint* DeserializeJoint (NewtonBody* const body0, NewtonBody* const body1,							\
											   NewtonDeserializeCallback callback, void* const userData)					\
		{																													\
			return new className (body0, body1, callback, userData);														\
		}																													\
	};																														\
	friend class SerializeMetaData;																							\
	CUSTOM_JOINTS_API static SerializeMetaData m_metaData_##className;

#define IMPLEMENT_CUSTON_JOINT(className)																					\
	className::SerializeMetaData className::m_metaData_##className(#className);												\

// this is the base class to implement custom joints, it is not a joint it just provide functionality
// for the user to implement it own joints
class CustomJoint: public CustomAlloc  
{
	public:
	class SerializeMetaData
	{
		public:
		CUSTOM_JOINTS_API SerializeMetaData(const char* const name);
		CUSTOM_JOINTS_API virtual void SerializeJoint (CustomJoint* const joint, NewtonSerializeCallback callback, void* const userData);
		CUSTOM_JOINTS_API virtual CustomJoint* DeserializeJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);
	};

	class SerializeMetaDataDictionary: public dTree<SerializeMetaData*, dCRCTYPE>
	{
		public:
		SerializeMetaDataDictionary()
			:dTree<SerializeMetaData*, dCRCTYPE>()
		{
		}
	};

	class AngularIntegration
	{
		public:
		AngularIntegration()
		{
			SetAngle (0.0f);
		}

		AngularIntegration(dFloat angle)
		{
			SetAngle (angle);
		}

		dFloat GetAngle () const
		{
			return m_angle;
		}

		void SetAngle (dFloat angle)
		{
			m_angle = angle;
			m_sinJointAngle = dSin(angle);
			m_cosJointAngle = dCos(angle);
		}

		dFloat Update (dFloat newAngleCos, dFloat newAngleSin)
		{
			dFloat sin_da = newAngleSin * m_cosJointAngle - newAngleCos * m_sinJointAngle; 
			dFloat cos_da = newAngleCos * m_cosJointAngle + newAngleSin * m_sinJointAngle; 

			m_angle += dAtan2 (sin_da, cos_da);
			m_cosJointAngle = newAngleCos;
			m_sinJointAngle = newAngleSin;
			return m_angle;
		}

		AngularIntegration operator+ (const AngularIntegration& angle) const
		{
			dFloat sin_da = angle.m_sinJointAngle * m_cosJointAngle + angle.m_cosJointAngle * m_sinJointAngle; 
			dFloat cos_da = angle.m_cosJointAngle * m_cosJointAngle - angle.m_sinJointAngle * m_sinJointAngle; 
			dFloat angle_da = dAtan2 (sin_da, cos_da);
			return AngularIntegration(m_angle + angle_da);
		}

		AngularIntegration operator- (const AngularIntegration& angle) const
		{
			dFloat sin_da = angle.m_sinJointAngle * m_cosJointAngle - angle.m_cosJointAngle * m_sinJointAngle; 
			dFloat cos_da = angle.m_cosJointAngle * m_cosJointAngle + angle.m_sinJointAngle * m_sinJointAngle; 
			dFloat angle_da = dAtan2 (sin_da, cos_da);
			return AngularIntegration (angle_da);
		}


		dFloat Update (dFloat angle)
		{
			return Update (dCos (angle), dSin (angle));
		}

		private:
		dFloat m_angle;
		dFloat m_sinJointAngle;
		dFloat m_cosJointAngle;
	};


	CUSTOM_JOINTS_API CustomJoint();
	CUSTOM_JOINTS_API CustomJoint(int maxDOF, NewtonBody* const body0, NewtonBody* const body1);
	CUSTOM_JOINTS_API CustomJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual ~CustomJoint();

	CUSTOM_JOINTS_API static void Initalize(NewtonWorld* const world);

	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const;
	
	CUSTOM_JOINTS_API void SetBodiesCollisionState (int state);
	CUSTOM_JOINTS_API int GetBodiesCollisionState () const;

	CUSTOM_JOINTS_API NewtonBody* GetBody0 () const;
	CUSTOM_JOINTS_API NewtonBody* GetBody1 () const;
	CUSTOM_JOINTS_API NewtonJoint* GetJoint () const;
	CUSTOM_JOINTS_API const dMatrix& GetMatrix0 () const;
	CUSTOM_JOINTS_API const dMatrix& GetMatrix1 () const;

	// the application needs to implement this function for serialization
	CUSTOM_JOINTS_API virtual const char* GetTypeName() const;
	CUSTOM_JOINTS_API virtual dCRCTYPE GetSerializeKey() const;
	CUSTOM_JOINTS_API virtual void GetInfo (NewtonJointRecord* const info) const;

	// these member function are only used by the C interface or for hooking callback to customize a particular 
	// joint without deriving a new one
	// note: this is not a extension of a virtual function, DO NOT CALL the base class SubmitConstraints!! 
	CUSTOM_JOINTS_API void SetUserData (void* userData) {m_userData = userData;}
	CUSTOM_JOINTS_API void* GetUserData () const {return m_userData;}

	CUSTOM_JOINTS_API dFloat GetStiffness () const;
	CUSTOM_JOINTS_API void SetStiffness (dFloat stiffness);

	CUSTOM_JOINTS_API void SetUserDestructorCallback (JointUserDestructorCallback callback) {m_userDestructor = callback;}
	CUSTOM_JOINTS_API void SetUserSubmintConstraintCallback (JointUserSubmitConstraintCallback callback) {m_userConstrationCallback = callback;}

	CUSTOM_JOINTS_API virtual void UserUpdate(dFloat timestep, int threadIndex) { dAssert (0);}

	private:
	// this are the callback needed to have transparent c++ method interfaces 
	CUSTOM_JOINTS_API static void Destructor (const NewtonJoint* me);	
	CUSTOM_JOINTS_API static void SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API static void GetInfo (const NewtonJoint* const me, NewtonJointRecord* const info);
	CUSTOM_JOINTS_API static void Serialize (const NewtonJoint* const me, NewtonSerializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API static void Deserialize (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);

	protected:
	CUSTOM_JOINTS_API void Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1);

	// the application needs to implement this function for each derived joint. See examples for more detail
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	CUSTOM_JOINTS_API void CalculateGlobalMatrix (dMatrix& matrix0, dMatrix& matrix1) const;
	CUSTOM_JOINTS_API void CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;

	CUSTOM_JOINTS_API static SerializeMetaDataDictionary& GetDictionary();

	CUSTOM_JOINTS_API dFloat CalculateAngle (const dVector& dir, const dVector& cosDir, const dVector& sinDir) const;
	CUSTOM_JOINTS_API dFloat CalculateAngle (const dVector& dir, const dVector& cosDir, const dVector& sinDir, dFloat& sinAngle, dFloat& cosAngle) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	void* m_userData;
	NewtonBody* m_body0;
	NewtonBody* m_body1;
	NewtonJoint* m_joint;
	NewtonWorld* m_world;
	JointUserDestructorCallback m_userDestructor;
	JointUserSubmitConstraintCallback m_userConstrationCallback;
	dFloat m_stiffness;
	int m_maxDof;
	int m_autoDestroy;
	
	static SerializeMetaData m_metaData;
};



#endif 

