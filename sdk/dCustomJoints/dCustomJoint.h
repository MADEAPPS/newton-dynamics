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


// dCustomJoint.h: interface for the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __D_NEWTON_CUSTOM_JOINT_H__
#define __D_NEWTON_CUSTOM_JOINT_H__

#include "dCustomJointLibraryStdAfx.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "dCustomAlloc.h"

class dCustomJoint;
class dCustomJointSaveLoad;
typedef void (*dJointUserDestructorCallback) (const dCustomJoint* const me);	


#define D_CUSTOM_LARGE_VALUE		dFloat (1.0e20f)

#define DECLARE_CUSTOM_JOINT(className,baseClass)																			\
	public:																													\
	virtual bool IsType (dCRCTYPE type) const																				\
	{																														\
		if (type == m_metaData_##className.m_key_##className) {																\
			return true;																									\
		}																													\
		return baseClass::IsType(type);																						\
	}																														\
	className(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)	\
		:baseClass(child, parent, callback, userData)																		\
	{																														\
		dAssert (0);																								\
	}																														\
	className(dCustomJointSaveLoad* const fileLoader, NewtonBody* const body0, NewtonBody* const body1)						\
		:baseClass(fileLoader, body0, body1)																				\
	{																														\
		Load(fileLoader);																									\
	}																														\
	virtual dCRCTYPE GetSerializeKey() const { return m_metaData_##className.m_key_##className;}							\
	static dCRCTYPE GetType () { return m_metaData_##className.m_key_##className; }										    \
	virtual const char* GetTypeName() const { return #className; }															\
	class SerializeMetaData_##className: public baseClass::dSerializeMetaData												\
	{																														\
		public:																												\
		SerializeMetaData_##className(const char* const name)																\
			:baseClass::dSerializeMetaData(name)																			\
			,m_key_##className (dCRC64(#className))																			\
		{																													\
		}																													\
		virtual void SerializeJoint (dCustomJoint* const joint, NewtonSerializeCallback callback, void* const userData)		\
		{																													\
			joint->Serialize(callback, userData);																			\
		}																													\
		virtual dCustomJoint* DeserializeJoint (NewtonBody* const body0, NewtonBody* const body1,							\
											    NewtonDeserializeCallback callback, void* const userData)					\
		{																													\
			return new className (body0, body1, callback, userData);														\
		}																													\
		virtual dCustomJoint* Load(dCustomJointSaveLoad* const fileLoader, NewtonBody* const body0, NewtonBody* const body1)\
		{																													\
			return new className (fileLoader, body0, body1);																\
		}																													\
		dCRCTYPE m_key_##className;																							\
	};																														\
	friend class SerializeMetaData_##className;																				\
	static SerializeMetaData_##className m_metaData_##className;

#define IMPLEMENT_CUSTOM_JOINT(className)																					\
	className::SerializeMetaData_##className className::m_metaData_##className(#className);									\

// this is the base class to implement custom joints, it is not a joint it just provide functionality
// for the user to implement it own joints
class dCustomJoint: public dCustomAlloc  
{
	public:
	class dDebugDisplay
	{
		public:
		dDebugDisplay (const dMatrix& cameraMatrix) 
			:m_cameraMatrix(cameraMatrix)
			,m_width(0)
			,m_height(0)
		{
		}
		virtual ~dDebugDisplay () {}

		virtual void SetColor(const dVector& color) = 0;
		virtual void DrawLine(const dVector& p0, const dVector& p1) = 0;
		virtual void SetOrthRendering () {};
		virtual void ResetOrthRendering () {};

		const dMatrix& GetCameraMatrix() const {return m_cameraMatrix;}
		CUSTOM_JOINTS_API void DrawFrame(const dMatrix& matrix);

		dMatrix m_cameraMatrix;
		int m_width;
		int m_height;
	};


	class dSerializeMetaData
	{
		public:
		CUSTOM_JOINTS_API dSerializeMetaData(const char* const name);
		CUSTOM_JOINTS_API virtual void SerializeJoint (dCustomJoint* const joint, NewtonSerializeCallback callback, void* const userData);
		CUSTOM_JOINTS_API virtual dCustomJoint* DeserializeJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);
		CUSTOM_JOINTS_API virtual bool IsType (dCRCTYPE type) const {return false;}
		CUSTOM_JOINTS_API virtual dCustomJoint* Load (dCustomJointSaveLoad* const fileLoader, NewtonBody* const body0, NewtonBody* const body1);
	};

	class dSerializeMetaDataDictionary: public dTree<dSerializeMetaData*, dCRCTYPE>
	{
		public:
		dSerializeMetaDataDictionary()
			:dTree<dSerializeMetaData*, dCRCTYPE>()
		{
		}
	};

	class dAngularIntegration
	{
		public:
		dAngularIntegration()
		{
			SetAngle (0.0f);
		}

		dAngularIntegration(dFloat angle)
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

		dAngularIntegration operator+ (const dAngularIntegration& angle) const
		{
			dFloat sin_da = angle.m_sinJointAngle * m_cosJointAngle + angle.m_cosJointAngle * m_sinJointAngle; 
			dFloat cos_da = angle.m_cosJointAngle * m_cosJointAngle - angle.m_sinJointAngle * m_sinJointAngle; 
			dFloat angle_da = dAtan2 (sin_da, cos_da);
			return dAngularIntegration(m_angle + angle_da);
		}

		dAngularIntegration operator- (const dAngularIntegration& angle) const
		{
			dFloat sin_da = angle.m_sinJointAngle * m_cosJointAngle - angle.m_cosJointAngle * m_sinJointAngle; 
			dFloat cos_da = angle.m_cosJointAngle * m_cosJointAngle + angle.m_sinJointAngle * m_sinJointAngle; 
			dFloat angle_da = dAtan2 (sin_da, cos_da);
			return dAngularIntegration (angle_da);
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


	CUSTOM_JOINTS_API dCustomJoint();
	CUSTOM_JOINTS_API dCustomJoint(int maxDOF, NewtonBody* const body0, NewtonBody* const body1);
	CUSTOM_JOINTS_API dCustomJoint(dCustomJointSaveLoad* const fileLoader, NewtonBody* const body0, NewtonBody* const body1);
	CUSTOM_JOINTS_API dCustomJoint (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API virtual ~dCustomJoint();

	CUSTOM_JOINTS_API static void Initalize(NewtonWorld* const world);

	CUSTOM_JOINTS_API virtual void Serialize (NewtonSerializeCallback callback, void* const userData) const;
	CUSTOM_JOINTS_API virtual void Deserialize (NewtonSerializeCallback callback, void* const userData) const;
	
	CUSTOM_JOINTS_API void SetBodiesCollisionState (int state);
	CUSTOM_JOINTS_API int GetBodiesCollisionState () const;

	CUSTOM_JOINTS_API NewtonBody* GetBody0 () const;
	CUSTOM_JOINTS_API NewtonBody* GetBody1 () const;
	CUSTOM_JOINTS_API NewtonJoint* GetJoint () const;
	CUSTOM_JOINTS_API const dMatrix& GetMatrix0 () const;
	CUSTOM_JOINTS_API const dMatrix& GetMatrix1 () const;
	CUSTOM_JOINTS_API void CalculateGlobalMatrix (dMatrix& matrix0, dMatrix& matrix1) const;

	// the application needs to implement this function for serialization
	CUSTOM_JOINTS_API virtual bool IsType (dCRCTYPE type) const;
	CUSTOM_JOINTS_API virtual const char* GetTypeName() const;
	CUSTOM_JOINTS_API virtual dCRCTYPE GetSerializeKey() const;
	
	// these member function are only used by the C interface or for hooking callback to customize a particular 
	// joint without deriving a new one
	// note: this is not a extension of a virtual function, DO NOT CALL the base class SubmitConstraints!! 
	CUSTOM_JOINTS_API void SetUserData (void* userData) {m_userData = userData;}
	CUSTOM_JOINTS_API void* GetUserData () const {return m_userData;}

	CUSTOM_JOINTS_API dFloat GetStiffness () const;
	CUSTOM_JOINTS_API void SetStiffness (dFloat stiffness);

	CUSTOM_JOINTS_API void SetSolverModel(int model);
	CUSTOM_JOINTS_API int GetSolverModel() const;

	CUSTOM_JOINTS_API void SetUserDestructorCallback(dJointUserDestructorCallback callback) { m_userDestructor = callback; }
	CUSTOM_JOINTS_API virtual void Debug(dDebugDisplay* const debugDisplay) const;

	CUSTOM_JOINTS_API virtual void Load(dCustomJointSaveLoad* const fileLoader);
	CUSTOM_JOINTS_API virtual void Save(dCustomJointSaveLoad* const fileSaver) const;
	


	private:
	// this are the callback needed to have transparent c++ method interfaces 
	CUSTOM_JOINTS_API static void Destructor (const NewtonJoint* me);	
	CUSTOM_JOINTS_API static void SubmitConstraints (const NewtonJoint* const me, dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API static void Serialize (const NewtonJoint* const me, NewtonSerializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API static void Deserialize (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback callback, void* const userData);
	CUSTOM_JOINTS_API static dCustomJoint* Load(dCustomJointSaveLoad* const fileLoader, const char* const jointType, NewtonBody* const body0, NewtonBody* const body1);

	protected:
	CUSTOM_JOINTS_API dCustomJoint (NewtonInverseDynamics* const invDynSolver, void* const invDynNode);
	CUSTOM_JOINTS_API void Init (int maxDOF, NewtonBody* const body0, NewtonBody* const body1);

	// the application needs to implement this function for each derived joint. See examples for more detail
	CUSTOM_JOINTS_API virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	
	CUSTOM_JOINTS_API void CalculateLocalMatrix (const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;

	CUSTOM_JOINTS_API static dSerializeMetaDataDictionary& GetDictionary();

	CUSTOM_JOINTS_API dFloat CalculateAngle (const dVector& planeDir, const dVector& cosDir, const dVector& sinDir) const;
	CUSTOM_JOINTS_API dFloat CalculateAngle (const dVector& planeDir, const dVector& cosDir, const dVector& sinDir, dFloat& sinAngle, dFloat& cosAngle) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	void* m_userData;
	NewtonBody* m_body0;
	NewtonBody* m_body1;
	NewtonJoint* m_joint;
	NewtonWorld* m_world;
	dJointUserDestructorCallback m_userDestructor;
	dFloat m_stiffness;
	int m_maxDof;
	int m_autoDestroy;
	CUSTOM_JOINTS_API static dCRCTYPE m_key;
	CUSTOM_JOINTS_API static dSerializeMetaData m_metaData_CustomJoint;

	friend class dCustomJointSaveLoad;
};



#endif 

