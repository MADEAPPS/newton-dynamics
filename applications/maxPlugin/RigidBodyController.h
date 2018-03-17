/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/


#ifndef __RIGIDBODY_CONTROLLER_H__
#define __RIGIDBODY_CONTROLLER_H__



#define RIGIDBODY_CONTROLLER_ID Class_ID(0x6e6c6a1b, 0x6c7d3fb9)


class RigidBodyControllerDesc: public ClassDesc2 
{
	public:
	int  IsPublic();
	void* Create(BOOL loading);
	const TCHAR* ClassName();
	SClass_ID SuperClassID();
	Class_ID ClassID();
	const TCHAR* Category();
	static ClassDesc* GetDescriptor();
};

class RigidBodyController: public Control, public RigidBodyData
{
	public:
	//Constructor/Destructor
	RigidBodyController(BOOL loading=FALSE);
	~RigidBodyController();

	void Init (const RigidBodyData& data, INode* const myNode);
	void DeleteThis();

	Matrix3 ApplyInheritance(TimeValue t,const Matrix3 &ptm,Control *pos,Point3 cpos=Point3(0,0,0),BOOL usecpos=FALSE);

	void Rotate(TimeValue t, const AngAxis& aa, int commit);
	void Move (TimeValue t, Matrix3& partm, Matrix3& tmAxis, Point3& v, BOOL localOrigin, int commit);
	void SetAbsValue(TimeValue t, const Matrix3 &val, const Matrix3 &parent, int commit);


	//From Animatable
	Class_ID ClassID();
	SClass_ID SuperClassID();
	void GetClassName(TSTR& s);

	void PostCloneNode();
	RefTargetHandle Clone( RemapDir &remap );
	RefResult NotifyRefChanged(Interval changeInt, RefTargetHandle hTarget, 
	PartID& partID,  RefMessage message);

	int NumSubs();
	TSTR SubAnimName(int i);				
	Animatable* SubAnim(int i);

	// TODO: Maintain the number or references here
	int NumRefs();
	RefTargetHandle GetReference(int i);
	void SetReference(int i, RefTargetHandle rtarg);

	int	NumParamBlocks();
	IParamBlock2* GetParamBlock(int i);
	IParamBlock2* GetParamBlockByID(BlockID id);

	void Copy(Control *from);
	void GetValue(TimeValue t, void *val, Interval &valid, GetSetMethod method);
	void SetValue(TimeValue t, void *val, int commit, GetSetMethod method);
	void Update(TimeValue t);

	static void ApplyGravityForce (const NewtonBody* const body, dFloat timestep, int threadIndex);
	static void RenderGizmo (void* const userData, int vertexCount, const dFloat* const faceArray, int faceId);
	virtual int Display(TimeValue t, INode* inode, ViewExp *vpt, int flags);
	virtual IOResult Load(ILoad* iload);
	virtual IOResult Save(ISave* isave);

	virtual void MouseCycleStarted (TimeValue  t);

	void SaveUndoState(TimeValue t);
	void UpdateRigidBodyMatrix(TimeValue  t);

	Control* m_scaleControl;
	Control* m_positionControl;
	Control* m_rotationControl;
	IParamBlock2 *pblock;	//// Parameter block,  ref 0

	Interval ivalid;
	BOOL blockUpdate;
	bool m_updateMatrixFormRedo;
};


inline int RigidBodyControllerDesc::IsPublic() 
{
	return 1;
}

inline void* RigidBodyControllerDesc::Create(BOOL loading) 
{
	return new RigidBodyController();
}

inline const TCHAR* RigidBodyControllerDesc::ClassName() 
{
	return _T("RigidBody Controller");
}

inline SClass_ID RigidBodyControllerDesc::SuperClassID() 
{
	return CTRL_MATRIX3_CLASS_ID;
}

inline Class_ID RigidBodyControllerDesc::ClassID() 
{
	return RIGIDBODY_CONTROLLER_ID;
}

inline const TCHAR* RigidBodyControllerDesc::Category() 
{
	return _T("");
}

inline ClassDesc* RigidBodyControllerDesc::GetDescriptor() 
{
	static RigidBodyControllerDesc controllerDesc;
	return &controllerDesc;
}



#endif