/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __DEMO_ENTITY_NOTIFY_H__
#define __DEMO_ENTITY_NOTIFY_H__

#include "ndDemoEntityManager.h"
#include "ndPhysicsUtils.h"

class ndDemoEntity;
class ndShaderCache;
class ndAnimKeyframe;
class ndDemoMeshInterface;

//class ndDemoEntityNotify: public ndBodyNotify
class ndDemoEntityNotify : public ndModelNotify
{
	public:
	//D_CLASS_REFLECTION(ndDemoEntityNotify, ndBodyNotify)
	//class ndDemoEntityNotifyFileLoadSave: public ndFileFormatNotify
	//{
	//	public:
	//	ndDemoEntityNotifyFileLoadSave(const char* const className = ndDemoEntityNotify::StaticClassName())
	//		:ndFileFormatNotify(className)
	//	{
	//	}
	//
	//	void SaveNotify(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
	//	{
	//		nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_NOTIFY_CLASS, ndDemoEntityNotify::StaticClassName());
	//		ndFileFormatNotify::SaveNotify(scene, classNode, notify);
	//		xmlSaveParam(classNode, "useCollisionForVisual", 1);
	//	}
	//};

	ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody = nullptr, ndFloat32 gravity = DEMO_GRAVITY);
	virtual ~ndDemoEntityNotify();

	void* GetUserData() const
	{
		return m_entity;
	}

	//virtual void OnObjectPick() const;
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);

	void RemoveBody();
	bool CheckInWorld(const ndMatrix& matrix) const
	{
		return matrix.m_posit.m_y > -100.0f;
	}

	ndDemoEntity* m_entity;
	ndDemoEntityManager* m_manager;
};

class ndBindingRagdollEntityNotify : public ndDemoEntityNotify
{
	public:
	//D_CLASS_REFLECTION(ndBindingRagdollEntityNotify, ndDemoEntityNotify)
	//class ndBindingRagdollEntityNotifyFileSaveLoad : public ndDemoEntityNotifyFileLoadSave
	//{
	//	public:
	//	ndBindingRagdollEntityNotifyFileSaveLoad()
	//		:ndDemoEntityNotifyFileLoadSave(ndBindingRagdollEntityNotify::StaticClassName())
	//	{
	//	}
	//
	//	void SaveNotify(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
	//	{
	//		nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndBindingNotifyClass", ndBindingRagdollEntityNotify::StaticClassName());
	//		ndDemoEntityNotifyFileLoadSave::SaveNotify(scene, classNode, notify);
	//
	//		ndBindingRagdollEntityNotify* const bindNotiFy = (ndBindingRagdollEntityNotify*)notify;
	//		xmlSaveParam(classNode, "bindMatrix", bindNotiFy->m_bindMatrix);
	//		xmlSaveParam(classNode, "capSpeed", bindNotiFy->m_capSpeed);
	//	}
	//};

	ndBindingRagdollEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody, ndFloat32 campSpeed);

	void OnTransform(ndInt32, const ndMatrix& matrix);
	void OnApplyExternalForce(ndInt32 thread, ndFloat32 timestep);

	ndMatrix m_bindMatrix;
	ndFloat32 m_capSpeed;
};


#endif
