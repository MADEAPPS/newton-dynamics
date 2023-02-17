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
class ndAnimKeyframe;
class ndShaderCache;
class ndDemoMeshInterface;

class ndDemoEntityNotify: public ndBodyNotify
{
	public:
	class ndFileDemoEntityNotify: public ndFileFormatNotify
	{
		public:
		ndFileDemoEntityNotify()
			:ndFileFormatNotify(ndDemoEntityNotify::StaticClassName())
		{
		}

		void SaveNotify(nd::TiXmlElement* const parentNode, ndBodyNotify* const notify)
		{
			nd::TiXmlElement* const classNode = new nd::TiXmlElement(ndDemoEntityNotify::StaticClassName());
			parentNode->LinkEndChild(classNode);
			ndFileFormatNotify::SaveNotify(classNode, notify);

			nd::TiXmlElement* const visualNode = new nd::TiXmlElement("visual");
			classNode->LinkEndChild(visualNode);
			xmlSaveParam(visualNode, "useCollisionforVisual", 1);
		}
	};

	D_CLASS_REFLECTION(ndDemoEntityNotify, ndBodyNotify)
	ndDemoEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody = nullptr, ndFloat32 gravity = DEMO_GRAVITY);
	virtual ~ndDemoEntityNotify();

	void* GetUserData() const
	{
		return m_entity;
	}

	virtual void OnObjectPick() const;
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	void RemoveBody();
	bool CheckInWorld(const ndMatrix& matrix) const
	{
		return matrix.m_posit.m_y > -100.0f;
	}

	ndDemoEntity* m_entity;
	ndBodyKinematic* m_parentBody;
	ndDemoEntityManager* m_manager;
};

class ndBindingRagdollEntityNotify : public ndDemoEntityNotify
{
	public:
	D_CLASS_REFLECTION(ndBindingRagdollEntityNotify, ndDemoEntityNotify)
	ndBindingRagdollEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody, ndFloat32 campSpeed);

	void OnTransform(ndInt32, const ndMatrix& matrix);
	void OnApplyExternalForce(ndInt32 thread, ndFloat32 timestep);

	ndMatrix m_bindMatrix;
	ndFloat32 m_capSpeed;
};


#endif
