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

class ndDemoEntityNotify : public ndModelBodyNotify
{
	public:
	ndDemoEntityNotify(const ndDemoEntityNotify& notify);
	ndDemoEntityNotify(
		ndDemoEntityManager* const manager, 
		const ndSharedPtr<ndRenderSceneNode>& entity, 
		ndBodyKinematic* const parentBody = nullptr, 
		const ndVector& gravity = ndVector(0.0f, DEMO_GRAVITY, 0.0f, 0.0f));

	ndBodyNotify* Clone() const
	{
		return new ndDemoEntityNotify(*this);
	}

	virtual ~ndDemoEntityNotify();

	ndSharedPtr<ndRenderSceneNode> GetUserData() const
	{
		return m_entity;
	}

	void RemoveBody();
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);

	ndDemoEntityManager* m_manager;
	ndSharedPtr<ndRenderSceneNode> m_entity;
	ndTransform m_transform;
	ndMatrix m_bindMatrix;
	ndFloat32 m_capSpeed;
};

//class ndBindingRagdollEntityNotify : public ndDemoEntityNotify
//{
//	public:
//	ndBindingRagdollEntityNotify(ndDemoEntityManager* const manager, const ndSharedPtr<ndRenderSceneNode>& entity, ndBodyDynamic* const parentBody, ndFloat32 campSpeed);
//	~ndBindingRagdollEntityNotify();
//
//	void OnTransform(ndInt32, const ndMatrix& matrix);
//	void OnApplyExternalForce(ndInt32 thread, ndFloat32 timestep);
//
//	ndMatrix m_bindMatrix;
//	ndFloat32 m_capSpeed;
//};
#endif
