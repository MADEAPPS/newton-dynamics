/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _ND_RIGIB_BODY_GLUE_H_
#define _ND_RIGIB_BODY_GLUE_H_

#include "ndClassAlloc.h"
#include "ndContainersAlloc.h"

class ndMatrixGlue;
class ndBodyKinematic;
class ndBodyNotifyGlue;
class ndShapeInstanceGlue;

enum nRigidBodyType
{
	m_dynamic,
	m_triggerVolume,
	m_playerCapsule,
};

class ndRigidBodyGlue: public ndContainersFreeListAlloc<ndRigidBodyGlue>
{
	public:
	ndRigidBodyGlue(nRigidBodyType type);
	~ndRigidBodyGlue();

	int GetId() const;
	ndBodyNotifyGlue* GetNotifyCallback() const;
	const ndShapeInstanceGlue* GetCollisionShape() const;

	void SetMatrix(const ndMatrixGlue* const matrix);
	void SetNotifyCallback(ndBodyNotifyGlue* const notify);
	void SetCollisionShape(const ndShapeInstanceGlue* const shapeInstance);
	void SetMassMatrix(float mass, const ndShapeInstanceGlue* const shapeInstance);

	private:
	ndBodyKinematic* m_body;
	ndShapeInstanceGlue* m_shapeInstance;
	friend class ndWorldGlue;
};

#endif 

