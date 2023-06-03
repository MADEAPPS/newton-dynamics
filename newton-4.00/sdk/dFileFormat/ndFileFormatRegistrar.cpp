/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndFileFormatStdafx.h"
#include "ndFileFormatRegistrar.h"

#include "ndFileFormatBody.h"
#include "ndFileFormatWorld.h"
#include "ndFileFormatShape.h"
#include "ndFileFormatJoint.h"
#include "ndFileFormatModel.h"
#include "ndFileFormatNotify.h"
#include "ndFileFormatShapeNull.h"
#include "ndFileFormatJointGear.h"
#include "ndFileFormatJointPlane.h"
#include "ndFileFormatJointHinge.h"
#include "ndFileFormatJointWheel.h"
#include "ndFileFormatJointSlider.h"
#include "ndFileFormatJointPulley.h"
#include "ndFileFormatShapeConvex.h"
#include "ndFileFormatBodyDynamic.h"
#include "ndFileFormatModelNotify.h"
#include "ndFileFormatJointRoller.h"
#include "ndFileFormatJointFix6dof.h"
#include "ndFileFormatJointIkHinge.h"
#include "ndFileFormatBodyKinematic.h"
#include "ndFileFormatShapeInstance.h"
#include "ndFileFormatShapeCompound.h"
#include "ndFileFormatJointCylinder.h"
#include "ndFileFormatJointUpVector.h"
#include "ndFileFormatJointSpherical.h"
#include "ndFileFormatShapeConvexBox.h"
#include "ndFileFormatShapeConvexCone.h"
#include "ndFileFormatShapeConvexHull.h"
#include "ndFileFormatShapeStaticMesh.h"
#include "ndFileFormatJointFollowPath.h"
#include "ndFileFormatJointDoubleHinge.h"
#include "ndFileFormatJointFixDistance.h"
#include "ndFileFormatJointIkSpherical.h"
#include "ndFileFormatJointVehicleTire.h"
#include "ndFileFormatModelArticulation.h"
#include "ndFileFormatJointVehicleMotor.h"
#include "ndFileFormatBodyTriggerVolume.h"
#include "ndFileFormatBodyKinematicBase.h"
#include "ndFileFormatShapeConvexSphere.h"
#include "ndFileFormatShapeConvexCapsule.h"
#include "ndFileFormatJointIkDoubleHinge.h"
#include "ndFileFormatJointIk6DofEffector.h"
#include "ndFileFormatShapeConvexCylinder.h"
#include "ndFileFormatShapeStaticMesh_bvh.h"
#include "ndFileFormatJointVehicleGearBox.h"
#include "ndFileFormatModelPassiveRagdoll.h"
#include "ndFileFormatJointRollingFriction.h"
#include "ndFileFormatJointVehicleTorsionBar.h"
#include "ndFileFormatShapeStaticHeightfield.h"
#include "ndFileFormatJointKinematicController.h"
#include "ndFileFormatJointVehicleDifferential.h"
#include "ndFileFormatShapeStaticProceduralMesh.h"
#include "ndFileFormatShapeConvexChamferCylinder.h"
#include "ndFileFormatBodyKinematicPlayerCapsule.h"
#include "ndFileFormatJointVehicleDifferentialAxle.h"
#include "ndFileFormatJointIkSwivelPositionEffector.h"


ndFixSizeArray<ndFileFormatRegistrar*, 256> ndFileFormatRegistrar::m_registry;

ndFileFormatRegistrar::ndFileFormatRegistrar(const char* const className)
	:ndClassAlloc()
	,m_hash(ndCRC64(className))
{
	ndAssert(!GetHandler(className));

	m_registry.PushBack(this);
	for (ndInt32 i = m_registry.GetCount() - 2; i >= 0; --i)
	{
		ndFileFormatRegistrar* const entry = m_registry[i];
		if (entry->m_hash > m_hash)
		{
			m_registry[i] = this;
			m_registry[i + 1] = entry;
		}
		else
		{
			break;
		}
	}
}

ndFileFormatRegistrar::~ndFileFormatRegistrar()
{
}

void ndFileFormatRegistrar::Init()
{
	static ndFileFormatShapeNull shapeNull;
	static ndFileFormatShapeConvex shapeConvex;
	static ndFileFormatShapeConvexBox shapeBox;
	static ndFileFormatShapeConvexCone shapeCone;
	static ndFileFormatShapeCompound shapeCompound;
	static ndFileFormatShapeConvexSphere shapeSphere;
	static ndFileFormatShapeConvexCapsule shapeCapsule;
	static ndFileFormatShapeConvexHull shapeConvexHull;
	static ndFileFormatShapeStaticMesh shapeStaticMesh;
	static ndFileFormatShapeConvexCylinder shapeCylinder;
	static ndFileFormatShapeStaticMesh_bvh shapeStaticBvh;
	static ndFileFormatJointIk6DofEffector ik6DofEffector;
	static ndFileFormatShapeStaticHeightfield shapeHeightfield;
	static ndFileFormatShapeStaticProceduralMesh shapeProceduralMesh;
	static ndFileFormatShapeConvexChamferCylinder shapeChamferCylinder;

	static ndFileFormatBody body;
	static ndFileFormatShape shape;
	static ndFileFormatWorld world;
	static ndFileFormatNotify bodyNotiy;
	static ndFileFormatModelNotify modelNotify;
	static ndFileFormatBodyDynamic dynamicBody;
	static ndFileFormatShapeInstance collision;
	static ndFileFormatBodyKinematic kinematicBody;
	static ndFileFormatBodyTriggerVolume triggerVolume;
	static ndFileFormatBodyKinematicBase kinematicBodyBase;
	static ndFileFormatBodyKinematicPlayerCapsule kinematicPlayerCapsule;

	static ndFileFormatJoint joint;
	static ndFileFormatJointGear jointGear;
	static ndFileFormatJointHinge jointHinge;
	static ndFileFormatJointPlane jointPlane;
	static ndFileFormatJointWheel jopintWheel;
	static ndFileFormatJointSlider jointSlider;
	static ndFileFormatJointRoller jointRoller;
	static ndFileFormatJointPulley jointPulley;
	static ndFileFormatJointFix6dof jointFix6Dof;
	static ndFileFormatJointIkHinge jointIkHinge;
	static ndFileFormatJointUpVector jointUpVector;
	static ndFileFormatJointCylinder jointCylinder;
	static ndFileFormatJointSpherical jointSpherical;
	static ndFileFormatJointFollowPath jointFollowPath;
	static ndFileFormatJointIkSpherical jointIkSpherical;
	static ndFileFormatJointDoubleHinge jointDoubleHinge;
	static ndFileFormatJointFixDistance jointFixDistance;
	static ndFileFormatJointVehicleTire jointVehicleTire;
	static ndFileFormatJointVehicleMotor jointVehicleMotor;
	static ndFileFormatJointIkDoubleHinge jointIkDoubleHinge;
	static ndFileFormatJointVehicleGearBox jointVehicleGearBox;
	static ndFileFormatJointRollingFriction jointRollingFriction;
	static ndFileFormatJointVehicleTorsionBar jointVehicleTorsionBar;
	static ndFileFormatJointVehicleDifferential jointVehicleDiffrential;
	static ndFileFormatJointKinematicController jointkinematicController;
	static ndFileFormatJointVehicleDifferentialAxle jointVehicleDiffAxle;
	static ndFileFormatJointIkSwivelPositionEffector jointIkSwivelPositionEffector;

	static ndFileFormatModel model;
	static ndFileFormatModelArticulation modelArticulation;
	static ndFileFormatModelPassiveRagdoll modelPassiveRagdoll;
}

ndFileFormatRegistrar* ndFileFormatRegistrar::GetHandler(const char* const className)
{
	ndUnsigned64 hash = ndCRC64(className);
	ndInt32 i0 = 0;
	ndInt32 i1 = m_registry.GetCount() - 1;
	while ((i1 - i0 > 4))
	{
		ndInt32 mid = (i1 + i0) / 2;
		if (m_registry[mid]->m_hash <= hash)
		{
			i0 = mid;
		}
		else
		{
			i1 = mid;
		}
	}
	
	for (ndInt32 i = i0; i <= i1; ++i)
	{
		if (m_registry[i]->m_hash == hash)
		{
			return m_registry[i];
		}
	}

	return nullptr;
}

void ndFileFormatRegistrar::SaveWorld(ndFileFormatSave* const, nd::TiXmlElement* const, const ndWorld* const)
{
	ndAssert(0);
}

void ndFileFormatRegistrar::SaveBody(ndFileFormatSave* const, nd::TiXmlElement* const, const ndBody* const)
{
	ndAssert(0);
}

void ndFileFormatRegistrar::SaveNotify(ndFileFormatSave* const, nd::TiXmlElement* const, const ndBodyNotify* const)
{
	ndAssert(0);
}

ndInt32 ndFileFormatRegistrar::SaveShape(ndFileFormatSave* const, nd::TiXmlElement* const, const ndShape* const)
{
	ndAssert(0);
	return 0;
}

void ndFileFormatRegistrar::SaveCollision(ndFileFormatSave* const, nd::TiXmlElement* const, const ndShapeInstance* const)
{
	ndAssert(0);
}

void ndFileFormatRegistrar::SaveJoint(ndFileFormatSave* const, nd::TiXmlElement* const, const ndJointBilateralConstraint* const)
{
	ndAssert(0);
}

void ndFileFormatRegistrar::SaveModel(ndFileFormatSave* const, nd::TiXmlElement* const, const ndModel* const)
{
	ndAssert(0);
}

ndShape* ndFileFormatRegistrar::LoadShape(const nd::TiXmlElement* const, const ndTree<ndShape*, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

ndBody* ndFileFormatRegistrar::LoadBody(const nd::TiXmlElement* const, const ndTree<ndShape*, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

ndShapeInstance* ndFileFormatRegistrar::LoadCollision(const nd::TiXmlElement* const, const ndTree<ndShape*, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

ndBodyNotify* ndFileFormatRegistrar::LoadNotify(const nd::TiXmlElement* const)
{
	ndAssert(0);
	return nullptr;
}

ndJointBilateralConstraint* ndFileFormatRegistrar::LoadJoint(const nd::TiXmlElement* const, const ndTree<ndSharedPtr<ndBody>, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

ndModel* ndFileFormatRegistrar::LoadModel(const nd::TiXmlElement* const, const ndTree<ndSharedPtr<ndBody>, ndInt32>&, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}