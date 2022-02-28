/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_CHARACTER_H__
#define __ND_CHARACTER_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"

class ndWorld;
class ndCharacterNode;
class ndIk6DofEffector;
class ndCharacterRootNode;
class ndCharacterSkeleton;
class ndCharacterForwardDynamicNode;
class ndCharacterInverseDynamicNode;

//class ndCharacterCentreOfMassState
//{
//	public:
//	dVector m_centerOfMass;
//	dVector m_centerOfMassVeloc;
//	ndFloat32 m_mass;
//};

class ndCharacter: public ndModel
{
	public:
	class ndEffetorInfo
	{
		public:
		ndMatrix m_bindMatrix;
		ndCharacterNode* m_controlNode;
		ndIk6DofEffector* m_effector;
	};

	D_CLASS_REFLECTION(ndCharacter);
	D_NEWTON_API ndCharacter();
	D_NEWTON_API ndCharacter(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndCharacter ();

	D_NEWTON_API virtual void AddToWorld(ndWorld* const world);
	D_NEWTON_API virtual void RemoveFromToWorld(ndWorld* const world);

	D_NEWTON_API ndCharacterRootNode* CreateRoot(ndBodyDynamic* const body);
	D_NEWTON_API ndCharacterForwardDynamicNode* CreateForwardDynamicLimb(const ndMatrix& matrixInGlobalSpace, ndBodyDynamic* const body, ndCharacterNode* const parent);
	D_NEWTON_API ndCharacterInverseDynamicNode* CreateInverseDynamicLimb(const ndMatrix& matrixInGlobalSpace, ndBodyDynamic* const body, ndCharacterNode* const parent);

	ndCharacter* GetAsCharacter();
	ndCharacterRootNode* GetRootNode() const;

	D_NEWTON_API void AddAttachment(ndJointBilateralConstraint* const joint);
	D_NEWTON_API void RemoveAttachment(ndJointBilateralConstraint* const joint);

	D_NEWTON_API void CreateKinematicChain(const ndMatrix& globalOrientation, const ndCharacterNode* const node);

	//ndCharacterPoseController* GetController() const;
	//void SetController(ndCharacterPoseController* const controller);
	//ndCharacterCentreOfMassState CalculateCentreOfMassState() const;
	//void UpdateGlobalPose(ndWorld* const world, ndFloat32 timestep);
	//void CalculateLocalPose(ndWorld* const world, ndFloat32 timestep);
	//D_NEWTON_API ndCharacterSkeleton* CreateSkeleton() const;
	D_NEWTON_API void SetPose();

	protected:
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;
	D_NEWTON_API virtual void Update(ndWorld* const world, ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndWorld* const world, ndFloat32 timestep);
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	
	ndCharacterRootNode* m_rootNode;
	//ndCharacterPoseController* m_controller;
	ndList<ndEffetorInfo> m_effectors;
	ndList<ndJointBilateralConstraint*> m_extraJointAttachments;
};

inline ndCharacter* ndCharacter::GetAsCharacter()
{
	return this;
}

//inline ndCharacterPoseController* ndCharacter::GetController() const
//{
//	return m_controller;
//}
//
//inline void ndCharacter::SetController(ndCharacterPoseController* const controller)
//{
//	m_controller = controller;
//}

inline ndCharacterRootNode* ndCharacter::GetRootNode() const
{
	return m_rootNode;
}

#endif