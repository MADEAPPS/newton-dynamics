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

#ifndef __D_CHARACTER_H__
#define __D_CHARACTER_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"

class ndWorld;
class ndCharacterLimbNode;
class ndCharacterRootNode;
class ndCharacterEffectorNode;
class ndCharacterFowardDynamicNode;
class ndCharacterInverseDynamicNode;

class ndCharacter: public ndModel
{
	public:
	//class ndLinearState
	//{
	//	public:
	//	dVector m_centerOfMass;
	//	dVector m_centerOfMassVeloc;
	//	dVector m_centerOfMassOmega;
	//	dFloat32 m_mass;
	//};

	class ndLinearState
	{
		public:
		dVector m_centerOfMass;
		dVector m_centerOfMassVeloc;
		dFloat32 m_mass;
	};

	D_CLASS_RELECTION(ndCharacter);

	D_NEWTON_API ndCharacter();
	D_NEWTON_API ndCharacter(const nd::TiXmlNode* const xmlNode);
	D_NEWTON_API virtual ~ndCharacter ();

	D_NEWTON_API ndCharacterRootNode* CreateRoot(ndBodyDynamic* const body);
	D_NEWTON_API ndCharacterFowardDynamicNode* CreateFowardDynamicLimb(const dMatrix& matrixInGlobalScape, ndBodyDynamic* const body, ndCharacterLimbNode* const parent);
	D_NEWTON_API ndCharacterInverseDynamicNode* CreateInverseDynamicLimb(const dMatrix& matrixInGlobalScape, ndBodyDynamic* const body, ndCharacterLimbNode* const parent);
	D_NEWTON_API ndCharacterEffectorNode* CreateInverseDynamicEffector(const dMatrix& matrixInGlobalScape, ndCharacterLimbNode* const child, ndCharacterLimbNode* const parent);
	
	ndCharacter* GetAsCharacter();

	protected:
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;
	D_NEWTON_API virtual void Update(ndWorld* const world, dFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndWorld* const world, dFloat32 timestep);

	ndLinearState CalculateCom() const;
	inline void UpdateGlobalPose(ndWorld* const world, dFloat32 timestep);
	
	ndCharacterRootNode* m_rootNode;
};

inline ndCharacter* ndCharacter::GetAsCharacter()
{
	return this;
}

#endif