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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndCharacter.h"
#include "ndCharacterIkNode.h"
#include "ndCharacterIkRootNode.h"
#include "ndCharacterIkOrganicLimbNode.h"

ndCharacter::ndCharacter()
	:ndModel()
	,m_rootNode(nullptr)
{
}

ndCharacter::ndCharacter(const nd::TiXmlNode* const xmlNode)
	:ndModel(xmlNode)
{
}

ndCharacter::~ndCharacter()
{
	if (m_rootNode)
	{
		delete m_rootNode;
	}
}

ndCharacterIkRootNode* ndCharacter::CreateRoot(ndBodyDynamic* const body)
{
	m_rootNode = new ndCharacterIkRootNode(this, body);
	return m_rootNode;
}

ndCharacterIkOrganicLimbNode* ndCharacter::CreateOrganicLimb(const dMatrix& matrixInGlobalScape, ndBodyDynamic* const body, ndCharacterIkNode* const parent)
{
	ndCharacterIkOrganicLimbNode* const limb = new ndCharacterIkOrganicLimbNode(matrixInGlobalScape, body, parent);
	return limb;
}

//void ndCharacter::Debug(ndConstraintDebugCallback& context) const
void ndCharacter::Debug(ndConstraintDebugCallback&) const
{
}

void ndCharacter::PostUpdate(ndWorld* const, dFloat32)
{
}

void ndCharacter::Update(ndWorld* const, dFloat32)
{
}
