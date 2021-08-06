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
#include "ndCharacter.h"
#include "ndBodyDynamic.h"
#include "ndCharacterRootNode.h"

ndCharacterRootNode::ndCharacterRootNode(ndCharacter* const owner, ndBodyDynamic* const body)
	:ndCharacterLimbNode(nullptr)
	,m_owner(owner)
	,m_body(body)
	,m_localFrame(dGetIdentityMatrix())
	,m_globalPose(dGetIdentityMatrix())
{
	SetLocalFrame(m_body->GetMatrix());
}

ndCharacterRootNode::~ndCharacterRootNode()
{
}

void ndCharacterRootNode::SetLocalFrame(const dMatrix& frameInGlobalSpace)
{
	dMatrix matrix(m_body->GetMatrix());
	m_localFrame = frameInGlobalSpace * matrix.Inverse();
}

void ndCharacterRootNode::UpdateGlobalPose(ndWorld* const world, dFloat32 timestep)
{
	// for now just; 
	m_globalPose = m_body->GetMatrix();
}