/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgDeformableBodiesUpdate.h"
#include "dgCollisionDeformableMesh.h"

dgDeformableBodiesUpdate::dgDeformableBodiesUpdate (dgMemoryAllocator* const allocator)
	:dgList<dgCollisionDeformableMesh*>(allocator)
	,m_dictionary(allocator)
{
}

void dgDeformableBodiesUpdate::AddShape(dgCollisionDeformableMesh* const shape)
{
	dgListNode* const node = Append (shape);
	m_dictionary.Insert (node, shape);
}

void dgDeformableBodiesUpdate::RemoveShape(dgCollisionDeformableMesh* const shape)
{
	dgTree <dgListNode*, const dgCollisionDeformableMesh*>::dgTreeNode* const node = m_dictionary.Find (shape);
	dgAssert (node);
	Remove (node->GetInfo());
	m_dictionary.Remove (node);
}


void dgDeformableBodiesUpdate::ApplyExternaForces(dgFloat32 timestep)
{
    for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
        dgCollisionDeformableMesh* const softShape = node->GetInfo();

        if (softShape->GetBody()) {
            softShape->ApplyExternalForces (timestep);
        }
    }
}

void dgDeformableBodiesUpdate::SolveConstraintsAndIntegrate (dgFloat32 timestep)
{
    for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
        dgCollisionDeformableMesh* const softShape = node->GetInfo();

        if (softShape->GetBody()) {
            softShape->ResolvePositionsConstraints (timestep);
        }
    }
}