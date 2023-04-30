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

#include "ndModelStdafx.h"
#include "ndModelPassiveRagdoll.h"

ndModelPassiveRagdoll::ndModelPassiveRagdoll()
	:ndModelArticulation()
{
}

ndModelPassiveRagdoll::~ndModelPassiveRagdoll()
{
}

void ndModelPassiveRagdoll::NormalizeMassDistribution(ndFloat32 totalMass)
{
	ndFixSizeArray<ndBodyDynamic*, 256> bodyArray;
	ndFixSizeArray<ndModelPassiveRagdoll::ndNode*, 256> stack;
	if (GetRoot())
	{
		stack.PushBack(GetRoot());
		while (stack.GetCount())
		{
			ndInt32 index = stack.GetCount() - 1;
			ndModelPassiveRagdoll::ndNode* const node = stack[index];
			stack.SetCount(index);

			bodyArray.PushBack(node->m_body->GetAsBodyDynamic());
			for (ndModelPassiveRagdoll::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
			{
				stack.PushBack(child);
			}
		}
	}

	ndFloat32 volume = 0.0f;
	for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
	{
		//volume += bodyArray[i]->GetCollisionShape().GetVolume() * massWeight[i];
		volume += bodyArray[i]->GetCollisionShape().GetVolume();
	}
	ndFloat32 density = totalMass / volume;
	
	for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
	{
		ndBodyDynamic* const body = bodyArray[i];
		//ndFloat32 scale = density * body->GetCollisionShape().GetVolume() * massWeight[i];
		ndFloat32 scale = density * body->GetCollisionShape().GetVolume();
		ndVector inertia(body->GetMassMatrix().Scale(scale));
		body->SetMassMatrix(inertia);
	}
}
