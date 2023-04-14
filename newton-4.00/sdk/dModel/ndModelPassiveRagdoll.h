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

#ifndef _ND_MODEL_PASSIVE_RAGDOLL_H__
#define _ND_MODEL_PASSIVE_RAGDOLL_H__

#include "ndModelStdafx.h"
#include "ndModelBase.h"

class ndModelPassiveRagdoll: public ndModelBase
{
	public: 
	D_CLASS_REFLECTION(ndModelPassiveRagdoll, ndModelBase)

	class ndRagdollNode : public ndNodeHierarchy<ndRagdollNode>
	{
		public:
		ndRagdollNode(ndBodyDynamic* const body, ndRagdollNode* const parent);
		virtual ~ndRagdollNode();
		
		ndBodyDynamic* m_body;
	};

	ndModelPassiveRagdoll();
	virtual ~ndModelPassiveRagdoll();

	ndRagdollNode* GetRoot() const;
	ndRagdollNode* AddRootBody(ndSharedPtr<ndBody>& rootBody);
	ndRagdollNode* AddLimb(ndRagdollNode* const parent, ndSharedPtr<ndBody>& body, ndSharedPtr<ndJointBilateralConstraint>& joint);

	void NormalizeMassDistribution(ndFloat32 totalMass);

	protected:
	ndRagdollNode* m_rootNode;
};

#endif 

