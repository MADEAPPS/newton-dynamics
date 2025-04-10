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

#ifndef _ND_MODEL_ARTICULATION_H__
#define _ND_MODEL_ARTICULATION_H__

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndModel.h"
#include "dIkSolver/ndIkSolver.h"

class ndMultiBodyVehicle;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndModelArticulation: public ndModel
{
	public: 
	D_CLASS_REFLECTION(ndModelArticulation, ndModelBase)

	class ndNode : public ndNodeHierarchy<ndNode>
	{
		public:
		D_NEWTON_API ndNode(const ndNode& src);
		D_NEWTON_API ndNode(const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint, ndNode* const parent);
		D_NEWTON_API virtual ~ndNode();

		ndSharedPtr<ndBody> m_body;
		ndSharedPtr<ndJointBilateralConstraint> m_joint;
		ndString m_name;
	};

	D_NEWTON_API ndModelArticulation();
	D_NEWTON_API ndModelArticulation(const ndModelArticulation& src);
	D_NEWTON_API virtual ~ndModelArticulation();
	D_NEWTON_API virtual ndModel* Clone() const;

	D_NEWTON_API virtual ndModelArticulation* GetAsModelArticulation();

	D_NEWTON_API ndNode* GetRoot() const;
	D_NEWTON_API ndNode* AddRootBody(const ndSharedPtr<ndBody>& rootBody);
	D_NEWTON_API ndNode* AddLimb(ndNode* const parent, const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint);

	D_NEWTON_API const ndList<ndModelArticulation::ndNode>& GetCloseLoops() const;
	D_NEWTON_API void AddCloseLoop(const ndSharedPtr<ndJointBilateralConstraint>& joint, const char* const name = nullptr);

	D_NEWTON_API virtual void OnAddToWorld() override;
	D_NEWTON_API virtual void OnRemoveFromToWorld() override;

	D_NEWTON_API virtual void AddBodiesAndJointsToWorld() override;
	D_NEWTON_API virtual void RemoveBodiesAndJointsFromWorld() override;

	D_NEWTON_API const ndString& GetName() const;
	D_NEWTON_API void SetName(const ndString& name);
	D_NEWTON_API ndNode* FindByName(const char* const name) const;
	D_NEWTON_API ndNode* FindByBody(const ndBody* const body) const;
	D_NEWTON_API ndNode* FindLoopByName(const char* const name) const;
	D_NEWTON_API ndNode* FindLoopByJoint(const ndJointBilateralConstraint* const joint) const;

	D_NEWTON_API void ClearMemory();
	D_NEWTON_API void SetTransform(const ndMatrix& matrix);

	protected:
	D_NEWTON_API void ConvertToUrdf();

	ndString m_name;
	ndNode* m_rootNode;
	ndList<ndNode> m_closeLoops;

	friend class ndUrdfFile;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

