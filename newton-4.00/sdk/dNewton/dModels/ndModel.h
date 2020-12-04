/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_MODEL_H__
#define __D_MODEL_H__

#include "ndNewtonStdafx.h"
#include "ndModelList.h"

class ndConstraintDebugCallback;

D_MSV_NEWTON_ALIGN_32
class ndModel: public dClassAlloc
{
	public:
	ndModel();
	D_NEWTON_API ndModel(const nd::TiXmlNode* const xmlNode);
	virtual ~ndModel ();

	virtual void Debug(ndConstraintDebugCallback& context) const;

	protected:
	virtual void Update(const ndWorld* const world, dFloat32 timestep) = 0;
	ndModelList::dListNode* m_node;

	friend class ndWorld;
} D_GCC_NEWTON_ALIGN_32;

inline ndModel::ndModel()
	:dClassAlloc()
	,m_node(nullptr)
{
}

inline ndModel::~ndModel()
{
	dAssert(!m_node);
}

inline void ndModel::Debug(ndConstraintDebugCallback& context) const
{

}
#endif 


