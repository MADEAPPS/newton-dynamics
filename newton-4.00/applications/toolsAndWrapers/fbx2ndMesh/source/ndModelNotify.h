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

#ifndef __ND_MODEL_NOTIFY_H__
#define __ND_MODEL_NOTIFY_H__

#include "ndNewtonStdafx.h"

class ndModel;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndModelNotify : public ndContainersFreeListAlloc<ndModelNotify>
{
	public:  
	D_BASE_CLASS_REFLECTION(ndModelNotify)

	D_NEWTON_API ndModelNotify();
	D_NEWTON_API ndModelNotify(const ndModelNotify& src);

	D_NEWTON_API virtual ~ndModelNotify();

	D_NEWTON_API ndModel* GetModel() const;
	D_NEWTON_API void SetModel(ndModel* const model);

	D_NEWTON_API virtual ndModelNotify* Clone() const;
	D_NEWTON_API virtual void Update(ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void PostTransformUpdate(ndFloat32 timestep);

	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;

	private:
	ndModel* m_model;

	friend class ndModel;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

