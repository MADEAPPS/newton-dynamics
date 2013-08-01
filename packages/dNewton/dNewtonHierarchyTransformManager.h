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


#ifndef _D_NEWTON_HIERARCHY_MANAGER_H_
#define _D_NEWTON_HIERARCHY_MANAGER_H_

#include "dStdAfxNewton.h"


class dNewtonHierarchyTransformManager: public CustomSkeletonTransformManager
{
	public:
	class dNewtonHierarchyTransformController: public dNewtonAlloc
	{
		public:
		CNEWTON_API dNewtonHierarchyTransformController (dNewtonHierarchyTransformManager* const manager);
		CNEWTON_API ~dNewtonHierarchyTransformController ();

		private:
		CustomSkeletonTransformController* m_controller;
		friend class dNewtonHierarchyTransformManager;
	};

	CNEWTON_API dNewtonHierarchyTransformManager (dNewton* const world);
	CNEWTON_API virtual ~dNewtonHierarchyTransformManager ();

	NEWTON_API virtual void UpdateTransform (const CustomSkeletonTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
	NEWTON_API void DestroyController (CustomSkeletonTransformController* const controller);


	CNEWTON_API dNewtonHierarchyTransformController* GetFirstController() const;
	CNEWTON_API dNewtonHierarchyTransformController* GetNextController(const dNewtonHierarchyTransformController* const controller) const;
//	CNEWTON_API virtual void EventCallback (const CustomTriggerController* const trigger, TriggerEventType event, NewtonBody* const guess) const;
};


#endif
