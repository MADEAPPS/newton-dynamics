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

class dNewtonBody;

class dNewtonHierarchyTransformManager: public CustomHierarchicalTransformManager
{
	public:
	class dNewtonHierarchyTransformController: public dNewtonAlloc
	{
		public:
		CNEWTON_API dNewtonHierarchyTransformController (dNewtonHierarchyTransformManager* const manager);
		CNEWTON_API ~dNewtonHierarchyTransformController ();

		NEWTON_API virtual void OnPreUpdate (dFloat timestep) = 0;
		NEWTON_API virtual void OnUpdateBoneTransform (dNewtonBody* const bone, const dFloat* const localMatrix) = 0;
		NEWTON_API virtual void* AddBone (dNewtonBody* const bone, const dFloat* const bindMatrix, void* const parentBodne = NULL);

		NEWTON_API void DisableAllSelfCollision ();
		NEWTON_API void SetDefaultSelfCollisionMask ();
		NEWTON_API void SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode);
		NEWTON_API bool SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const;

		private:
		CustomHierarchicalTransformController* m_controller;
		friend class dNewtonHierarchyTransformManager;
	};

	CNEWTON_API dNewtonHierarchyTransformManager (dNewton* const world);
	CNEWTON_API virtual ~dNewtonHierarchyTransformManager ();

	NEWTON_API void DisableAllSelfCollision (CustomHierarchicalTransformController* const controller);
	NEWTON_API void SetDefaultSelfCollisionMask (CustomHierarchicalTransformController* const controller);
	NEWTON_API void SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode);
	NEWTON_API bool SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const;

	NEWTON_API virtual void OnPreUpdate (CustomHierarchicalTransformController* const constroller, dFloat timestep, int threadIndex) const;
	NEWTON_API virtual void OnUpdateTransform (const CustomHierarchicalTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
	NEWTON_API void DestroyController (CustomHierarchicalTransformController* const controller);

	CNEWTON_API dNewtonHierarchyTransformController* GetFirstController() const;
	CNEWTON_API dNewtonHierarchyTransformController* GetNextController(const dNewtonHierarchyTransformController* const controller) const;
};


#endif
