/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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


#ifndef _D_NEWTON_ARTICULATED_MANAGER_H_
#define _D_NEWTON_ARTICULATED_MANAGER_H_

#include "dStdAfxNewton.h"

class dNewtonBody;

class dNewtonArticulationManager: public dCustomArticulaledTransformManager
{
	public:
	class dNewtonArticulationController: public dNewtonAlloc
	{
		public:
		CNEWTON_API dNewtonArticulationController (dNewtonArticulationManager* const manager);
		CNEWTON_API ~dNewtonArticulationController ();

		CNEWTON_API virtual void OnPreUpdate (dFloat timestep) = 0;
		CNEWTON_API virtual void OnUpdateBoneTransform (dNewtonBody* const bone, const dFloat* const localMatrix) = 0;
		CNEWTON_API virtual void* AddBone (dNewtonBody* const bone, const dFloat* const bindMatrix, void* const parentBodne = NULL);

		CNEWTON_API int GetBoneCount() const;
		CNEWTON_API void* GetBone(int bonexIndex) const;
		CNEWTON_API dNewtonBody* GetBoneBody (int index) const;
		CNEWTON_API dNewtonBody* GetBoneBody (void* const bone) const;
		CNEWTON_API void* GetBoneParent (const void* const bone) const;

		CNEWTON_API void DisableAllSelfCollision ();
		CNEWTON_API void SetDefaultSelfCollisionMask ();
		CNEWTON_API void SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode);
		CNEWTON_API bool SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const;
		

		private:
		dCustomArticulatedTransformController* m_controller;
		friend class dNewtonArticulationManager;
	};

	CNEWTON_API dNewtonArticulationManager (dNewton* const world);
	CNEWTON_API virtual ~dNewtonArticulationManager ();

	CNEWTON_API void DisableAllSelfCollision (dCustomArticulatedTransformController* const controller);
	CNEWTON_API void SetDefaultSelfCollisionMask (dCustomArticulatedTransformController* const controller);
	CNEWTON_API void SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode);
	CNEWTON_API bool SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const;

	CNEWTON_API virtual void OnPreUpdate (dCustomArticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const;
	CNEWTON_API virtual void OnUpdateTransform (const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const;
	CNEWTON_API void DestroyController (dCustomArticulatedTransformController* const controller);

	CNEWTON_API dNewtonArticulationController* GetFirstController() const;
	CNEWTON_API dNewtonArticulationController* GetNextController(const dNewtonArticulationController* const controller) const;
};


#endif
