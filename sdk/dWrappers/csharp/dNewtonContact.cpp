/*
* This software is provided 'as-is', without any express or implied
* warranty.In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
*including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions :
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software.If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"
#include "dNewtonContact.h"

void* dNewtonContact::GetContactInfo(void* const body, void* const contact)
{
	NewtonMaterial* mat = NewtonContactGetMaterial(contact);
	NewtonBody* bd = static_cast<NewtonBody*>(body);
	
	NewtonMaterialGetContactForce(mat, bd, &cInfo.force[0]);
	NewtonMaterialGetContactPositionAndNormal(mat, bd, &cInfo.position[0], &cInfo.normal[0]);
	cInfo.normalSpeed = NewtonMaterialGetContactNormalSpeed(mat);

	return &cInfo;
}

ContactInfo dNewtonContact::cInfo;

//NEWTON_API void *NewtonMaterialGetMaterialPairUserData(const NewtonMaterial* const material);
//NEWTON_API unsigned NewtonMaterialGetContactFaceAttribute(const NewtonMaterial* const material);
//NEWTON_API NewtonCollision* NewtonMaterialGetBodyCollidingShape(const NewtonMaterial* const material, const NewtonBody* const body);
//NEWTON_API dFloat NewtonMaterialGetContactNormalSpeed(const NewtonMaterial* const material);
//NEWTON_API void NewtonMaterialGetContactForce(const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const force);
//NEWTON_API void NewtonMaterialGetContactPositionAndNormal(const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const posit, dFloat* const normal);
//NEWTON_API void NewtonMaterialGetContactTangentDirections(const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const dir0, dFloat* const dir1);
//NEWTON_API dFloat NewtonMaterialGetContactTangentSpeed(const NewtonMaterial* const material, int index);
//NEWTON_API dFloat NewtonMaterialGetContactMaxNormalImpact(const NewtonMaterial* const material);
//NEWTON_API dFloat NewtonMaterialGetContactMaxTangentImpact(const NewtonMaterial* const material, int index);
//NEWTON_API dFloat NewtonMaterialGetContactPenetration(const NewtonMaterial* const material);
//
//NEWTON_API void NewtonMaterialSetContactSoftness(const NewtonMaterial* const material, dFloat softness);
//NEWTON_API void NewtonMaterialSetContactThickness(const NewtonMaterial* const material, dFloat thickness);
//NEWTON_API void NewtonMaterialSetContactElasticity(const NewtonMaterial* const material, dFloat restitution);
//NEWTON_API void NewtonMaterialSetContactFrictionState(const NewtonMaterial* const material, int state, int index);
//NEWTON_API void NewtonMaterialSetContactFrictionCoef(const NewtonMaterial* const material, dFloat staticFrictionCoef, dFloat kineticFrictionCoef, int index);
//
//NEWTON_API void NewtonMaterialSetContactNormalAcceleration(const NewtonMaterial* const material, dFloat accel);
//NEWTON_API void NewtonMaterialSetContactNormalDirection(const NewtonMaterial* const material, const dFloat* const directionVector);
//NEWTON_API void NewtonMaterialSetContactPosition(const NewtonMaterial* const material, const dFloat* const position);
//
//NEWTON_API void NewtonMaterialSetContactTangentFriction(const NewtonMaterial* const material, dFloat friction, int index);
//NEWTON_API void NewtonMaterialSetContactTangentAcceleration(const NewtonMaterial* const material, dFloat accel, int index);
//NEWTON_API void NewtonMaterialContactRotateTangentDirections(const NewtonMaterial* const material, const dFloat* const directionVector);
//
//NEWTON_API dFloat NewtonMaterialGetContactPruningTolerance(const NewtonBody* const body0, const NewtonBody* const body1);
//NEWTON_API void NewtonMaterialSetContactPruningTolerance(const NewtonBody* const body0, const NewtonBody* const body1, dFloat tolerance);
