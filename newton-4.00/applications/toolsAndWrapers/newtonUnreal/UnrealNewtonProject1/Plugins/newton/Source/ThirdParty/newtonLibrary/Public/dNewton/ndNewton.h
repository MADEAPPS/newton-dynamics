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

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#ifndef __ND_NEWTON_H__
#define __ND_NEWTON_H__

#include <ndNewtonStdafx.h>
#include <ndWorld.h>
#include <ndJointList.h>
#include <ndWorldScene.h>
#include <ndConstraint.h>
#include <ndBodyNotify.h>
#include <ndBodyDynamic.h>
#include <ndContactArray.h>
#include <ndBodySphFluid.h>
#include <ndSkeletonList.h>
#include <ndBodyKinematic.h>
#include <ndContactSolver.h>
#include <ndDynamicsUpdate.h>
#include <ndSkeletonContainer.h>
#include <ndDynamicsUpdateSoa.h>

#include <dJoints/ndJointGear.h>
#include <dJoints/ndJointHinge.h>
#include <dJoints/ndJointPlane.h>
#include <dJoints/ndJointWheel.h>
#include <dJoints/ndJointRoller.h>
#include <dJoints/ndJointSlider.h>
#include <dJoints/ndJointPulley.h>
#include <dJoints/ndJointFix6dof.h>
#include <dJoints/ndJointCylinder.h>
#include <dJoints/ndJointUpVector.h>
#include <dJoints/ndJointSpherical.h>
#include <dJoints/ndJointFollowPath.h>
#include <dJoints/ndJointDoubleHinge.h>
#include <dJoints/ndJointFixDistance.h>
#include <dJoints/ndJointDryRollingFriction.h>
#include <dJoints/ndJointKinematicController.h>

#include <dIkSolver/ndIkSolver.h>
#include <dIkSolver/ndIkJointHinge.h>
#include <dIkSolver/ndIk6DofEffector.h>
#include <dIkSolver/ndIkJointSpherical.h>
#include <dIkSolver/ndIkJointDoubleHinge.h>
#include <dIkSolver/ndIkSwivelPositionEffector.h>

#include <dModels/ndModel.h>
#include <dModels/ndUrdfFile.h>
#include <dModels/ndModelList.h>
#include <dModels/ndModelNotify.h>
#include <dModels/ndModelArticulation.h>
#include <dModels/dVehicle/ndMultiBodyVehicle.h>
#include <dModels/dVehicle/ndMultiBodyVehicleMotor.h>
#include <dModels/dVehicle/ndMultiBodyVehicleGearBox.h>
#include <dModels/dVehicle/ndMultiBodyVehicleTireJoint.h>
#include <dModels/dVehicle/ndMultiBodyVehicleTorsionBar.h>
#include <dModels/dVehicle/ndMultiBodyVehicleDifferential.h>
#include <dModels/dVehicle/ndMultiBodyVehicleDifferentialAxle.h>

#endif 

