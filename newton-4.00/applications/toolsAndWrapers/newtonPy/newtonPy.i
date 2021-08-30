/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

// File : example.i 
%module newtonPy
%{
	#include <ndNewton.h>
%}

#define D_CORE_API 
#define D_NEWTON_API
#define D_COLLISION_API
#define D_MSV_NEWTON_ALIGN_32
#define D_GCC_NEWTON_ALIGN_32

%include "../../../sdk/dCore/dClassAlloc.h"
%include "../../../sdk/dCollision/ndBody.h"
%include "../../../sdk/dNewton/ndWorld.h"

//%template(objInfo) pyBaseNodeInfo<dSceneNodeInfo>;
//%template(meshInfo) pyBaseNodeInfo<dMeshNodeInfo>;
//%template(texInfo) pyBaseNodeInfo<dTextureNodeInfo>;
//%template(matInfo) pyBaseNodeInfo<dMaterialNodeInfo>;
//%template(rigidBidyInfo) pyBaseNodeInfo<dRigidbodyNodeInfo>;

//%include "carrays.i"
//%include "pyRigidBody.h"
//%array_class(int, intArray);
//%array_class(double, doubleArray);

