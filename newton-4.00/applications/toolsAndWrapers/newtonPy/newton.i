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
%module newton
#pragma SWIG nowarn=312		//Nested union not supported
#pragma SWIG nowarn=325		//Nested struct not currently supported 
#pragma SWIG nowarn=401		//Nothing known about base class

%begin 
%{
	#pragma warning(disable:4127 4316 4456 4701 4706)
%}

%{
	#include <ndNewton.h>
%}


//%rename(New) dVector::operator new;
//%rename(Delete) dVector::operator delete;
//%rename(NewArray) dVector::operator new[];
//%rename(DeleteArray) dVector::operator delete[];

%rename(GetElement) dVector::operator[](dInt32 i);
%rename(GetElement) dVector::operator[](dInt32 i) const;

%rename(GetElement) dBigVector::operator[](dInt32 i);
%rename(GetElement) dBigVector::operator[](dInt32 i) const;

%rename(GetElement) dMatrix::operator[](dInt32 i);
%rename(GetElement) dMatrix::operator[](dInt32 i) const;

%rename(Assigment) ndShapeInstance::operator=;  

//%template(objInfo) pyBaseNodeInfo<dSceneNodeInfo>;
//%template(meshInfo) pyBaseNodeInfo<dMeshNodeInfo>;
//%template(texInfo) pyBaseNodeInfo<dTextureNodeInfo>;
//%template(matInfo) pyBaseNodeInfo<dMaterialNodeInfo>;
//%template(rigidBidyInfo) pyBaseNodeInfo<dRigidbodyNodeInfo>;
//%include "carrays.i"
//%array_class(int, intArray);
//%array_class(double, doubleArray);

%include "newtonConfig.h"
%include "../../../sdk/dCore/dVectorSimd.h"
%include "../../../sdk/dCore/dMatrix.h"
%include "../../../sdk/dCore/dQuaternion.h"

%include "../../../sdk/dCollision/ndShape.h"
%include "../../../sdk/dCollision/ndShapeBox.h"
%include "../../../sdk/dCollision/ndShapeCone.h"
%include "../../../sdk/dCollision/ndShapeConvex.h"
%include "../../../sdk/dCollision/ndShapeSphere.h"
%include "../../../sdk/dCollision/ndShapeCapsule.h"
%include "../../../sdk/dCollision/ndShapeCompound.h"
%include "../../../sdk/dCollision/ndShapeCylinder.h"
%include "../../../sdk/dCollision/ndShapeStatic_bvh.h"
%include "../../../sdk/dCollision/ndShapeStaticMesh.h"
%include "../../../sdk/dCollision/ndShapeConvexHull.h"
%include "../../../sdk/dCollision/ndShapeHeightfield.h"
%include "../../../sdk/dCollision/ndShapeChamferCylinder.h"

%include "../../../sdk/dCollision/ndShapeInstance.h"

%include "../../../sdk/dCollision/ndBody.h"
%include "../../../sdk/dCollision/ndBodyKinematic.h"
%include "../../../sdk/dCollision/ndBodyTriggerVolume.h"
%include "../../../sdk/dCollision/ndBodyPlayerCapsule.h"

%include "../../../sdk/dCollision/ndBodyNotify.h"
%include "../../../sdk/dCollision/ndContactNotify.h"
%include "../../../sdk/dCollision/ndRayCastNotify.h"
%include "../../../sdk/dCollision/ndBodiesInAabbNotify.h"

%include "../../../sdk/dNewton/ndWorld.h"


