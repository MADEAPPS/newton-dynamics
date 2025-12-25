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
#pragma SWIG nowarn=312		//nested union not supported
#pragma SWIG nowarn=325		//nested struct not currently supported 
#pragma SWIG nowarn=389		//operator[] ignored
#pragma SWIG nowarn=401		//nothing known about base class


%begin 

%{
	#pragma warning(disable:4127 4316 4365 4456 4701 4706)
%}

%include "stdint.i"
%apply int32_t { ndInt32 }
%apply float { ndFloat32 }

%{
	//#include <ndNewtonInc.h>
	#include <newtonWorld.h>
	//#include "newtonConfig.h"
%}

//%include "newtonConfig.h"
//%rename(GetElement) ndVector::operator[](ndInt32 i);
//%rename(GetElement) ndVector::operator[](ndInt32 i) const;
//%rename(Add) ndVector::operator+(const ndVector& src) const;
//%rename(Sub) ndVector::operator-(const ndVector& src) const;
//%rename(Mul) ndVector::operator*(const ndVector& src) const;
//%rename(AddEqual) ndVector::operator+=(const ndVector& src);
//%rename(SubEqual) ndVector::operator-=(const ndVector& src);
//%rename(MulEqual) ndVector::operator*=(const ndVector& src);
//%rename(Or) ndVector::operator|(const ndVector& src) const;
//%rename(And) ndVector::operator&(const ndVector& src) const;
//%rename(Xor) ndVector::operator^(const ndVector& src) const;
//%rename(Less) ndVector::operator<(const ndVector& src) const;
//%rename(Greather) ndVector::operator>(const ndVector& src) const;
//%rename(Identical) ndVector::operator==(const ndVector& src) const;
//%rename(LessEqual) ndVector::operator<=(const ndVector& src) const;
//%rename(GreatherEqual) ndVector::operator>=(const ndVector& src) const;
//%rename(GetElement) ndBigVector::operator[](ndInt32 i);
//%rename(GetElement) ndBigVector::operator[](ndInt32 i) const;
//%rename(Add) ndBigVector::operator+(const ndBigVector& src) const;
//%rename(Sub) ndBigVector::operator-(const ndBigVector& src) const;
//%rename(Mul) ndBigVector::operator*(const ndBigVector& src) const;
//%rename(AddEqual) ndBigVector::operator+=(const ndBigVector& src);
//%rename(SubEqual) ndBigVector::operator-=(const ndBigVector& src);
//%rename(MulEqual) ndBigVector::operator*=(const ndBigVector& src);
//%rename(Or) ndBigVector::operator|(const ndBigVector& src) const;
//%rename(And) ndBigVector::operator&(const ndBigVector& src) const;
//%rename(Xor) ndBigVector::operator^(const ndBigVector& src) const;
//%rename(Less) ndBigVector::operator<(const ndBigVector& src) const;
//%rename(Greather) ndBigVector::operator>(const ndBigVector& src) const;
//%rename(Identical) ndBigVector::operator==(const ndBigVector& src) const;
//%rename(LessEqual) ndBigVector::operator<=(const ndBigVector& src) const;
//%rename(GreatherEqual) ndBigVector::operator>=(const ndBigVector& src) const;
//%rename(GetElement) ndMatrix::operator[](ndInt32 i);
//%rename(GetElement) ndMatrix::operator[](ndInt32 i) const;
//%rename(GetElement) ndMatrix::operator*(const ndMatrix& src) const;
//%rename(Add) ndQuaternion::operator+(const ndQuaternion& src) const;
//%rename(Sub) ndQuaternion::operator-(const ndQuaternion& src) const;
//%rename(Mul) ndQuaternion::operator*(const ndQuaternion& src) const;

%rename(Assigment) ndShapeInstance::operator=;  
%rename(Create) ndShapeStatic_bvh::operator new;  
%rename(Destroy) ndShapeStatic_bvh::operator delete;  

//%template(objInfo) pyBaseNodeInfo<dSceneNodeInfo>;
//%template(meshInfo) pyBaseNodeInfo<dMeshNodeInfo>;
//%template(texInfo) pyBaseNodeInfo<dTextureNodeInfo>;
//%template(matInfo) pyBaseNodeInfo<dMaterialNodeInfo>;
//%template(rigidBidyInfo) pyBaseNodeInfo<dRigidbodyNodeInfo>;
//%include "carrays.i"
//%array_class(int, intArray);
//%array_class(double, doubleArray);

%rename(Create) ndClassAlloc::operator new;  
%rename(Destroy) ndClassAlloc::operator delete;  
%rename(CreateArray) ndClassAlloc::operator new[];
%rename(DestroyArray) ndClassAlloc::operator delete[];


//%include "install/source/ndNewtonInc.h"
//%include "install/source/ndTypes.h"
//%include "install/source/ndCollisionStdafx.h"
//
//%include "install/source/ndClassAlloc.h"
//%include "install/source/ndVector.h"
//%include "install/source/ndMatrix.h"
//%include "install/source/ndQuaternion.h"
//
//%include "install/source/ndShape.h"
//%include "install/source/ndShapeBox.h"
//%include "install/source/ndShapeCone.h"
//%include "install/source/ndShapeConvex.h"
//%include "install/source/ndShapeSphere.h"
//%include "install/source/ndShapeCapsule.h"
//%include "install/source/ndShapeCompound.h"
//%include "install/source/ndShapeCylinder.h"
//%include "install/source/ndShapeStatic_bvh.h"
//%include "install/source/ndShapeStaticMesh.h"
//%include "install/source/ndShapeConvexHull.h"
//%include "install/source/ndShapeHeightfield.h"
//%include "install/source/ndShapeChamferCylinder.h"
//
//%include "install/source/ndShapeInstance.h"
//
//%include "install/source/ndBody.h"
//%include "install/source/ndBodyKinematic.h"
//%include "install/source/ndBodyTriggerVolume.h"
//%include "install/source/ndBodyPlayerCapsule.h"
//%include "install/source/ndBodyKinematicBase.h"
//
//%include "install/source/ndBodyNotify.h"
//%include "install/source/ndContactNotify.h"
//%include "install/source/ndRayCastNotify.h"
//%include "install/source/ndBodiesInAabbNotify.h"

%include "newtonWorld.h"

