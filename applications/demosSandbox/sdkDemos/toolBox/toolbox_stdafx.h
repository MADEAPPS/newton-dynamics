/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently


#ifndef _TOLLBOX_STDAFX_H_
#define _TOLLBOX_STDAFX_H_

// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#define DOMMY_API 

typedef char dInt8;
typedef unsigned char dUnsigned8;

typedef short dInt16;
typedef unsigned short dUnsigned16;

typedef int dInt32;
typedef unsigned dUnsigned32;
typedef unsigned int dUnsigned32;


typedef long long unsigned64;


#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <ctype.h>


#define USE_STATIC_MESHES_DEBUG_COLLISION
//#define USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK


#ifdef _MSC_VER
	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

	#undef APIENTRY
	#define GLFW_EXPOSE_NATIVE_WIN32
	#define GLFW_EXPOSE_NATIVE_WGL

	#include <windows.h>
	#include <Commctrl.h>
	#include <stdio.h>
	#include <stdlib.h>
	#include <malloc.h>
	#include <memory.h>
	#include <time.h>
	#include <tchar.h>
	#include <crtdbg.h>

	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <imgui.h>
	#include <glfw3.h>
	#include <glfw3native.h>
#endif
	
#if (defined (_POSIX_VER) || defined (_POSIX_VER_64))
	#include <stdlib.h>
	#include <unistd.h>
	#include <time.h>

	#include <glatter.h>
	#include <GL/glu.h>
	#include <GL/gl.h>
	#include <imgui.h>
	#include <GLFW/glfw3.h>

	// audio library support
	#include <AL/al.h>
	#include <AL/alc.h>
#endif

#ifdef _MACOSX_VER
	#include <CoreFoundation/CoreFoundation.h> 
	#include <unistd.h>
	#include <GL/glew.h>
	#include <OpenGL/glu.h>
	#include <OpenGL/gl.h>

	// audio library support
	#include <OpenAl/al.h>
	#include <OpenAl/alc.h>
#endif



#ifdef _MSC_VER
	#pragma warning (disable: 4100) //unreferenced formal parameter
	#pragma warning (disable: 4505) //unreferenced local function has been removed
	#pragma warning (disable: 4201) //nonstandard extension used : nameless struct/union
	#pragma warning (disable: 4127) //conditional expression is constant
	#pragma warning (disable: 4456) //declaration of 'bone' hides previous local declaration

	#if (_MSC_VER >= 1400)
		#pragma warning (disable: 4996) // for 2005 users declared deprecated
	#endif
#endif

// SDK includes
#include <Newton.h>
#include <dVector.h>
#include <dMatrix.h>
#include <dQuaternion.h>
#include <dMathDefines.h>
#include <dBezierSpline.h>

#include <dCRC.h>
#include <dMap.h>
#include <dHeap.h>
#include <dList.h>
#include <dTree.h>
#include <dRtti.h>
#include <dArray.h>
#include <dPointer.h>
#include <dClassInfo.h>
#include <dRefCounter.h>
#include <dBaseHierarchy.h>

#include <dCustomJoint.h>
#include <dCustomGear.h>
#include <dCustomHinge.h>
#include <dCustomPlane.h>
#include <dCustomWheel.h>
#include <dCustomMotor.h>
#include <dCustomSixdof.h>
#include <dCustomSlider.h>
#include <dCustomPulley.h>
#include <dCustomCorkScrew.h>
#include <dCustomPathFollow.h>
#include <dCustomDoubleHinge.h>
#include <dCustomFixDistance.h>
#include <dCustomTireSpringDG.h>
#include <dCustomRackAndPinion.h>
#include <dCustomHingeActuator.h>
#include <dCustomBallAndSocket.h>
#include <dCustomSliderActuator.h>
#include <dCustomSlidingContact.h>
#include <dCustomInverseDynamics.h>
#include <dCustomDoubleHingeActuator.h>

#include <dCustomInputManager.h>
#include <dCustomTriggerManager.h>
#include <dCustomTransformManager.h>
#include <dCustomControllerManager.h>
#include <dCustomKinematicController.h>
#include <dCustomPlayerControllerManager.h>
#include <dCustomPlayerControllerManager.h>
#include <dCustomVehicleControllerManager.h>

#include <dModelStdAfx.h>
#include <dModelManager.h>
#include <dModelAnimTree.h>
#include <dModelAnimTreePose.h>
#include <dModelAnimTreePoseBlender.h>


#include <dStdafxVehicle.h>
#include <dVehicleChassis.h>
#include <dVehicleManager.h>
#include <dVehicleInterface.h>
#include <dVehicleDashControl.h>

#include <dSceneStdafx.h>
#include <dScene.h>
#include <dRootNodeInfo.h>
#include <dBoneNodeInfo.h>
#include <dMeshNodeInfo.h>
#include <dLineNodeInfo.h>
#include <dSceneNodeInfo.h>
#include <dAnimationTake.h>
#include <dAnimationTrack.h>
#include <dTextureNodeInfo.h>
#include <dMaterialNodeInfo.h>
#include <dRigidbodyNodeInfo.h>
#include <dCollisionNodeInfo.h>
#include <dCollisionBoxNodeInfo.h>
#include <dCollisionSphereNodeInfo.h>
#include <dCollisionConvexHullNodeInfo.h>
#include <dGeometryNodeSkinClusterInfo.h>

#include <dStdAfxNewton.h>
#include <dNewtonMesh.h>

#include <dAnimationStdAfx.h>
#include <dAnimationJoint.h>
#include <dAnimationLoopJoint.h>
#include <dAnimationRagdollRoot.h>
#include <dAnimationModelManager.h>
#include <dAnimationRagdollJoint.h>
#include <dAnimationRagDollEffector.h>

#include <dAnimationKeyframesSequence.h>

/*
#include <dAnimIKManager.h>
#include <dAnimIK3dofJoint.h>
#include <dAnimIKBlendNodePose.h>
#include <dAnimIKBlendNodeRoot.h>
#include <dAnimIKBlendNodeTake.h>
#include <dAnimIDManager.h>
#include <dAnimIDRigLimb.h>
#include <dAnimIDRigJoint.h>
#include <dAnimIDRigHinge.h>
#include <dAnimIDRigEffector.h>
#include <dAnimIDBlendNode.h>
#include <dAnimIDBlendNodePose.h>
#include <dAnimIDBlendNodeRoot.h>
#include <dAnimIDBlendNodeTwoWay.h>
#include <dAnimIDRigForwardDynamicLimb.h>
*/

/*
#ifdef _NEWTON_USE_DOUBLE
	#define glMultMatrix(x) glMultMatrixd(x)
	#define glLoadMatrix(x) glMultMatrixd(x)
	#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
#else
	#define glMultMatrix(x) glMultMatrixf(x)
	#define glLoadMatrix(x) glMultMatrixf(x)
	#define glGetFloat(x,y) glGetFloatv(x,(dFloat  *)y) 
#endif
*/

#ifdef _NEWTON_USE_DOUBLE
inline void glMaterialParam (GLenum face, GLenum pname, const dFloat *params)
{
	GLfloat tmp[4] = {GLfloat(params[0]), GLfloat(params[1]), GLfloat(params[2]), GLfloat(params[3])};
	glMaterialfv (face, pname, &tmp[0]);
}
#define glMultMatrix(x) glMultMatrixd(x)
#define glLoadMatrix(x) glMultMatrixd(x)
#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
#else 
#define glMaterialParam glMaterialfv
#define glMultMatrix(x) glMultMatrixf(x)
#define glLoadMatrix(x) glMultMatrixf(x)
#define glGetFloat(x,y) glGetFloatv(x, (GLfloat*)y) 
#endif


#ifndef _MSC_VER
	#ifndef stricmp
		#define stricmp strcasecmp
	#endif

	#ifndef strlwr
		inline char *_strlwr_ (char *a) 
		{ 
			char *ret = a; 
			while (*a != '\0') 
			{ 
				//if (isupper (*a)) 
				*a = char (tolower (*a)); 
				++a; 
			} 
			return ret; 
		}

		#define strlwr(a) _strlwr_ (a) 
	#endif
#endif


//#define DEMO_CHECK_ASYN_UPDATE
#define dRAND_MAX		0x0fffffff

unsigned dRand ();
void dSetRandSeed (unsigned seed);
dFloat dGaussianRandom (dFloat amp);

inline int dTwosPower (int x)
{
	int rval=1;
	for (; rval < x; rval *= 2);
	return rval;
}


// for some reason specifying a relative does not seem to work in Linus
// and i have to specify a absolute path
// #define ASSETS_PATH "."
//void GetAplicationDirectory (char* const aplicationDir);
void dGetWorkingFileName (const char* const name, char* const outPathName);


// little Indian/big Indian conversion
unsigned SWAP_INT32(unsigned x);
unsigned short SWAP_INT16(unsigned short x);
void SWAP_FLOAT32_ARRAY (void* const array, dInt32 count);

#endif 

