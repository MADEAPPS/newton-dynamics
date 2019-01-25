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


#ifndef __NEWTON_MAX_COMMON_H__
#define __NEWTON_MAX_COMMON_H__

#include <windows.h>
#include <crtdbg.h> 
#pragma warning (disable: 4996) // for 2005 users declared deprecated


#ifndef nullptr
#define nullptr NULL
#endif

#include <Max.h>
#include <io.h>
#include <stdio.h>
#include <notify.h> 
#include <iparamb2.h>
#include <iparamm2.h>
#include <istdplug.h>
#include <utilapi.h>
#include <stdMat.h>
#include <mnMath.h>
#include <decomp.h>
#include <iskin.h>
#include <utilapi.h> 
#include <modstack.h>
#include <CS/IMixer8.h>
#include <ILayerControl.h>



#include <Newton.h>

#include <dCRC.h>
#include <dTree.h>
#include <dList.h>
#include <dVector.h>
#include <dMatrix.h>
#include <dQuaternion.h>


#include <dSceneStdAfx.h>
#include <dScene.h>
#include <dBoneNodeInfo.h>
#include <dMeshNodeInfo.h>
#include <dSceneNodeInfo.h>
#include <dTextureNodeInfo.h>
#include <dMaterialNodeInfo.h>
#include <dGeometryNodeSkinClusterInfo.h>

#include <tinyxml.h>

#define D_FILE_EXT			"ngd"
#define D_AUTHOR_NAME		"Julio Jerez: Newton Dynamics"
#define D_SHORT_DESCRIPTION "Newton Game Dynamics format"
#define D_LONG_DESCRIPTION  "Newton Game Dynamics 3.00 universal file format"

#include "resource.h"

#ifndef max
	#define max(a,b) ((a) > (b)) ? (a) : (b))
#endif

#ifndef min
	#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif


extern HINSTANCE hInstance;

TCHAR *GetString(int id);
void GetNodeName (INode* node, char* name);
dMatrix GetMatrixFromMaxMatrix (const Matrix3& pivot);

Matrix3 GetMatrixFromdMatrix (const dMatrix& matrix);

int FindFilePath (const char *name, const char *path,  char *fullPathName);

void GetNameFromPath (const char* pathName, char* name);

#endif