// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//


#ifndef _D_PLUGIN_COMMONN_STDAFX_H_
#define _D_PLUGIN_COMMONN_STDAFX_H_



#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers


#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <ctype.h>
#include <io.h>
#include <windows.h>
#include <GL/glew.h>
#include <GL/wglew.h>
#include <gl/gl.h>
#include <gl/glu.h>


#include "dContainersStdAfx.h"
#include <dSceneStdafx.h>

#include <dCRC.h>
#include <dHeap.h>
#include <dList.h>
#include <dTree.h>
#include <dRtti.h>
#include <dClassInfo.h>
#include <dRefCounter.h>
#include <dBaseHierarchy.h>


#include <dScene.h>
#include <dUndoRedo.h>
#include <dSceneGraph.h>
#include <dSceneRender.h>
#include <dBoneNodeInfo.h>
#include <dRootNodeInfo.h>
#include <dMeshNodeInfo.h>
#include <dSceneNodeInfo.h>
#include <dSceneModelInfo.h>
#include <dTextureNodeInfo.h>
#include <dMaterialNodeInfo.h>
#include <dRigidbodyNodeInfo.h>
#include <dCollisionNodeInfo.h>
#include <dCollisionBoxNodeInfo.h>
#include <dCollisionSphereNodeInfo.h>
#include <dCollisionConvexHullNodeInfo.h>
#include <dGeometryNodeSkinModifierInfo.h>

#endif