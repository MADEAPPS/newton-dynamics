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

#ifndef __ND_TINYXML_GLUE_H__
#define __ND_TINYXML_GLUE_H__

#include "ndArray.h"
#include "ndVector.h"
#include "ndMatrix.h"

#include "tinyxml.h"

D_TINY_API void xmlResetClassId();
D_TINY_API nd::TiXmlElement* xmlCreateClassNode(nd::TiXmlElement* const parent, const char* const className, const char* const name);

D_TINY_API void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 value);
D_TINY_API void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, ndReal value);
D_TINY_API void xmlSaveAtribute(nd::TiXmlElement* const rootNode, const char* const name, const char* const value);

D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt64 value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndFloat32 value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndVector& value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndMatrix& value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const value);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndReal>& array);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt32>& array);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndInt64>& array);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndVector>& array);
D_TINY_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndArray<ndTriplexReal>& array);

D_TINY_API ndInt32 xmlGetNodeId(const nd::TiXmlNode* const rootNode);
D_TINY_API ndInt32 xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name);
D_TINY_API ndInt64 xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name);
D_TINY_API ndMatrix xmlGetMatrix(const nd::TiXmlNode* const rootNode, const char* const name);
D_TINY_API ndFloat32 xmlGetFloat(const nd::TiXmlNode* const rootNode, const char* const name);
D_TINY_API ndVector xmlGetVector3(const nd::TiXmlNode* const rootNode, const char* const name);
D_TINY_API const char* xmlGetString(const nd::TiXmlNode* const rootNode, const char* const name);

D_TINY_API void xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndInt32>& array);
D_TINY_API void xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndInt64>& array);
//D_TINY_API void xmlGetRealArray(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndReal>& array);
D_TINY_API void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndVector>& array);
D_TINY_API void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndTriplexReal>& array);
D_TINY_API void xmlGetFloat64Array3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndBigVector>& array);

#endif

