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

#ifndef __ND_TINYXML_GLUE_H__
#define __ND_TINYXML_GLUE_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndVector.h"
#include "ndMatrix.h"

D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const type, const char* const value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt64 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndFloat32 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndVector& value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const ndMatrix& value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, ndInt32 count, const ndVector* const array);

D_CORE_API ndInt32 xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API ndInt64 xmlGetInt64(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API ndFloat32 xmlGetFloat(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API ndVector xmlGetVector3(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API ndMatrix xmlGetMatrix(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API const char* xmlGetString(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, ndArray<ndVector>& array);

#endif

