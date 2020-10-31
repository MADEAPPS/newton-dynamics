/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_TINYXML_GLUE_H__
#define __D_TINYXML_GLUE_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dArray.h"
#include "dVector.h"
#include "dMatrix.h"

D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const char* const type, const char* const value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt32 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt64 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dFloat32 value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const dVector& value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, const dMatrix& value);
D_CORE_API void xmlSaveParam(nd::TiXmlElement* const rootNode, const char* const name, dInt32 count, const dVector* const array);

D_CORE_API dInt32 xmlGetInt(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API dFloat32 xmlGetFloat(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API dVector xmlGetVector3(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API dMatrix xmlGetMatrix(const nd::TiXmlNode* const rootNode, const char* const name);
D_CORE_API void xmlGetFloatArray3(const nd::TiXmlNode* const rootNode, const char* const name, dArray<dVector>& array);

#endif

