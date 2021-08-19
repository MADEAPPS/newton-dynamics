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

#ifndef __D_SAVE_LOAD_SYSTEM_H__
#define __D_SAVE_LOAD_SYSTEM_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dTree.h"

class ndShape;
class ndBodyKinematic;

class ndShapeLoaderCache : public dTree<const ndShape*, dUnsigned32>
{
};

class ndBodyLoaderCache : public dTree<const ndBodyKinematic*, dUnsigned32>
{
};

class dClassLoaderBase
{
	public:
	virtual void* CreateClass(const nd::TiXmlNode* const, const char* const)
	{
		dAssert(0);
		return nullptr;
	}
};

D_CORE_API void RegisterLoaderClass(const char* const className, dClassLoaderBase* const loaderClass);
D_CORE_API void* LoadShapeClass(const char* const className, const nd::TiXmlNode* const xmlNode, const char* const assetPath);
D_CORE_API void* LoadBodyClass(const char* const className, const nd::TiXmlNode* const xmlNode, const char* const assetPath, const ndShapeLoaderCache& shapeMap);

template<class T>
class dClassLoader: public dClassLoaderBase
{
	public:
	dClassLoader<T>(const char* const className)
	{
		RegisterLoaderClass(className, this);
	}

	virtual void* CreateClass(const nd::TiXmlNode* const xmlNode, const char* const assetPath)
	{
		return new T(xmlNode, assetPath);
	}
};

#define D_CLASS_REFLECTION(Class)								\
	static const char* ClassName() {return #Class;}				\
	D_CORE_API static dClassLoader<Class> __classLoader__;

#define D_LOAD_SHAPE(castType,className,node,assetPath) \
	(castType*)LoadShapeClass(className, node, assetPath);

#define D_CLASS_REFLECTION_IMPLEMENT_SHAPE_LOADER(Class) \
	D_CORE_API dClassLoader<Class> Class::__classLoader__(#Class);

#define D_LOAD_BODY(castType,className,node,assetPath,shapeMap) \
	(castType*)LoadBodyClass(className, node, assetPath, shapeMap);

#define D_CLASS_REFLECTION_IMPLEMENT_BODY_LOADER(Class) \
	D_CORE_API dClassLoader<Class> Class::__classLoader__(#Class);

#endif

