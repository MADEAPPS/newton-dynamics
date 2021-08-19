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

class ndBody;
class ndShape;

class ndShapeLoaderCache : public dTree<const ndShape*, dUnsigned32>
{
};

class ndBodyLoaderCache : public dTree<const ndBody*, dUnsigned32>
{
};

class dClassLoaderBase
{
	public:
	class dDesc
	{
		public:
		dDesc()
			:m_assetPath(nullptr)
			,m_rootNode(nullptr)
			,m_shapeMap(nullptr)
			,m_bodyMap(nullptr)
		{
		}

		dDesc(const dDesc& desc)
			:m_assetPath(desc.m_assetPath)
			,m_rootNode(desc.m_rootNode->FirstChild())
			,m_shapeMap(desc.m_shapeMap)
			,m_bodyMap(desc.m_bodyMap)
		{
		}


		const char* m_assetPath;
		const nd::TiXmlNode* m_rootNode;
		const ndShapeLoaderCache* m_shapeMap;
		const ndBodyLoaderCache* m_bodyMap;
	};

	virtual void* CreateClass(const dDesc&)
	{
		dAssert(0);
		return nullptr;
	}
};

D_CORE_API void* LoadClass(const char* const className, const dClassLoaderBase::dDesc& desc);
D_CORE_API void RegisterLoaderClass(const char* const className, dClassLoaderBase* const loaderClass);

template<class T>
class dClassLoader: public dClassLoaderBase
{
	public:
	dClassLoader<T>(const char* const className)
	{
		RegisterLoaderClass(className, this);
	}

	virtual void* CreateClass(const dDesc& desc)
	{
		return new T(desc);
	}
};

#define D_CLASS_REFLECTION(Class)								\
	static const char* ClassName() {return #Class;}				\
	D_CORE_API static dClassLoader<Class> __classLoader__;

#define D_CLASS_REFLECTION_IMPLEMENT_LOADER(Class) \
	D_CORE_API dClassLoader<Class> Class::__classLoader__(#Class);

#define D_CLASS_REFLECTION_LOAD_NODE(castType,className,desc) \
	(castType*)LoadClass(className, desc);

#endif

