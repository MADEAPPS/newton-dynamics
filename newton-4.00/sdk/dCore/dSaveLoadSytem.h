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
class ndShapeInstance;

class ndShapeLoaderCache : public dTree<const ndShapeInstance, dUnsigned32>
{
};

class ndBodyLoaderCache: public dTree<const ndBody*, dUnsigned32>
{
};

class dLoadSaveBase
{
	public:
	class dLoadDescriptor
	{
		public:
		dLoadDescriptor()
			:m_assetPath(nullptr)
			,m_rootNode(nullptr)
			,m_bodyMap(nullptr)
			,m_shapeMap(nullptr)
		{
		}

		dLoadDescriptor(const dLoadDescriptor& desc)
			:m_assetPath(desc.m_assetPath)
			,m_rootNode(desc.m_rootNode->FirstChild())
			,m_bodyMap(desc.m_bodyMap)
			,m_shapeMap(desc.m_shapeMap)
		{
		}

		const char* m_assetPath;
		const nd::TiXmlNode* m_rootNode;
		const ndBodyLoaderCache* m_bodyMap;
		const ndShapeLoaderCache* m_shapeMap;
	};

	class dSaveDescriptor
	{
		public:
		dSaveDescriptor()
			:m_assetPath(nullptr)
			,m_assetName(nullptr)
			,m_rootNode(nullptr)
			,m_assetIndex(0)
			,m_nodeNodeHash(0)
			,m_shapeNodeHash(0)
			,m_body0NodeHash(0)
			,m_body1NodeHash(0)
			,m_shapeMap(nullptr)
		{
		}

		dSaveDescriptor(const dSaveDescriptor& desc, nd::TiXmlNode* const rootNode)
			:m_assetPath(desc.m_assetPath)
			,m_assetName(desc.m_assetName)
			,m_rootNode(rootNode)
			,m_assetIndex(desc.m_assetIndex)
			,m_nodeNodeHash(desc.m_nodeNodeHash)
			,m_shapeNodeHash(desc.m_shapeNodeHash)
			,m_body0NodeHash(desc.m_body0NodeHash)
			,m_body1NodeHash(desc.m_body1NodeHash)
			,m_shapeMap(desc.m_shapeMap)
		{
		}

		const char* m_assetPath;
		const char* m_assetName;
		nd::TiXmlNode* m_rootNode;
		mutable dInt32 m_assetIndex;
		dInt32 m_nodeNodeHash;
		dInt32 m_shapeNodeHash;
		dInt32 m_body0NodeHash;
		dInt32 m_body1NodeHash;
		dTree<dUnsigned32, const ndShape*>* m_shapeMap;
	};

	virtual void* CreateClass(const dLoadDescriptor&)
	{
		dAssert(0);
		return nullptr;
	}
};

D_CORE_API void* LoadClass(const char* const className, const dLoadSaveBase::dLoadDescriptor& desc);
D_CORE_API void RegisterLoaderClass(const char* const className, dLoadSaveBase* const loaderClass);

template<class T>
class dLoadSaveClass: public dLoadSaveBase
{
	public:
	dLoadSaveClass<T>(const char* const className)
	{
		RegisterLoaderClass(className, this);
	}

	virtual void* CreateClass(const dLoadDescriptor& desc)
	{
		return new T(desc);
	}
};

#define D_CLASS_REFLECTION(Class)								\
	static const char* ClassName() {return #Class;}				\
	static dLoadSaveClass<Class> __classLoader__;

#define D_CLASS_REFLECTION_IMPLEMENT_LOADER(Class) \
	dLoadSaveClass<Class> Class::__classLoader__(#Class);

#define D_CLASS_REFLECTION_LOAD_NODE(castType,className,desc) \
	(castType*)LoadClass(className, desc);

#endif

