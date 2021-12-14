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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndCRC.h"
#include "ndClassAlloc.h"
#include "ndFixSizeArray.h"	
#include "ndSaveLoadSytem.h"

#define D_LOADER_DICTIONARY_SIZE 128

class ndLoaderFactory
{
	public:
	ndUnsigned64 m_classNameHash;
	ndLoadSaveBase* m_loader;
};

class ndLoaderClassArray: public ndFixSizeArray<ndLoaderFactory, D_LOADER_DICTIONARY_SIZE>
{
	public:
	ndLoaderClassArray()
		:ndFixSizeArray<ndLoaderFactory, D_LOADER_DICTIONARY_SIZE>()
	{
		nd::__free__ = Free;
		nd::__alloc__ = Malloc;
	}

	static void* Malloc(size_t size)
	{
		return ndMemory::Malloc(size);
	}

	static void Free(void* ptr)
	{
		return ndMemory::Free(ptr);
	}
};

static ndFixSizeArray<ndLoaderFactory, D_LOADER_DICTIONARY_SIZE>& GetFactory()
{
	static ndLoaderClassArray factory;
	return factory;
}

void RegisterLoaderClass(const char* const className, ndLoadSaveBase* const loaderClass)
{
	ndLoaderFactory entry;
	entry.m_classNameHash = dCRC64(className);
	entry.m_loader = loaderClass;
	ndFixSizeArray<ndLoaderFactory, D_LOADER_DICTIONARY_SIZE>& factory = GetFactory();
	factory.PushBack(entry);
	for (ndInt32 i = factory.GetCount() - 2; i >= 0; i--)
	{
		if (entry.m_classNameHash < factory[i].m_classNameHash)
		{
			factory[i + 1] = factory[i];
			factory[i] = entry;
		}
		else
		{
			break;
		}
	}
}

void* LoadClass(const char* const className, const ndLoadSaveBase::ndLoadDescriptor& descriptor)
{
	ndUnsigned64 classNameHash = dCRC64(className);

	const ndFixSizeArray<ndLoaderFactory, D_LOADER_DICTIONARY_SIZE>& factory = GetFactory();

	ndInt32 i0 = 0; 
	ndInt32 i1 = factory.GetCount() - 1;
	while ((i1 - i0 > 4))
	{
		ndInt32 mid = (i1 + i0) / 2;
		if (factory[mid].m_classNameHash <= classNameHash)
		{
			i0 = mid;
		}
		else
		{
			i1 = mid;
		}
	}

	for (ndInt32 i = i0; i <= i1; i++)
	{
		if (factory[i].m_classNameHash == classNameHash)
		{
			return factory[i].m_loader->CreateClass(descriptor);
		}
	}
	
	#ifdef _DEBUG
	for (ndInt32 i = 0; i < factory.GetCount(); i++)
	{
		if (factory[i].m_classNameHash == classNameHash)
		{
			dAssert (0);
		}
	}
	#endif
	
	ndLoadSaveBase::ndLoadDescriptor baseClassDesc(descriptor);
	for (const nd::TiXmlNode* node = descriptor.m_rootNode->FirstChild(); node; node = node->NextSibling())
	{
		const char* const name = node->Value();
		baseClassDesc.m_rootNode = node;
		void* object = LoadClass(name, baseClassDesc);
		if (object)
		{
			return object;
		}
	}

	//dAssert(0);
	return nullptr;
}

