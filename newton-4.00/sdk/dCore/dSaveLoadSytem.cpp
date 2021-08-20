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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dCRC.h"
#include "dClassAlloc.h"
#include "dFixSizeArray.h"	
#include "dSaveLoadSytem.h"

class dLoaderFactory
{
	public:
	dLoadSaveBase* m_loader;
	dUnsigned64 m_classNameHash;
};

static dFixSizeArray<dLoaderFactory, 128>& GetFactory()
{
	static dFixSizeArray<dLoaderFactory, 128> factory;
	return factory;
}

void RegisterLoaderClass(const char* const className, dLoadSaveBase* const loaderClass)
{
	dLoaderFactory entry;
	entry.m_classNameHash = dCRC64(className);
	entry.m_loader = loaderClass;
	dFixSizeArray<dLoaderFactory, 128>& factory = GetFactory();
	factory.PushBack(entry);
}

void* LoadClass(const char* const className, const dLoadSaveBase::dDesc& descriptor)
{
	dUnsigned64 classNameHash = dCRC64(className);

	const dFixSizeArray<dLoaderFactory, 128>& factory = GetFactory();
	for (dInt32 i = 0; i < factory.GetCount(); i++)
	{
		if (factory[i].m_classNameHash == classNameHash)
		{
			return factory[i].m_loader->CreateClass(descriptor);
		}
	}
	
	dLoadSaveBase::dDesc baseClassDesc(descriptor);
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

