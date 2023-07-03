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

#include "ndBrainStdafx.h"
#include "ndBrain.h"

ndBrain::ndBrain()
	:ndArray<ndBrainLayer*>()
	,m_memory(nullptr)
	,m_memorySize(0)
	,m_isReady(false)
{
}

ndBrain::ndBrain(const ndBrain& src)
	:ndArray<ndBrainLayer*>()
	,m_memory(nullptr)
	,m_memorySize(0)
	,m_isReady(src.m_isReady)
{
	const ndArray<ndBrainLayer*>& srcLayers = src;
	BeginAddLayer();
	for (ndInt32 i = 0; i < srcLayers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = srcLayers[i]->Clone();
		AddLayer(layer);
	}
	EndAddLayer(ndReal(0.0f));
	CopyFrom(src);
}

ndBrain::~ndBrain()
{
	if (m_memory)
	{
		for (ndInt32 i = 0; i < GetCount(); ++i)
		{
			ndBrainLayer& layer = *(*this)[i];
			layer.SetFloatPointers(nullptr);
			layer.SetPointers(nullptr);
		}
		ndMemory::Free(m_memory);
	}

	for (ndInt32 i = GetCount() - 1; i >= 0 ; --i)
	{
		delete (*this)[i];
	}
}

ndInt32 ndBrain::GetInputSize() const
{
	return GetCount() ? (*this)[0]->GetInputSize() : 0;
}

ndInt32 ndBrain::GetOutputSize() const
{
	return GetCount() ? (*this)[GetCount()-1]->GetOuputSize() : 0;
}

void ndBrain::CopyFrom(const ndBrain& src)
{
	const ndArray<ndBrainLayer*>& layers = *this;
	const ndArray<ndBrainLayer*>& srcLayers = src;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		layers[i]->CopyFrom(*srcLayers[i]);
	}
}

ndBrainLayer* ndBrain::AddLayer(ndBrainLayer* const layer)
{
	ndAssert(!GetCount() || ((*this)[GetCount() - 1]->GetOuputSize() == layer->GetInputSize()));
	PushBack(layer);
	return layer;
}

void ndBrain::BeginAddLayer()
{
	m_isReady = false;
}

void ndBrain::EndAddLayer(ndReal randomVariance)
{
	ndInt32 floatsCount = 0;
	ndInt32 vectorSizeInBytes = 0;
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBrainLayer& layer = *(*this)[i];
		ndInt32 columnSize = (layer.GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
		ndInt32 rowSize = layer.GetOuputSize() + 1;
		floatsCount += columnSize * rowSize;
		vectorSizeInBytes += layer.GetOuputSize() * sizeof(ndArray<ndDeepBrainMemVector>);
	}

	ndInt32 memorySize = floatsCount * ndInt32(sizeof(ndReal)) + vectorSizeInBytes + 256;
	m_memorySize = memorySize;
	m_memory = ndMemory::Malloc(size_t(memorySize));
	memset(m_memory, 0, size_t(memorySize));

	// assign vector pointers
	ndUnsigned8* mem = (ndUnsigned8*)m_memory;
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBrainLayer& layer = *(*this)[i];
		mem = layer.SetPointers(mem);
	}

	ndIntPtr metToVal;
	metToVal.m_ptr = mem;
	//ndReal* floatMemory = (ndReal*) ((ndUnsigned64 (mem) + 31) & -32);
	ndReal* floatMemory = (ndReal*)((ndUnsigned64(metToVal.m_int) + 31) & -32);
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBrainLayer& layer = *(*this)[i];
		floatMemory = layer.SetFloatPointers(floatMemory);
	}

	InitGaussianWeights(randomVariance);
	m_isReady = true;
}

bool ndBrain::Compare(const ndBrain& src) const
{
	if (m_isReady != src.m_isReady)
	{
		ndAssert(0);
		return false;
	}

	if (GetCount() != src.GetCount())
	{
		ndAssert(0);
		return false;
	}

	const ndArray<ndBrainLayer*>& layers0 = *this;
	const ndArray<ndBrainLayer*>& layers1 = src;
	for (ndInt32 i = 0; i < layers0.GetCount(); ++i)
	{
		bool test = layers0[i]->Compare(*layers1[i]);
		if (!test)
		{
			ndAssert(0);
			return false;
		}
	}

	return true;
}

void ndBrain::InitGaussianWeights(ndReal variance)
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitGaussianWeights(variance);
	}
}

#if 0
void ndBrain::Save(const char* const pathName) const
{
	nd::TiXmlDocument asciifile;
	nd::TiXmlDeclaration* const decl = new nd::TiXmlDeclaration("1.0", "", "");
	asciifile.LinkEndChild(decl);

	nd::TiXmlElement* const rootNode = new nd::TiXmlElement("ndBrain");
	asciifile.LinkEndChild(rootNode);

	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBrainLayer* const layer = (*this)[i];
		nd::TiXmlElement* const layerNode = new nd::TiXmlElement("ndLayer");
		rootNode->LinkEndChild(layerNode);
		layer->Save (layerNode);
	}

	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");
	asciifile.SaveFile(pathName);
	setlocale(LC_ALL, oldloc);
}

bool ndBrain::Load(const char* const pathName)
{
	ndAssert(0);

	char* const oldloc = setlocale(LC_ALL, 0);
	setlocale(LC_ALL, "C");

	nd::TiXmlDocument doc(pathName);
	doc.LoadFile();
	if (doc.Error())
	{
		setlocale(LC_ALL, oldloc);
		return false;
	}
	ndAssert(!doc.Error());

	const nd::TiXmlElement* const rootNode = doc.RootElement();
	if (!rootNode)
	{
		return false;
	}

	BeginAddLayer();
	for (const nd::TiXmlNode* layerNode = rootNode->FirstChild("ndLayer"); layerNode; layerNode = layerNode->NextSibling())
	{
		const char* const layerType = xmlGetString(layerNode, "type");
		if (layerType)
		{
			ndBrainLayer* layer = nullptr;
			if (!strcmp(layerType, "fullyConnected"))
			{
				layer = new ndBrainLayer(layerNode);
			}
			else
			{
				ndAssert(0);
			}
			if (layer)
			{
				AddLayer(layer);
			}
		}
	}
	EndAddLayer();

	ndInt32 index = 0;
	for (const nd::TiXmlNode* layerNode = rootNode->FirstChild("ndLayer"); layerNode; layerNode = layerNode->NextSibling())
	{
		ndBrainLayer* const layer = (*this)[index];
		layer->Load((nd::TiXmlElement*)layerNode);
		index++;
	}

	m_isReady = true;
	return true;
}
#endif

ndBrainLoadSave::ndBrainLoadSave()
	:ndClassAlloc() 
{
}

ndBrainLoadSave::~ndBrainLoadSave() 
{
}


//ndBrain* Load();
void ndBrainLoadSave::Save(const ndBrain* const brain)
{
	char buffer[1024];
	auto Save = [this, &buffer](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsprintf(buffer, fmt, v_args);
		va_end(v_args);
		SaveData(buffer);
	};

	Save("ndBrain version 1.0\n\n");
	Save("layersCount %d\n\n", brain->GetCount());
	
	for (ndInt32 i = 0; i < brain->GetCount(); ++i)
	{
		ndBrainLayer* const layer = (*brain)[i];
		Save("layer\n");
		Save("{\n");
		layer->Save(this);
		Save("}\n\n");
	}
}
