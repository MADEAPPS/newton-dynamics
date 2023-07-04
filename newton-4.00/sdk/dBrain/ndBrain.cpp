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
#include "ndBrainTypes.h"

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

ndBrain* ndBrainLoad::Load(const char* const pathName)
{
	class Loader : public ndBrainLoad
	{
		public:
		Loader(const char* const pathName)
			:ndBrainLoad()
		{
			m_file = fopen(pathName, "rb");
			ndAssert(m_file);
		}

		~Loader()
		{
			if (m_file)
			{
				fclose(m_file);
			}
		}

		ndInt32 ReadInt() const
		{
			ndInt32 value;
			fscanf(m_file, "%d", &value);
			return value;
		}

		ndFloat32 ReadFloat() const
		{
			ndFloat32 value;
			fscanf(m_file, "%f", &value);
			return value;
		}

		void ReadString(char* const buffer) const
		{
			fscanf(m_file, "%s", buffer);
		}

		FILE* m_file;
	};

	Loader loader(pathName);
	return loader.Load();
}

ndBrain* ndBrainLoad::Load() const
{
	ndBrain* const brain = new ndBrain;

	char buffer[1024];

	ReadString(buffer);
	ReadString(buffer);
	ReadString(buffer);
	
	ReadString(buffer);
	ndInt32 layersCount = ReadInt();

	brain->BeginAddLayer();
	for (ndInt32 i = 0; i < layersCount; ++i)
	{
		char layerType[256];
		ReadString(buffer);
		ReadString(layerType);

		ReadString(buffer);
		ndBrainLayer* layer = nullptr;

		ReadString(buffer);
		ReadString(buffer);

		ndBrainActivationType activation = m_sigmoid;
		if (!strcmp(buffer, "relu"))
		{
			activation = m_relu;
		}
		else if (!strcmp(buffer, "lineal"))
		{
			activation = m_lineal;
		}
		else if (!strcmp(buffer, "tanh"))
		{
			activation = m_tanh;
		}
		else if (!strcmp(buffer, "softmax"))
		{
			activation = m_softmax;
		}
		else
		{
			activation = m_sigmoid;
		}

		ReadString(buffer);
		ndInt32 inputs = ReadInt();
		ReadString(buffer);
		ndInt32 outputs = ReadInt();

		if (!strcmp(layerType, "fullyConnected"))
		{
			//layer = new ndBrainLayer(this);
			layer = new ndBrainLayer(inputs, outputs, activation);
		}
		else
		{
			ndAssert(0);
		}
		if (layer)
		{
			//layer->Load(this);
			brain->AddLayer(layer);
		}

		ReadString(buffer);
	}
	brain->EndAddLayer(ndReal (0.0f));

	for (ndInt32 i = 0; i < layersCount; ++i)
	{
		ndBrainLayer* const layer = (*brain)[i];
		ReadString(buffer);
		ReadString(buffer);
		layer->Load(this);
		ReadString(buffer);
	}
	
	brain->m_isReady = true;

	return brain;
}

void ndBrainSave::Save(const ndBrain* const brain)
{
	char buffer[1024];
	auto Save = [this, &buffer](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsprintf(buffer, fmt, v_args);
		va_end(v_args);
		WriteData(buffer);
	};
	
	Save("ndBrain version 1.0\n\n");
	Save("layersCount %d\n\n", brain->GetCount());

	for (ndInt32 i = 0; i < brain->GetCount(); ++i)
	{
		ndBrainLayer* const layer = (*brain)[i];
		Save("layer fullyConnected\n");
		Save("{\n");
		switch (layer->m_activation)
		{
			case m_relu:
				Save("\tactivation relu\n");
				break;

			case m_lineal:
				Save("\tactivation lineal\n");
				break;

			case m_tanh:
				Save("\tactivation tanh\n");
				break;

			case m_softmax:
				Save("\tactivation softmax\n");
				break;

			case m_sigmoid:
			default:
				Save("\tactivation sigmoid\n");
				break;
		}

		Save("\tinputs %d\n", layer->GetColumns());
		Save("\toutputs %d\n", layer->GetRows());
		Save("}\n\n");
	}

	for (ndInt32 i = 0; i < brain->GetCount(); ++i)
	{
		ndBrainLayer* const layer = (*brain)[i];
		Save("layer\n");
		Save("{\n");
		layer->Save(this);
		Save("}\n\n");
	}
}

void ndBrainSave::Save(const ndBrain* const brain, const char* const pathName)
{
	class SaveAgent : public ndBrainSave
	{
		public:
		SaveAgent(const char* const pathFilename)
			:ndBrainSave()
		{
			m_file = fopen(pathFilename, "wb");
			ndAssert(m_file);
		}

		~SaveAgent()
		{
			if (m_file)
			{
				fclose(m_file);
			}
		}

		void WriteData(const char* data) const
		{
			fprintf(m_file, data);
		}

		FILE* m_file;
	};

	SaveAgent saveAgent(pathName);
	saveAgent.Save(brain);
}