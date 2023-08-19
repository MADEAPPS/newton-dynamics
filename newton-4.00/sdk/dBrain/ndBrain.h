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

#ifndef _ND_BRAIN_H__
#define _ND_BRAIN_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"

class ndBrainLoad;
class ndBrainSave;

class ndHidenVariableOffsets: public ndFixSizeArray<ndInt32, 32>
{
	public:
	ndHidenVariableOffsets()
		:ndFixSizeArray<ndInt32, 32>()
	{
		Clear();
	}

	ndHidenVariableOffsets(const ndHidenVariableOffsets& src)
		:ndFixSizeArray<ndInt32, 32>()
	{
		Clear();
		for (ndInt32 i = 0; i < src.GetCount(); ++i)
		{
			ndFixSizeArray<ndInt32, 32>::PushBack(src[i]);
		}
	}

	void Clear()
	{
		ndFixSizeArray<ndInt32, 32>::SetCount(0);
		for (ndInt32 i = 0; i < 32; ++i)
		{
			ndFixSizeArray<ndInt32, 32>::PushBack(0);
		}
		ndFixSizeArray<ndInt32, 32>::SetCount(0);
	}
};

class ndBrain: public ndArray<ndBrainLayer*>
{
	public: 
	ndBrain();
	ndBrain(const ndBrain& src);
	~ndBrain();

	ndInt32 GetInputSize() const;
	ndInt32 GetOutputSize() const;
	void CopyFrom(const ndBrain& src);
	void SoftCopy(const ndBrain& src, ndReal blend);

	void BeginAddLayer();
	void EndAddLayer();
	ndBrainLayer* AddLayer(ndBrainLayer* const layer);

	bool Compare(const ndBrain& src) const;

	void InitWeightsXavierMethod();
	void InitGaussianBias(ndReal variance);
	void InitGaussianWeights(ndReal variance);

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output);
	void CalculateInputGradients(const ndBrainVector& input, ndBrainVector& inputGradients);
	void CalculateInputGradientLoss(const ndBrainVector& input, const ndBrainVector& groundTruth, ndBrainVector& inputGradients);

	private:
	void CalculateOffsets();
	void MakePrediction(const ndBrainVector& input, ndBrainVector& output, const ndBrainVector& hiddenLayerOutputs);

	ndReal* m_memory;
	ndInt32 m_memorySize;
	ndHidenVariableOffsets m_offsets;
	bool m_isReady;

	friend class ndBrainLoad;
	friend class ndBrainSave;
	friend class ndBrainTrainer;
};

#endif 

