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

#ifndef _ND_BRAIN_LAYER_H__
#define _ND_BRAIN_LAYER_H__

#include "ndBrainStdafx.h"
#include "ndBrainTrainer.h"
#include "ndBrainGpuContext.h"

class ndBrainLoad;
class ndBrainSave;
class ndBrainVector;
class ndBrainMatrix;
class ndBrainGpuBuffer;
class ndBrainGpuCommand;
class ndBrainTrainerCpu;
class ndBrainGpuFloatBuffer;
class ndBrainTrainerGpuCommand;
class ndBrainTrainerCpuInference;
class ndBrainTrainerGpuInference;
class ndBrainLayerFeedForwardCpuCommand;
class ndBrainLayerBackPropagateCpuCommand;

class ndBrainLayer : public ndClassAlloc
{
	public: 
	class ndCommandShareInfo
	{
		public:
			ndCommandShareInfo()
			:m_inputSize(0)
			,m_outputSize(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
			,m_parametersBatchSize(0)
			,m_parametersStartOffset(0)
			,m_tiledStride(0)
			,m_layer(nullptr)
		{
		}

		ndCommandShareInfo(ndBrainLayer* const layer)
			:m_inputSize(0)
			,m_outputSize(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
			,m_parametersBatchSize(0)
			,m_parametersStartOffset(0)
			,m_layer(layer)
		{
		}

		ndInt32 m_inputSize;
		ndInt32 m_outputSize;
		ndInt32 m_inputOutputSize;
		ndInt32 m_inputOutputStartOffset;
		ndInt32 m_parametersBatchSize;
		ndInt32 m_parametersStartOffset;
		ndInt32	m_tiledStride;
		ndBrainLayer* m_layer;
	};

	ndBrainLayer();
	ndBrainLayer(const ndBrainLayer& src);

	virtual ~ndBrainLayer();
	virtual ndBrainLayer* Clone() const;

	virtual bool HasParameters() const;
	virtual const char* GetLabelId() const;
	virtual ndInt32 GetNumberOfParameters() const;

	virtual ndInt32 GetInputSize() const;
	virtual ndInt32 GetOutputSize() const;
	virtual ndInt32 GetOutputBufferSize() const;
	
	virtual void Clear();
	virtual void FlushToZero();
	virtual void Scale(ndBrainFloat scale);
	virtual void Set(const ndBrainLayer& src);
	virtual void Add(const ndBrainLayer& src);
	virtual void Mul(const ndBrainLayer& src);
	virtual void Blend(const ndBrainLayer& src, ndBrainFloat blend);
	virtual void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale);
	virtual void AddReqularizerL1(const ndBrainLayer& weights, ndBrainFloat regularizer);
	virtual void AddReqularizerL2(const ndBrainLayer& weights, ndBrainFloat regularizer);

	virtual void ApplyDropOut(ndFloat32 rate);
	virtual void InitWeights();

	virtual void SetWeights(const ndBrainVector& input);
	virtual void CopyWeights(ndBrainVector& output) const;
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output, 
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	virtual void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon);

	virtual ndCommandShareInfo GetCommandSharedInfo();
	virtual ndBrainLayerFeedForwardCpuCommand* GetLayerCpuFeedForwardCommand();
	virtual ndBrainLayerBackPropagateCpuCommand* GetLayerCpuBackPropagateCommand();

	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const info, ndInt32 miniBatchIndex) const;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const;

	virtual bool HasGpuSupport() const;

	virtual ndBrainTrainerGpuCommand* CreateGpuFeedForwardCommand(ndBrainTrainerGpuInference* const owner,
		const ndBrainLayer::ndCommandShareInfo& info,
		ndBrainGpuContext* const context, ndInt32 miniBatchSize,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
		ndBrainGpuBuffer* const inputOutputData,
		ndBrainGpuBuffer* const weightsAndBias) const;

	virtual ndBrainTrainerGpuCommand* CreateGpuBackPropagateCommand(ndBrainTrainerGpuInference* const owner,
		const ndBrainLayer::ndCommandShareInfo& info,
		ndBrainGpuContext* const context, ndInt32 miniBatchSize,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
		ndBrainGpuBuffer* const inputOutputData,
		ndBrainGpuBuffer* const weightsAndBias,
		ndBrainGpuBuffer* const inputOutputGradients,
		ndBrainGpuBuffer* const weightsAndBiasGradients) const;
};

#endif 

