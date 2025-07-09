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
#include "ndBrainContext.h"
#include "ndBrainBufferCommand.h"

class ndBrainLoad;
class ndBrainSave;
class ndBrainLayer;
class ndBrainVector;
class ndBrainMatrix;
class ndBrainGpuBuffer;
class ndBrainGpuCommand;
class ndBrainFloatBuffer;
class ndBrainBufferCommand;
class ndBrainUniformBuffer;
class ndBrainTrainerInference;

class ndBrainLayerFeedForwardCpuCommand : public ndBrainBufferCommandCpu
{
	public:
	ndBrainLayerFeedForwardCpuCommand(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer);

	virtual void Execute(ndInt32 miniBatchIndex) override;

	ndBrainLayer* m_layer;
};

class ndBrainLayerBackPropagateCpuCommand : public ndBrainBufferCommandCpu
{ 
	public:
	ndBrainLayerBackPropagateCpuCommand(const ndBrainBufferCommandDesc& desc, ndBrainLayer* const layer);

	virtual void Execute(ndInt32 miniBatchIndex) override;

	ndBrainLayer* m_layer;
};

class ndCommandArray: public ndFixSizeArray<ndBrainBufferCommand*, 32>
{
	public:
	ndCommandArray()
		:ndFixSizeArray<ndBrainBufferCommand*, 32>(0)
	{
	}
	ndCommandArray(ndInt32 initialSize)
		:ndFixSizeArray<ndBrainBufferCommand*, 32>(initialSize)
	{
	}
};

class ndBrainLayer : public ndClassAlloc
{
	public: 
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

	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output, 
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	virtual void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon);

	virtual void SetCpuWeights(const ndBrainVector& input);
	virtual void SetGpuWeights(const ndBrainVector& input);

	virtual void CopyCpuWeights(ndBrainVector& output) const;
	virtual void CopyGpuWeights(ndBrainVector& output) const;

	virtual ndCommandSharedInfo GetCpuCommandSharedInfo() const;
	virtual ndCommandSharedInfo GetGpuCommandSharedInfo() const;

	virtual bool HasGpuSupport() const;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const info, ndInt32 miniBatchIndex) const;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const;

	virtual ndCommandArray CreateGpuFeedForwardCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const;

	virtual ndCommandArray CreateGpuBackPropagateCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context, 
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const;

	ndBrainBufferCommandDesc MakeFeedForwardDesctriptor(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndInt32 tileStride,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const;

	ndBrainBufferCommandDesc MakeBackpropagateDesctriptor(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndInt32 tileStride,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const;
};

#endif 

