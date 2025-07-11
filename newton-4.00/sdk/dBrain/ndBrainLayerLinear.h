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

#ifndef _ND_BRAIN_LAYER_LINEAR_H__
#define _ND_BRAIN_LAYER_LINEAR_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

#define ND_BRAIN_LAYER_LINEAR_NAME	"ndBrainLayerLinear"

#define ND_GPU_TILED_MATRIX_ROWS_BITS		4
#define ND_GPU_TILED_MATRIX_ROWS			(1<<ND_GPU_TILED_MATRIX_ROWS_BITS)
#define ND_CPU_MINI_BATCH_SIZE_GRANULARITY	(ND_GPU_TILED_MATRIX_ROWS * 2)

class ndBrainLayerLinear : public ndBrainLayer
{
	public: 
	ndBrainLayerLinear(ndInt32 inputs, ndInt32 outputs);
	ndBrainLayerLinear(const ndBrainLayerLinear& src);
	virtual ~ndBrainLayerLinear();
	virtual ndBrainLayer* Clone() const override;

	virtual bool HasParameters() const override;
	virtual ndInt32 GetOutputSize() const override;
	virtual ndInt32 GetInputSize() const override;
	virtual const char* GetLabelId() const override;
	virtual ndInt32 GetNumberOfParameters() const override;
	
	virtual ndBrainVector* GetBias();
	virtual ndBrainMatrix* GetWeights();
	
	virtual void InitWeights() override;
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output,
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const override;

	virtual void Save(const ndBrainSave* const loadSave) const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);
	
	void Clear() override;
	void FlushToZero() override;
	void Scale(ndBrainFloat scale) override;
	void Set(const ndBrainLayer& src) override;
	void Add(const ndBrainLayer& src) override;
	void Mul(const ndBrainLayer& src) override;
	void Blend(const ndBrainLayer& src, ndBrainFloat blend) override;
	void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale) override;

	void AddReqularizerL1(const ndBrainLayer& weights, ndBrainFloat regularizer) override;
	void AddReqularizerL2(const ndBrainLayer& weights, ndBrainFloat regularizer) override;

	protected:
	enum BackpropagatePass
	{
		m_biasPass,
		m_weightsPass,
		m_inputGradientsPass,
		m_biasAddPartialSumPass,
		m_biasCachePartialSumPass,
		m_dimFactor = 16,
	};

	void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon) override;

	virtual void SetCpuWeights(const ndBrainVector& input) override;
	virtual void SetGpuWeights(const ndBrainVector& input) override;

	virtual void CopyCpuWeights(ndBrainVector& oput) const override;
	virtual void CopyGpuWeights(ndBrainVector& oput) const override;

	void CalculateRoundedSize(ndInt32& width, ndInt32& height) const;
	virtual ndCommandSharedInfo GetCpuCommandSharedInfo() const override;
	virtual ndCommandSharedInfo GetGpuCommandSharedInfo() const override;

	virtual bool HasGpuSupport() const override;
	virtual void FeedForward(const ndBrainLayerFeedForwardCpuCommand* const info, ndInt32 miniBatchIndex) const override;
	virtual void BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const info, ndInt32 miniBatchIndex) const override;

	void BackPropagateBiasGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const;
	void BackPropagateInputGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const;
	void BackPropagateWeightsGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const;
	void BackPropagateBiasAddPartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const;
	void BackPropagateBiasCachePartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const;
	
	virtual ndCommandArray CreateGpuFeedForwardCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override;

	virtual ndCommandArray CreateGpuBackPropagateCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context, 
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		//const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const override;

	ndBrainVector m_bias;
	ndBrainMatrix m_weights;
	friend class ndBrainTrainerInference;
};

#endif 

