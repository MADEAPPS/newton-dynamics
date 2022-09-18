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

#ifndef _ND_DEEP_BRAIN_LAYER_H__
#define _ND_DEEP_BRAIN_LAYER_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainTypes.h"
#include "ndDeepBrainVector.h"
#include "ndDeepBrainMatrix.h"

class ndDeepBrainLayer: public ndDeepBrainMatrix
{
	public: 
	ndDeepBrainLayer(const ndDeepBrainLayer& src);
	ndDeepBrainLayer(const nd::TiXmlNode* layerNode);
	ndDeepBrainLayer(ndInt32 inputs, ndInt32 outputs, ndDeepBrainActivationType type);
	virtual ~ndDeepBrainLayer();

	ndReal* SetFloatPointers(ndReal* const mem);
	ndUnsigned8* SetPointers(ndUnsigned8* const mem);

	virtual ndDeepBrainLayer* Clone() const;
	
	ndDeepBrainVector& GetBias();
	const ndDeepBrainVector& GetBias() const;

	virtual ndInt32 GetOuputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual void InitGaussianWeights(ndReal mean, ndReal variance);
	virtual void MakePrediction(const ndDeepBrainVector& input, ndDeepBrainVector& output);
	virtual void MakePrediction(ndThreadPool& threadPool, const ndDeepBrainVector& input, ndDeepBrainVector& output);

	virtual void CopyFrom(const ndDeepBrainLayer& src);
	virtual bool Compare(const ndDeepBrainLayer& src) const;

	virtual void Load(const nd::TiXmlElement* const layerNode);
	virtual void Save(nd::TiXmlElement* const layerNode) const;

	void ApplyActivation(ndDeepBrainVector& output) const;
	void ActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& outputDerivative) const;

	protected:
	void ReluActivation(ndDeepBrainVector& output) const;
	void LinealActivation(ndDeepBrainVector& output) const;
	void SigmoidActivation(ndDeepBrainVector& output) const;
	void SoftmaxActivation(ndDeepBrainVector& output) const;
	void HyperbolicTanActivation(ndDeepBrainVector& output) const;

	void SigmoidDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const;
	void HyperbolicTanDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const;
	void ReluActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const;
	void LinealActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const;

	ndDeepBrainMemVector m_bias;
	ndDeepBrainActivationType m_activation;
	ndInt32 m_columns;
};

inline ndInt32 ndDeepBrainLayer::GetOuputSize() const
{
	return m_bias.GetCount();
}

inline ndInt32 ndDeepBrainLayer::GetInputSize() const
{
	return m_columns;
}

inline ndDeepBrainVector& ndDeepBrainLayer::GetBias()
{
	return m_bias;
}

inline const ndDeepBrainVector& ndDeepBrainLayer::GetBias() const
{
	return m_bias;
}

#endif 

