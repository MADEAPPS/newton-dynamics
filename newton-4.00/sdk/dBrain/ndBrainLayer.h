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
#include "ndBrainTypes.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayer: public ndBrainMatrix
{
	public: 
	ndBrainLayer(const ndBrainLayer& src);
	//ndBrainLayer(const nd::TiXmlNode* layerNode);
	ndBrainLayer(ndInt32 inputs, ndInt32 outputs, ndBrainActivationType type);
	virtual ~ndBrainLayer();

	ndReal* SetFloatPointers(ndReal* const mem);
	ndUnsigned8* SetPointers(ndUnsigned8* const mem);

	virtual ndBrainLayer* Clone() const;
	
	ndBrainVector& GetBias();
	const ndBrainVector& GetBias() const;

	virtual ndInt32 GetOuputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual void InitGaussianWeights(ndReal variance);
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output);
	virtual void MakePrediction(ndThreadPool& threadPool, const ndBrainVector& input, ndBrainVector& output);

	virtual void CopyFrom(const ndBrainLayer& src);
	virtual bool Compare(const ndBrainLayer& src) const;

	//virtual void Load(const nd::TiXmlElement* const layerNode);
	//virtual void Save(nd::TiXmlElement* const layerNode) const;

	void ApplyActivation(ndBrainVector& output) const;
	void ActivationDerivative(const ndBrainVector& input, ndBrainVector& outputDerivative) const;

	protected:
	void ReluActivation(ndBrainVector& output) const;
	void LinealActivation(ndBrainVector& output) const;
	void SigmoidActivation(ndBrainVector& output) const;
	void SoftmaxActivation(ndBrainVector& output) const;
	void HyperbolicTanActivation(ndBrainVector& output) const;

	void SigmoidDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void HyperbolicTanDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void ReluActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void LinealActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;

	ndDeepBrainMemVector m_bias;
	ndBrainActivationType m_activation;
	ndInt32 m_columns;
};

inline ndInt32 ndBrainLayer::GetOuputSize() const
{
	return m_bias.GetCount();
}

inline ndInt32 ndBrainLayer::GetInputSize() const
{
	return m_columns;
}

inline ndBrainVector& ndBrainLayer::GetBias()
{
	return m_bias;
}

inline const ndBrainVector& ndBrainLayer::GetBias() const
{
	return m_bias;
}

#endif 

