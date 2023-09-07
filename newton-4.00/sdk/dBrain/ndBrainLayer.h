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

class ndBrainLoad;
class ndBrainSave;
class ndBrainVector;

class ndBrainLayer : public ndClassAlloc
{
	public: 
	ndBrainLayer();
	ndBrainLayer(const ndBrainLayer& src);

	virtual ~ndBrainLayer();
	virtual ndBrainLayer* Clone() const;

	virtual ndInt32 GetOuputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual void InitGaussianBias(ndReal variance);
	virtual void InitGaussianWeights(ndReal variance);
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output);

	virtual void InitWeightsXavierMethod();
	virtual void InitWeights(ndReal weighVariance, ndReal biasVariance);

	virtual ndBrainActivationType GetActivationType() const;

	virtual void CopyFrom(const ndBrainLayer& src);
	virtual void Blend(const ndBrainLayer& src, ndReal blend);

	virtual bool Compare(const ndBrainLayer& src) const;

	virtual void Load(const ndBrainLoad* const loadSave);
	virtual void Save(const ndBrainSave* const loadSave) const;

	virtual void ApplyActivation(ndBrainVector& output) const;
	virtual void ActivationDerivative(const ndBrainVector& input, ndBrainVector& outputDerivative) const;
};


inline ndBrainLayer::ndBrainLayer(const ndBrainLayer& src)
	:ndClassAlloc(src)
{
}

inline ndBrainLayer::ndBrainLayer()
	:ndClassAlloc()
{
}

inline ndBrainLayer::~ndBrainLayer()
{
}

inline ndBrainLayer* ndBrainLayer::Clone() const
{
	ndAssert(0);
	return nullptr;
}

inline ndInt32 ndBrainLayer::GetOuputSize() const
{
	ndAssert(0);
	return 0;
}

inline ndInt32 ndBrainLayer::GetInputSize() const
{
	ndAssert(0);
	return 0;
}

inline void ndBrainLayer::InitGaussianBias(ndReal)
{
	ndAssert(0);
}

inline void ndBrainLayer::InitGaussianWeights(ndReal)
{
	ndAssert(0);
}

inline void ndBrainLayer::MakePrediction(const ndBrainVector&, ndBrainVector&)
{
	ndAssert(0);
}

inline void ndBrainLayer::InitWeightsXavierMethod()
{
	ndAssert(0);
}

inline void ndBrainLayer::InitWeights(ndReal, ndReal)
{
	ndAssert(0);
}

inline ndBrainActivationType ndBrainLayer::GetActivationType() const
{
	ndAssert(0);
	return m_noActivation;
}

inline void ndBrainLayer::CopyFrom(const ndBrainLayer&)
{
	ndAssert(0);
}

inline void ndBrainLayer::Blend(const ndBrainLayer&, ndReal)
{
	ndAssert(0);
}

inline bool ndBrainLayer::Compare(const ndBrainLayer&) const
{
	ndAssert(0);
	return false;
}

inline void ndBrainLayer::Load(const ndBrainLoad* const)
{
	ndAssert(0);
}

inline void ndBrainLayer::Save(const ndBrainSave* const) const
{
	ndAssert(0);
}

inline void ndBrainLayer::ApplyActivation(ndBrainVector&) const
{
	ndAssert(0);
}

inline void ndBrainLayer::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}
#endif 

