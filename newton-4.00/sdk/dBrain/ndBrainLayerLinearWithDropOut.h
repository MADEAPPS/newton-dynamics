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

#ifndef _ND_BRAIN_LAYER_LINEAR_WITH_DROP_OUT_H__
#define _ND_BRAIN_LAYER_LINEAR_WITH_DROP_OUT_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerLinear.h"

class ndBrainLayerLinearWithDropOut : public ndBrainLayerLinear
{
	public: 
	ndBrainLayerLinearWithDropOut(ndInt32 inputs, ndInt32 outputs, ndBrainFloat dropOutFactor = ndBrainFloat(0.9f));
	ndBrainLayerLinearWithDropOut(const ndBrainLayerLinearWithDropOut& src);
	virtual ~ndBrainLayerLinearWithDropOut();
	virtual ndBrainLayer* Clone() const;

	virtual void UpdateDropOut();
	virtual const char* GetLabelId() const;
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output,
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	ndBrainVector m_dropout;
	ndBrainFloat m_dropoutFactor;
};

#endif 

