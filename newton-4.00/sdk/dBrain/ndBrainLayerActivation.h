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

#ifndef _ND_BRAIN_LAYER_ACTIVATION_H__
#define _ND_BRAIN_LAYER_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainTypes.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayerActivation : public ndBrainLayer
{
	public: 
	ndBrainLayerActivation(ndInt32 neurons);
	ndBrainLayerActivation(const ndBrainLayerActivation& src);
	virtual ~ndBrainLayerActivation();
	virtual ndBrainLayer* Clone() const;
	virtual bool HasParameters() const;

	virtual const char* GetLabelId() const;
	virtual ndInt32 GetOuputSize() const;
	virtual ndInt32 GetInputSize() const;

	virtual void InitWeightsXavierMethod();
	virtual void InitWeights(ndReal weighVariance, ndReal biasVariance);
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output);
	
	//virtual void CopyFrom(const ndBrainLayer& src);
	//virtual void Blend(const ndBrainLayer& src, ndReal blend);
	//
	//virtual bool Compare(const ndBrainLayer& src) const;
	//
	//virtual void Load(const ndBrainLoad* const loadSave);
	//virtual void Save(const ndBrainSave* const loadSave) const;
	//
	//virtual void ApplyActivation(ndBrainVector& output) const;
	//virtual void ActivationDerivative(const ndBrainVector& input, ndBrainVector& outputDerivative) const;

	protected:
	ndInt32 m_neurons;
};

#endif 

