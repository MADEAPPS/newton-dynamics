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

#ifndef _ND_DEEP_BRAIN_TRAINER_SGD_H__
#define _ND_DEEP_BRAIN_TRAINER_SGD_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainTrainingOperator.h"

class ndDeepBrainTrainerSDG: public ndDeepBrainTrainingOperator
{
	public: 
	ndDeepBrainTrainerSDG(ndDeepBrain* const brain);
	ndDeepBrainTrainerSDG(const ndDeepBrainTrainerSDG& src);
	virtual ~ndDeepBrainTrainerSDG();

	virtual void UpdateWeights(ndReal learnRate);
	virtual void MakePrediction(const ndDeepBrainVector& input);
	virtual void BackPropagate(const ndDeepBrainVector& groundTruth);
	virtual void Optimize(const ndDeepBrainMatrix& inputBatch, const ndDeepBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps);

	protected:
	virtual void PrefixScan();
	virtual void ApplyWeightTranspose();
	
	virtual void BackPropagateHiddenLayer(ndInt32 layerIndex);
	virtual void BackPropagateCalculateBiasGradient(ndInt32 layerIndex);
	virtual void BackPropagateOutputLayer(const ndDeepBrainVector& groundTruth);

	ndDeepBrainVector m_output;
	ndDeepBrainVector m_zDerivative;
	ndDeepBrainVector m_biasGradients;
	ndDeepBrainVector m_weightGradients;
	ndDeepBrainPrefixScan m_weightGradientsPrefixScan;
	ndArray <ndDeepBrainMatrix*> m_weightsLayersTranspose;
};

#endif 

