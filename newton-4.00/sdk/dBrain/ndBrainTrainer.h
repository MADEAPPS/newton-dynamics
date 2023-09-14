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

#ifndef _ND_BRAIN_TRAINER_H__
#define _ND_BRAIN_TRAINER_H__

#include "ndBrainStdafx.h"
class ndBrain;
class ndBrainLoss;
class ndBrainVector;

class ndBrainTrainer: public ndClassAlloc
{
	public: 
	class ndLayerData;

	enum ndSolveModel
	{
		m_sgd,
		m_adam
	};

	ndBrainTrainer(ndBrain* const brain);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndBrain* GetBrain() const;

	ndReal GetRegularizer() const;
	void SetRegularizer(ndReal regularizer);

	ndSolveModel GetModel() const;
	void SetModel(ndSolveModel model);

	void ClearGradientsAcc();
	void AcculumateGradients(const ndBrainTrainer& src);
	void UpdateWeights(ndReal learnRate, ndInt32 batchSize);
	void BackPropagate(const ndBrainVector& input, ndBrainLoss& loss);

	private:
	//void BackPropagateOutputLayer(const ndBrainVector& loss);
	//void BackPropagateCalculateBiasGradient(ndInt32 layerIndex);
	//void BackPropagateCalculateWeightsGradient(ndInt32 layerIndex);

	void AdamUpdate(ndReal learnRate);
	void StochasticUpdate(ndReal learnRate);

	ndArray<ndLayerData*> m_layerData;
	ndBrain* m_brain;
	ndReal m_beta;
	ndReal m_alpha;
	ndReal m_epsilon;
	ndReal m_betaAcc;
	ndReal m_alphaAcc;
	ndReal m_weighDecayRegularizer;
	ndSolveModel m_model;
};

#endif 

