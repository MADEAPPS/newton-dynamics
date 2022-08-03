/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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


#ifndef _ND_DEEP_BRAIN_NEURON_H__
#define _ND_DEEP_BRAIN_NEURON_H__

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainVector.h"

class ndDeepBrainNeuron : public ndClassAlloc
{
	public: 
	ndDeepBrainNeuron(ndInt32 inputs);
	~ndDeepBrainNeuron();

	ndDeepBrainVector& GetWeights();
	void InitGaussianWeights(ndFloat32 mean, ndFloat32 variance);

	ndFloat32 FowardPass(const ndDeepBrainVector& input);

	protected:
	ndFloat32 m_weight0;
	ndDeepBrainVector m_weights;
};

#endif 

