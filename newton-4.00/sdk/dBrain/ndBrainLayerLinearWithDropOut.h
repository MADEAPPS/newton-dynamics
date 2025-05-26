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
#include "ndBrainLayerActivation.h"
//#include "ndBrainLayerLinear.h"

//#define ND_LINEAL_DROPOUT_FACTOR ndBrainFloat(0.7f)
#define ND_BRAIN_LAYER_ACTIVATION_LINEAR_DROPOUT	"ndBrainLayerLinearWithDropOut"

class ndBrainLayerLinearWithDropOut : public ndBrainLayerActivation
{
	public: 
	ndBrainLayerLinearWithDropOut(ndInt32 neurons);
	ndBrainLayerLinearWithDropOut(const ndBrainLayerLinearWithDropOut& src);

	virtual ndBrainLayer* Clone() const;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	virtual const char* GetLabelId() const;
	virtual void ApplyDropOut(ndFloat32 rate);

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual bool HasGpuSupport() const override;
	virtual ndLayerUniformDataCpu* GetLayerUniformDataCpu() const override;
	virtual ndLayerUniformDataGpu GetLayerUniformDataGpu(const ndBrainGpuContext* const context) const override;

	ndBrainVector m_dropOut;
};

#endif 

