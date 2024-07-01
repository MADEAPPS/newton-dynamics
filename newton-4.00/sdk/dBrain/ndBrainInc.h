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

#ifndef _ND_BRAIN_INC_H__
#define _ND_BRAIN_INC_H__

#include <ndBrainStdafx.h>

#include <ndBrain.h>
#include <ndBrainLoss.h>
#include <ndBrainAgent.h>
#include <ndBrainLayer.h>
#include <ndBrainFloat4.h>
#include <ndBrainVector.h>
#include <ndBrainMatrix.h>
#include <ndBrainTrainer.h>
#include <ndBrainSaveLoad.h>
#include <ndBrainAgentDQN.h>
#include <ndBrainAgentDDPG.h>
#include <ndBrainOptimizer.h>
#include <ndBrainGpuBuffer.h>
#include <ndBrainThreadPool.h>
#include <ndBrainGpuCommand.h>
#include <ndBrainGpuContext.h>
#include <ndBrainLayerLinear.h>
#include <ndBrainGpuInference.h>
#include <ndBrainReplayBuffer.h>
#include <ndBrainOptimizerSgd.h>
#include <ndBrainOptimizerAdam.h>
#include <ndBrainGpuFloatBuffer.h>
#include <ndBrainLayerActivation.h>
#include <ndBrainGpuIntegerBuffer.h>
#include <ndBrainGpuUniformBuffer.h>
#include <ndBrainAgentDQN_Trainer.h>
#include <ndBrainGpuScopeMapBuffer.h>
#include <ndBrainAgentDDPG_Trainer.h>
#include <ndBrainLayerImagePadding.h>
#include <ndBrainLayerReluActivation.h>
#include <ndBrainLayerTanhActivation.h>
#include <ndBrainLayerImagePolling_2x2.h>
#include <ndBrainLayerConvolutional_2d.h>
#include <ndBrainLossLeastSquaredError.h>
#include <ndBrainLayerLinearWithDropOut.h>
#include <ndBrainLayerSigmoidActivation.h>
#include <ndBrainLayerSoftmaxActivation.h>
#include <ndBrainLayerCrossCorrelation_2d.h>
#include <ndBrainAgentDiscretePolicyGradient.h>
#include <ndBrainAgentContinuePolicyGradient.h>
#include <ndBrainLossCategoricalCrossEntropy.h>
#include <ndBrainLayerSigmoidLinearActivation.h>
#include <ndBrainLayerConvolutionalWithDropOut_2d.h>
#include <ndBrainLayerCategoricalSoftmaxActivation.h>
#include <ndBrainAgentDiscretePolicyGradient_Trainer.h>
#include <ndBrainAgentContinuePolicyGradient_Trainer.h>

#endif 

