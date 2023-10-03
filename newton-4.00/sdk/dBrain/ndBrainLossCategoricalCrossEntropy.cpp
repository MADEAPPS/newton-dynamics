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

#include "ndBrainStdafx.h"
#include "ndBrainLossCategoricalCrossEntropy.h"

ndBrainLossCategoricalCrossEntropy::ndBrainLossCategoricalCrossEntropy(ndInt32 size)
	:ndBrainLoss()
{
	m_truth.SetCount(size);
}

bool ndBrainLossCategoricalCrossEntropy::IsCategorical() const
{
	return true;
}

void ndBrainLossCategoricalCrossEntropy::SetTruth(const ndBrainVector& truth)
{
#ifdef _DEBUG
	ndAssert(m_truth.GetCount() == truth.GetCount());
	ndInt32 index = 0;
	for (ndInt32 i = 0; i < truth.GetCount(); ++i)
	{
		ndAssert((truth[i] == ndBrainFloat(0.0f)) || (truth[i] == ndBrainFloat(1.0f)));
		index += (truth[i] == ndBrainFloat(1.0f)) ? 1 : 0;
	}
	ndAssert(index == 1);
#endif
	
	m_truth.Set(truth);
}

// note: Categorical entropy loss is designed you work with the SoftMax activation layer
// the rules for using it are
// 1- can only be use as when the last layer of the neural net is SoftMax layer
// 2- the function does not calculate the derivative since this is done by the SoftMax layer 
// which make use that the combine the truth value can only be 1 or 0, 
// and this fact cancel out many term from the derivative equation. 
// in that regard the loss is just the truth value.
void ndBrainLossCategoricalCrossEntropy::GetLoss(const ndBrainVector&, ndBrainVector& loss)
{
	//ndAssert(output.GetCount() == loss.GetCount());
	ndAssert(m_truth.GetCount() == loss.GetCount());
	loss.Set(m_truth);
}