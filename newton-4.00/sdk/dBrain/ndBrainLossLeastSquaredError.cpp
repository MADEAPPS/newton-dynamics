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
#include "ndBrainLossLeastSquaredError.h"

ndBrainLossLeastSquaredError::ndBrainLossLeastSquaredError(ndInt32 size)
	:ndBrainLoss()
{
	m_truth.SetCount(size);
}

bool ndBrainLossLeastSquaredError::HasGpuSupport() const
{
	return true;
}

void ndBrainLossLeastSquaredError::SetTruth(const ndBrainVector& truth)
{
	ndAssert(m_truth.GetCount() == truth.GetCount());
	m_truth.Set(truth);
}

void ndBrainLossLeastSquaredError::GetLoss(const ndBrainVector& output, ndBrainVector& loss)
{
	ndAssert(output.GetCount() == loss.GetCount());
	ndAssert(m_truth.GetCount() == loss.GetCount());
	loss.Set(output);
	loss.Sub(m_truth);
}

ndBrainLossHuber::ndBrainLossHuber(ndInt32 size, ndBrainFloat lambda)
	:ndBrainLossLeastSquaredError(size)
	,m_lambda(lambda)
{
}


void ndBrainLossHuber::GetLoss(const ndBrainVector& output, ndBrainVector& loss)
{
	ndAssert(output.GetCount() == loss.GetCount());
	ndAssert(m_truth.GetCount() == loss.GetCount());

	ndBrainLossLeastSquaredError::GetLoss(output, loss);
	for (ndInt32 i = ndInt32(output.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat x = loss[i];
		if (x > m_lambda)
		{
			loss[i] = m_lambda;
		}
		else if (x < -m_lambda)
		{
			loss[i] = -m_lambda;
		}
	}
}
