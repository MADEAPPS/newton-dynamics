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

#ifndef _ND_BRAIN_INSTANCE_H__
#define _ND_BRAIN_INSTANCE_H__

#include "ndBrainStdafx.h"
#include "ndBrainTypes.h"
#include "ndBrainVector.h"

class ndBrain;
class ndBrainLayer;
class ndBrainTrainerBase;

class ndBrainInstance: public ndClassAlloc
{
	public: 
	ndBrainInstance(ndBrain* const brain);
	ndBrainInstance(const ndBrainInstance& src);
	~ndBrainInstance();

	void CalculatePrefixScan();
	ndBrain* GetBrain() const;
	ndBrainVector& GetOutput();
	const ndBrainVector& GetOutput() const;
	const ndBrainPrefixScan& GetPrefixScan() const;

	void MakePrediction(const ndBrainVector& input, ndBrainVector& output);
	void MakePrediction(ndThreadPool& threadPool, const ndBrainVector& input, ndBrainVector& output);

	protected:
	ndBrainVector m_z;
	ndBrainPrefixScan m_zPrefixScan;
	ndBrain* m_brain;
};

inline ndBrain* ndBrainInstance::GetBrain() const
{
	return m_brain;
}

inline ndBrainVector& ndBrainInstance::GetOutput()
{
	return m_z;
}

inline const ndBrainVector& ndBrainInstance::GetOutput() const
{
	return m_z;
}

inline const ndBrainPrefixScan& ndBrainInstance::GetPrefixScan() const
{
	return m_zPrefixScan;
}

#endif 

