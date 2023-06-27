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

#ifndef _ND_BRAIN_TRAINER_BASE_H__
#define _ND_BRAIN_TRAINER_BASE_H__

#include "ndBrainStdafx.h"
#include "ndBrainMatrix.h"
#include "ndBrainInstance.h"

class ndBrainTrainerBase: public ndClassAlloc
{
	public: 
	enum ndSolveModel
	{
		m_cgd,
		m_adam,
	};

	class ndValidation
	{
		public:
		ndValidation(ndBrainTrainerBase& trainer)
			:m_trainer(trainer)
		{
		}

		virtual ~ndValidation()
		{
		}

		virtual ndReal Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth);

		ndBrainTrainerBase& m_trainer;
		ndBrainVector m_output;
	};


	ndBrainTrainerBase(ndBrain* const brain);
	ndBrainTrainerBase(const ndBrainTrainerBase& src);
	virtual ~ndBrainTrainerBase();

	ndSolveModel GetModel() const;
	void SetModel(ndSolveModel model);

	ndBrain* GetBrain() const;
	ndBrainInstance& GetInstance();
	const ndBrainInstance& GetInstance() const;
	void SetMiniBatchSize(ndInt32 m_miniBatchSize);
	virtual ndReal Validate(const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndBrainVector& output);

	virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const = 0;
	virtual void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, ndReal learnRate, ndInt32 steps) = 0;
	//void Optimize___(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndReal learnRate, ndInt32 steps);

	protected:
	ndBrainInstance m_instance;
	ndInt32 m_miniBatchSize;
	ndSolveModel m_model;

	friend class ndBrainInstance;
};

inline ndBrainTrainerBase::ndSolveModel ndBrainTrainerBase::GetModel() const
{
	return m_model;
}

inline void ndBrainTrainerBase::SetModel(ndSolveModel model)
{
	m_model = model;
}

inline ndBrainInstance& ndBrainTrainerBase::GetInstance()
{
	return m_instance;
}

inline const ndBrainInstance& ndBrainTrainerBase::GetInstance() const
{
	return m_instance;
}

inline ndBrain* ndBrainTrainerBase::GetBrain() const
{
	return GetInstance().GetBrain();
}
#endif 

