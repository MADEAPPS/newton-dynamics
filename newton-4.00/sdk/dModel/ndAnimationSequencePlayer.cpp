/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndModelStdafx.h"
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"
#include "ndAnimationSequencePlayer.h"

ndAnimationSequencePlayer::ndAnimationSequencePlayer(ndSharedPtr<ndAnimationSequence>& sequence)
	:ndAnimationBlendTreeNode(ndSharedPtr<ndAnimationBlendTreeNode>(nullptr))
	,m_sequence(sequence)
	,m_veloc(ndVector::m_zero)
	,m_time(ndFloat32 (0.0f))
{
}

ndAnimationSequencePlayer::~ndAnimationSequencePlayer()
{
}

ndFloat32 ndAnimationSequencePlayer::GetTime() const
{
	return m_time;
}

void ndAnimationSequencePlayer::SetTime(ndFloat32 time)
{
	ndFloat32 duration = m_sequence->GetDuration();
	m_time = ndFmod(time, duration);
	if (m_time < ndFloat32(0.0f))
	{
		m_time += duration;
	}
}

void ndAnimationSequencePlayer::Update(ndFloat32 timestep)
{
	ndFloat32 duration = m_sequence->GetDuration();
	ndAssert(m_time >= ndFloat32 (0.0f));
	ndAssert(m_time <= duration);

	ndFloat32 t0 = m_time;
	ndFloat32 t1 = t0 + timestep;

	m_veloc = ndVector::m_zero;
	if (timestep > ndFloat32(0.0f))
	{
		const ndVector p0(m_sequence->GetTranslation(t0));
		if (t1 < duration)
		{
			const ndVector p1(m_sequence->GetTranslation(t1));
			const ndVector step(p1 - p0);
			m_veloc = step.Scale(ndFloat32(1.0f) / timestep);
		}
		else
		{
			t1 -= duration;
			ndAssert(t1 <= duration);
			ndAssert(t1 >= ndFloat32(0.0f));
			const ndVector q(m_sequence->GetTranslation(duration));
			const ndVector p1(m_sequence->GetTranslation(t1) + q);
			const ndVector step(p1 - p0);
			m_veloc = step.Scale(ndFloat32(1.0f) / timestep);
		}
	}
	else if (timestep < ndFloat32(0.0f))
	{
		const ndVector p0(m_sequence->GetTranslation(t0));
		if (t1 >= ndFloat32(0.0f))
		{
			const ndVector p1(m_sequence->GetTranslation(t1));
			const ndVector step(p1 - p0);
			m_veloc = step.Scale(ndFloat32(-1.0f) / timestep);
		}
		else
		{
			//t1 = ndModAdd(t0, timestep, duration);
			t1 += duration;
			ndAssert(t1 <= duration);
			ndAssert(t1 >= ndFloat32(0.0f));
			const ndVector q(m_sequence->GetTranslation(duration));
			const ndVector p1(m_sequence->GetTranslation(t1) - q);
			const ndVector step(p1 - p0);
			m_veloc = step.Scale(ndFloat32(-1.0f) / timestep);
		}
	}
	m_veloc.m_w = ndFloat32(0.0f);
	SetTime(m_time + timestep);
}

ndSharedPtr<ndAnimationSequence>& ndAnimationSequencePlayer::GetSequence()
{
	return m_sequence;
}

void ndAnimationSequencePlayer::Evaluate(ndAnimationPose& output, ndVector& veloc)
{
	veloc = m_veloc;
	ndAssert(m_time >= ndFloat32(0.0f));
	ndAssert(m_time <= m_sequence->GetDuration());
	m_sequence->CalculatePose(output, m_time);
}
