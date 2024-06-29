/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndAnimationKeyframesTrack.h"

template<class OBJECT>
ndInt32 ndAnimationKeyFramesTrack::ndKeyFramesArray<OBJECT>::GetIndex(ndFloat32 time) const
{
	ndAssert(time >= 0.0f);
	const ndInt32 count = ndInt32(ndArray<OBJECT>::GetCount());
	const ndFloat32* const timePtr = &m_time[0];
	time = ndClamp(time, timePtr[0], timePtr[count - 1]);

	ndInt32 i0 = 1;
	ndInt32 i1 = count - 1;
	while ((i1 - i0) > 4)
	{
		const ndInt32 mid = (i1 + i0) / 2;
		if (timePtr[mid] > time)
		{
			i1 = mid;
		}
		else
		{
			i0 = mid;
		}
	}

	ndInt32 index = i0 - 1;
	ndAssert(timePtr[index] <= time);
	for (ndInt32 i = i0; i < count; ++i)
	{
		if (timePtr[i] >= time)
		{
			index = i;
			break;
		}
	}

	return index;
}

void ndAnimationKeyFramesTrack::InterpolatePosition(ndFloat32 param, ndVector& posit) const
{
	if (m_position.GetCount() >= 2)
	{
		const ndInt32 base = m_position.GetIndex(param);
		const ndFloat32 t0 = m_position.m_time[base - 1];
		const ndFloat32 t1 = m_position.m_time[base - 0];
		const ndVector& p0 = m_position[base - 1];
		const ndVector& p1 = m_position[base - 0];
		const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
		posit = p0 + (p1 - p0).Scale(t);
	}
	else if (m_position.GetCount() == 1)
	{
		posit = m_position[0];
	}
}

void ndAnimationKeyFramesTrack::InterpolateRotation(ndFloat32 param, ndQuaternion& rotation) const
{
	if (m_rotation.GetCount() >= 2)
	{
		const ndInt32 base = m_rotation.GetIndex(param);
		const ndFloat32 t0 = m_rotation.m_time[base - 1];
		const ndFloat32 t1 = m_rotation.m_time[base - 0];
		const ndQuaternion& rot0 = m_rotation[base - 1];
		const ndQuaternion& rot1 = m_rotation[base - 0];
		const ndFloat32 t = (param - t0) / (t1 - t0 + ndFloat32(1.0e-6f));
		ndAssert(rot0.DotProduct(rot1).GetScalar() > 0.0f);
		rotation = rot0.Slerp(rot1, t);
	}
	else if (m_rotation.GetCount() == 1)
	{
		rotation = m_rotation[0];
	}
}
