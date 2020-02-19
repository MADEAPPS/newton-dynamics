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

#include "dAnimationStdAfx.h"
#include "dAnimationKeyframesTrack.h"


const void dAnimimationKeyFramesTrack::InterpolatePosition(dFloat t, dVector& posit) const
{
	if (m_position.GetSize()) {
		int base = m_position.GetIndex(t);
		const dFloat t0 = m_position[base].m_time;
		const dFloat t1 = m_position[base + 1].m_time;
		const dFloat param = (t - t0) / (t1 - t0 + dFloat(1.0e-6f));
		const dVector& p0 = m_position[base].m_posit;
		const dVector& p1 = m_position[base + 1].m_posit;
		posit = p0 + (p1 - p0).Scale(param);
	}
}

const void dAnimimationKeyFramesTrack::InterpolateRotation(dFloat t, dQuaternion& rotation) const
{
	if (m_rotation.GetSize()) {
		int base = m_rotation.GetIndex(t);
		const dFloat t0 = m_rotation[base].m_time;
		const dFloat t1 = m_rotation[base + 1].m_time;
		const dFloat param = (t - t0) / (t1 - t0 + dFloat (1.0e-6f));
		const dQuaternion& rot0 = m_rotation[base].m_rotation;
		const dQuaternion& rot1 = m_rotation[base + 1].m_rotation;
		rotation = rot0.Slerp(rot1, param);
	}
}

void dAnimimationKeyFramesTrack::Save(FILE* const file) const
{
	fprintf(file, "\ttrackName: %s\n", m_name.GetStr());
	
	fprintf(file, "\t\tpositions (t, x, y, z): %d\n", m_position.GetSize());
	for (int i = 0; i < m_position.GetSize(); i++) {
		fprintf(file, "\t\t\t%f %f %f %f\n", m_position[i].m_time, m_position[i].m_posit.m_x, m_position[i].m_posit.m_y, m_position[i].m_posit.m_z);
	}

	fprintf(file, "\t\trotation (t, qx, qy, qz, qw): %d\n", m_rotation.GetSize());
	for (int i = 0; i < m_rotation.GetSize(); i++) {
		dQuaternion q(m_rotation[i].m_rotation);
		fprintf(file, "\t\t\t%f %f %f %f %f\n", m_rotation[i].m_time, q.m_x, q.m_y, q.m_z, q.m_w);
	}

}