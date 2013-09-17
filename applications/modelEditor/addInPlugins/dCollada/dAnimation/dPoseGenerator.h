/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _dPoseGenerator_
#define _dPoseGenerator_
#include <dClassInfo.h>

class dKeyFrames;
class dAnimationClip;
class dPoseGeneratorBind;


class dPoseTransform
{
    public:
    dQuaternion m_rotation;
    dFloat m_posit[3];
    dKeyFrames* m_source;
};

class dPoseGenerator: public dList<dPoseTransform>, virtual public dClassInfo
{
    public:
    dPoseGenerator(dAnimationClip* clip);
	dPoseGenerator(const char* clipFileNameName);

	dAnimationClip* GetAnimationClip() const;
	void SetAnimationClip(dAnimationClip* clip);
    void Generate (dFloat param);

    protected:
    ~dPoseGenerator(void);

    dAnimationClip* m_clip;

	dAddRtti(dClassInfo);
    friend class dPoseGeneratorBind;
};


#endif
