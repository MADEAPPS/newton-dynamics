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

#ifndef _dBonesToPoseBinding_
#define _dBonesToPoseBinding_
#include <dClassInfo.h>


class dAnimationClip;
class dPoseGenerator;
class dPoseTransform;

typedef void (*PosetUpdateCallback) (void* useData, dFloat* posit, dFloat* rotation);

class dBindFrameToNode
{
    public:
    void* m_userData;
    dPoseTransform* m_sourceTranform;
};

class dBonesToPoseBinding: public dList<dBindFrameToNode>, virtual public dClassInfo
{
    public:
	dBonesToPoseBinding(dAnimationClip* clip);
    dBonesToPoseBinding(dPoseGenerator* pose);
	dBonesToPoseBinding(const char* clipFileName);

	dPoseGenerator* GetPoseGenerator () const;
	void SetPoseGenerator(dPoseGenerator* pose);

    void UpdatePose() const;
    void GeneratePose (dFloat param);
    void SetUpdateCallback (PosetUpdateCallback callback);
    
    protected:
	dAddRtti(dClassInfo);

    ~dBonesToPoseBinding(void);
    dPoseGenerator* m_pose;
    PosetUpdateCallback m_updateCallback;
};


#endif
