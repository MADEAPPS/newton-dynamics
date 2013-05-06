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

#ifndef _dAnimationClip_
#define _dAnimationClip_
#include <dClassInfo.h>

class TiXmlElement;
class dPoseGenerator;

class dKeyFrames
{
    public:
    dKeyFrames();
    ~dKeyFrames();

	void AllocaFrames (int count);

	int FindKey(float entry) const;

    char m_bindName[D_NAME_STRING_LENGTH];
    int m_keyFrameCounts;
    int* m_keys;
    dVector* m_posit;
    dQuaternion* m_rotation;
};




class dAnimationClip: public dList<dKeyFrames>, virtual public dClassInfo  
{
	public:
	dAnimationClip(void);
	~dAnimationClip(void);

#ifdef D_LOAD_SAVE_XML
	void LoadXML (const char* fileName);
	void SaveXML (const char* fileName) const;
#endif

	void RemoveNode (const char* fileName);

	int GetFramesCount() const;
	void SetFramesCount(int count);


	dAddRtti(dClassInfo);

	int m_framesCount;
	char m_name[D_NAME_STRING_LENGTH];

	friend class dPoseGenerator;
};


#endif
