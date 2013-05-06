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

#pragma once




struct msModel;


class Exporter : public cMsPlugIn
{
	public:
	Exporter ();
    virtual ~Exporter ();


    int             GetType ();
    const char *    GetTitle ();
    int             Execute (msModel* pModel);

//	void ConvertMeshToSkins(dModel& model);
//	void MergeEqualMaterials(dModel& model);
//    void AddSkeleton (msModel* pModel, dModel& model);
};

