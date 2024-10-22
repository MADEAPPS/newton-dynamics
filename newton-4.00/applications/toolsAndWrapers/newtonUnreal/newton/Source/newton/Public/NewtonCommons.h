// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once


#define UNREAL_UNIT_SYSTEM		ndFloat32 (100.0f)
#define UNREAL_INV_UNIT_SYSTEM	ndFloat32 (1.0f / UNREAL_UNIT_SYSTEM)

class ndShape;
class ANewtonWorldActor;

template <typename T>
class ndSharedPtr;

enum ndPluginVersion
{
	m_firstVersion,
};

class ndHullPoints : public TArray<FVector3f>
{
	public:
};

class ndConvexHullSet : public TArray<ndHullPoints*>
{
	public:
	ndConvexHullSet()
		:TArray<ndHullPoints*>()
	{
	}

	~ndConvexHullSet()
	{
		for (int i = 0; i < Num(); ++i)
		{
			delete (*this)[i];
		}
	}
};
