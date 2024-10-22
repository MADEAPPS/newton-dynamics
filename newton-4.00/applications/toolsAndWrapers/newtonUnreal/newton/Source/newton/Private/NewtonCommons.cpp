// Copyright Epic Games, Inc. All Rights Reserved.

#include "NewtonCommons.h"


ndHullPoints::ndHullPoints()
	:TArray<FVector3f>()
{
}

ndHullPoints::~ndHullPoints()
{
}

ndConvexHullSet::ndConvexHullSet()
	:TArray<ndHullPoints*>()
{
}

ndConvexHullSet::~ndConvexHullSet()
{
	for (int i = 0; i < Num(); ++i)
	{
		delete (*this)[i];
	}
}

