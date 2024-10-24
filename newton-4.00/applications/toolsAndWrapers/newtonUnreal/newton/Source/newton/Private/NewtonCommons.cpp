

#include "NewtonCommons.h"

#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

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

FTransform ToUnRealTransform(const ndMatrix& matrix)
{
	const ndQuaternion rotation(matrix);
	const ndVector posit(matrix.m_posit.Scale(UNREAL_UNIT_SYSTEM));
	const FVector uPosit(posit.m_x, posit.m_y, posit.m_z);
	const FQuat uRot(rotation.m_x, rotation.m_y, rotation.m_z, rotation.m_w);

	FTransform transform;
	transform.SetRotation(uRot);
	transform.SetLocation(uPosit);
	return transform;
}

ndMatrix ToNewtonMatrix(const FTransform& tranform)
{
	const FVector location(tranform.GetLocation());
	const FQuat rotation(tranform.Rotator().Quaternion());

	const ndQuaternion quat(ndFloat32(rotation.X), ndFloat32(rotation.Y), ndFloat32(rotation.Z), ndFloat32(rotation.W));
	const ndVector posit(UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.X), UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.Y), UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.Z), ndFloat32(1.0f));
	const ndMatrix matrix(ndCalculateMatrix(quat, posit));
	return matrix;
}
