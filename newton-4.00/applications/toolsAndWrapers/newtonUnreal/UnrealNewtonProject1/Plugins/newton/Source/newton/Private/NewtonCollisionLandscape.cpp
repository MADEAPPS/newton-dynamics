// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionLandscape.h"

#include "Landscape.h"
#include "LandscapeProxy.h"
#include "Chaos/HeightField.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionLandscape::UNewtonCollisionLandscape()
	:Super()
{
	//SetMobility(EComponentMobility::Static);
}

long long UNewtonCollisionLandscape::CalculateHash() const
{
	const ULandscapeHeightfieldCollisionComponent* const tile = Cast<ULandscapeHeightfieldCollisionComponent>(GetGeometryMesh().Get());
	check(tile);

	const ULandscapeHeightfieldCollisionComponent::FHeightfieldGeometryRef* const heightfieldRef = tile->HeightfieldRef;
	const Chaos::FHeightField* const mapData = heightfieldRef->HeightfieldGeometry;
	const Chaos::FHeightField::FData<uint16>& elevationData = mapData->GeomData;

	const Chaos::FVec3 elevationScale(elevationData.Scale);
	ndFloat32 xScale = elevationScale[0] * UNREAL_INV_UNIT_SYSTEM;
	// this is not a bug, y and z are swapped after the mapping
	ndFloat32 zScale = elevationScale[1] * UNREAL_INV_UNIT_SYSTEM;
	ndFloat32 yScale = elevationScale[2] * UNREAL_INV_UNIT_SYSTEM;

	long long hash = ndCRC64(ndShapeHeightfield::StaticClassName(), strlen(ndShapeHeightfield::StaticClassName()), 0);
	hash = ndCRC64(&xScale, sizeof(float), hash);
	hash = ndCRC64(&yScale, sizeof(float), hash);
	hash = ndCRC64(&zScale, sizeof(float), hash);
	hash = ndCRC64(&elevationData.Heights[0], sizeof(uint16) * elevationData.Heights.Num(), hash);
	return hash;
}

void UNewtonCollisionLandscape::ApplyPropertyChanges()
{
	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}

ndShape* UNewtonCollisionLandscape::CreateShape() const
{
	const ULandscapeHeightfieldCollisionComponent* const tile = Cast<ULandscapeHeightfieldCollisionComponent>(GetGeometryMesh().Get());
	check(tile);

	const ULandscapeHeightfieldCollisionComponent::FHeightfieldGeometryRef* const heightfieldRef = tile->HeightfieldRef;
	const Chaos::FHeightField* const mapData = heightfieldRef->HeightfieldGeometry;
	check(mapData);
	const Chaos::FHeightField::FData<uint16>& elevationData = mapData->GeomData;

	const Chaos::FVec3 elevationScale(elevationData.Scale);
	// this is not a bug, y and z are swapped after the mapping
	ndFloat32 xScale = elevationScale[0] * UNREAL_INV_UNIT_SYSTEM;
	ndFloat32 zScale = elevationScale[1] * UNREAL_INV_UNIT_SYSTEM;

	ndShapeHeightfield* const shape = new ndShapeHeightfield(
		ndInt32(mapData->GetNumCols()), ndInt32(mapData->GetNumRows()),
		ndShapeHeightfield::m_normalDiagonals, xScale, zScale);

	ndReal minValue = elevationData.MinValue;
	ndReal highScale = elevationData.HeightPerUnit;

	const ndShapeInfo info(((ndShape*)shape)->GetShapeInfo());
	ndReal* const dstHeigh = info.m_heightfield.m_elevation;
	ndReal yScale = ndReal(elevationScale[2] * UNREAL_INV_UNIT_SYSTEM);

	ndInt32 dstRow = 0;
	ndInt32 srcRow = (ndInt32(elevationData.NumRows) - 1) * elevationData.NumCols;
	for (ndInt32 row = ndInt32(elevationData.NumRows) - 1; row >= 0; --row)
	{
		for (ndInt32 colum = elevationData.NumCols - 1; colum >= 0; --colum)
		{
			ndReal h = minValue + ndReal(elevationData.Heights[srcRow + colum]) * highScale;
			dstHeigh[dstRow + colum] = h * yScale;
		}
		srcRow -= elevationData.NumCols;
		dstRow += elevationData.NumCols;
	}

	shape->UpdateElevationMapAabb();
	return shape;
}

ndShapeInstance* UNewtonCollisionLandscape::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	ULandscapeHeightfieldCollisionComponent* const tile = Cast<ULandscapeHeightfieldCollisionComponent>(GetGeometryMesh().Get());
	check(tile);
	const FVector uScale(GetComponentToWorld().GetScale3D());
	ndMatrix origin(ndGetIdentityMatrix());
	//origin.m_posit.m_x = ndFloat32(tile->SectionBaseX * tile->CollisionScale * uScale.X * UNREAL_INV_UNIT_SYSTEM);
	origin.m_posit.m_y = ndFloat32(tile->CollisionSizeQuads * tile->CollisionScale * uScale.Y * UNREAL_INV_UNIT_SYSTEM);
	
	const ndMatrix aligment(ndPitchMatrix(ndPi * 0.5f));
	const ndMatrix tileMatrix(aligment * origin);
	instance->SetLocalMatrix(tileMatrix);

	ndMatrix scaleMatrix(ndGetIdentityMatrix());
	scaleMatrix[0][0] = ndFloat32(1.0f / uScale.X);
	scaleMatrix[1][1] = ndFloat32(1.0f / uScale.Y);
	scaleMatrix[2][2] = ndFloat32(1.0f / uScale.Z);
	const ndMatrix instanceMatrix(tileMatrix * scaleMatrix * tileMatrix.OrthoInverse());
	instance->SetGlobalScale(instanceMatrix);
	return instance;
}

ndShapeInstance* UNewtonCollisionLandscape::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	ULandscapeHeightfieldCollisionComponent* const tile = Cast<ULandscapeHeightfieldCollisionComponent>(GetGeometryMesh().Get());
	check(tile);
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix (UNewtonRigidBody::ToNewtonMatrix(transform));
	const FVector uScale(transform.GetScale3D());
	ndMatrix origin(ndGetIdentityMatrix());
	//origin.m_posit.m_x = ndFloat32(tile->SectionBaseX * tile->CollisionScale * uScale.X);
	origin.m_posit.m_y = ndFloat32(tile->CollisionSizeQuads * tile->CollisionScale * uScale.Y * UNREAL_INV_UNIT_SYSTEM);

	const ndMatrix aligment(ndPitchMatrix(ndPi * 0.5f));
	const ndMatrix tileMatrix(aligment * origin * matrix * bodyMatrix);
	instance->SetLocalMatrix(tileMatrix);

	//ndMatrix scaleMatrix(ndGetIdentityMatrix());
	//scaleMatrix[0][0] = ndFloat32(1.0f / uScale.X);
	//scaleMatrix[1][1] = ndFloat32(1.0f / uScale.Y);
	//scaleMatrix[2][2] = ndFloat32(1.0f / uScale.Z);
	//const ndMatrix instanceMatrix(tileMatrix * scaleMatrix * tileMatrix.OrthoInverse());
	//instance->SetGlobalScale(instanceMatrix);
	return instance;
}
