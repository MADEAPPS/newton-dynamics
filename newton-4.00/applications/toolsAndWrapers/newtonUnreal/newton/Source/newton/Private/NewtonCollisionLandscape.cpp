// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionLandscape.h"

#include "Landscape.h"
#include "Landscape.h"
#include "LandscapeProxy.h"
#include "Chaos/HeightField.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionLandscape::UNewtonCollisionLandscape()
	:Super()
{
	m_scale_x = 0.0f;
	m_scale_y = 0.0f;
	m_scale_z = 0.0f;
	m_tileSize_x = 32;
	m_tileSize_y = 32;
}

void UNewtonCollisionLandscape::Serialize(FArchive& ar)
{
	Super::Serialize(ar);

	ar << m_scale_x;
	ar << m_scale_y;
	ar << m_scale_z;
	ar << m_tileSize_x;
	ar << m_tileSize_y;

	ar << m_materials;
	ar << m_heightfield;
}

void UNewtonCollisionLandscape::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	SetTransform(meshComponent);

	const ULandscapeHeightfieldCollisionComponent* const tile = Cast<ULandscapeHeightfieldCollisionComponent>(meshComponent);
	check(tile);
	
	const ULandscapeHeightfieldCollisionComponent::FHeightfieldGeometryRef* const heightfieldRef = tile->HeightfieldRef;
	const Chaos::FHeightField* const mapData = heightfieldRef->HeightfieldGeometry;
	check(mapData);
	const Chaos::FHeightField::FData<uint16>& elevationData = mapData->GeomData;
	
	const Chaos::FVec3 elevationScale(elevationData.Scale);
	// this is not a bug, y and z are swapped after the mapping
	ndFloat32 xScale = elevationScale[0] * UNREAL_INV_UNIT_SYSTEM;
	ndFloat32 zScale = elevationScale[1] * UNREAL_INV_UNIT_SYSTEM;
	
	ndShapeHeightfield* const heightfield = new ndShapeHeightfield(
		ndInt32(mapData->GetNumCols()), ndInt32(mapData->GetNumRows()),
		ndShapeHeightfield::m_normalDiagonals, xScale, zScale);
	
	ndReal minValue = elevationData.MinValue;
	ndReal highScale = elevationData.HeightPerUnit;
	
	const ndShape* const shape = heightfield;
	const ndShapeInfo info(shape->GetShapeInfo());
	ndInt8* const attributes = info.m_heightfield.m_atributes;
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
			attributes[dstRow + colum] = 0;
		}
		srcRow -= elevationData.NumCols;
		dstRow += elevationData.NumCols;
	}
	
	m_scale_z = 1.0f;
	m_scale_x = info.m_heightfield.m_horizonalScale_x;
	m_scale_y = info.m_heightfield.m_horizonalScale_z;
	m_tileSize_x = info.m_heightfield.m_width;
	m_tileSize_y = info.m_heightfield.m_height;
	
	ndInt32 size = info.m_heightfield.m_width * info.m_heightfield.m_height;
	for (ndInt32 i = 0; i < size; ++i)
	{
		m_heightfield.Push(dstHeigh[i]);
		m_materials.Push(attributes[i]);
	}
	
	delete heightfield;
}

long long UNewtonCollisionLandscape::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeHeightfield::StaticClassName(), strlen(ndShapeHeightfield::StaticClassName()), 0);

	if (m_heightfield.Num())
	{
		const char* const materialBuffer = &m_materials[0];
		const float* const elevations = &m_heightfield[0];
		hash = ndCRC64(materialBuffer, m_materials.Num() * sizeof(char), hash);
		hash = ndCRC64(elevations, m_heightfield.Num() * sizeof(float), hash);
	}
	return hash;
}

void UNewtonCollisionLandscape::ApplyPropertyChanges()
{
	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}

ndShape* UNewtonCollisionLandscape::CreateShape() const
{
	if (!m_heightfield.Num())
	{
		return new ndShapeNull();
	}
	
	ndShapeHeightfield* const shape = new ndShapeHeightfield(
		m_tileSize_x, m_tileSize_y,
		ndShapeHeightfield::m_normalDiagonals, m_scale_x, m_scale_z);

	const ndShapeInfo info(((ndShape*)shape)->GetShapeInfo());
	ndInt8* const attributes = info.m_heightfield.m_atributes;
	ndReal* const dstHeigh = info.m_heightfield.m_elevation;

	ndInt32 size = m_tileSize_x * m_tileSize_y;
	for (ndInt32 i = 0; i < size; ++i)
	{
		dstHeigh[i] = ndReal(m_heightfield[i]);
		attributes[i] = ndInt8(m_materials[i]);
	}
	shape->UpdateElevationMapAabb();
	return shape;
}

ndShapeInstance* UNewtonCollisionLandscape::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	const FVector uScale(GetComponentToWorld().GetScale3D());
	ndMatrix origin(ndGetIdentityMatrix());
	//origin.m_posit.m_x = ndFloat32(tile->SectionBaseX * tile->CollisionScale * uScale.X * UNREAL_INV_UNIT_SYSTEM);
	origin.m_posit.m_y = ndFloat32(ndFloat32(m_tileSize_y - 1) * m_scale_y * uScale.Y * UNREAL_INV_UNIT_SYSTEM);
	
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
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix (ToNewtonMatrix(transform));
	const FVector uScale(transform.GetScale3D());
	ndMatrix origin(ndGetIdentityMatrix());
	origin.m_posit.m_y = ndFloat32(ndFloat32(m_tileSize_y - 1) * m_scale_y * uScale.Y * UNREAL_INV_UNIT_SYSTEM);
	
	const ndMatrix aligment(ndPitchMatrix(ndPi * 0.5f));
	const ndMatrix tileMatrix(aligment * origin * matrix * bodyMatrix);
	instance->SetLocalMatrix(tileMatrix);
	
	return instance;
}
