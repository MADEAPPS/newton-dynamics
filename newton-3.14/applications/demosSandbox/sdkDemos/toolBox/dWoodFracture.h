#ifndef AFX_WOOD_FRACTURE_H___H
#define AFX_WOOD_FRACTURE_H___H

#include "toolbox_stdafx.h"
#include <dVector.h>
#include <dMatrix.h>
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

void AddFracturedWoodPrimitive(DemoEntityManager* const scene, dFloat density, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, int stype, int materialID, const dMatrix& shapeOffsetMatrix);

#endif
