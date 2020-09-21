#ifndef AFX_WOOD_FRACTURE_H___H
#define AFX_WOOD_FRACTURE_H___H

#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"

void AddFracturedWoodPrimitive(ndDemoEntityManager* const scene, dFloat density, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, int stype, int materialID, const dMatrix& shapeOffsetMatrix);

#endif
