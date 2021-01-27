#ifndef __D_WOOD_FRACTURE_H__
#define __D_WOOD_FRACTURE_H__

#include "ndSandboxStdafx.h"

class ndDemoEntityManager;
void AddFracturedWoodPrimitive(ndDemoEntityManager* const scene, dFloat32 density, const dVector& origin, const dVector& size, dInt32 xCount, dInt32 zCount, dFloat32 spacing, dInt32 type, dInt32 materialID, const dMatrix& shapeOffsetMatrix);

#endif
