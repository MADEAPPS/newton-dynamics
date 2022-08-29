#pragma once
#include "bvhMatrix.h"

class BvhNode;
bool ExportFbx(const BvhNode* scene, const char* const name);
