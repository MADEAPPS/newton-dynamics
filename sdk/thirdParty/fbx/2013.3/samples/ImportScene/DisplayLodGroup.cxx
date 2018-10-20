/****************************************************************************************

   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.

   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.

****************************************************************************************/

#include <fbxsdk.h>

#include "DisplayCommon.h"

void DisplayLodGroup(FbxNode* pNode)
{
    const char* lDisplayLevels[] = { "UseLOD", "Show", "Hide" }; 

    DisplayString("LodGroup Name: ", (char *) pNode->GetName());

    DisplayInt("    ", pNode->GetChildCount(), " Geometries");
    for (int i = 0; i < pNode->GetChildCount(); i++)
    {
        FbxNode* lChildNode = pNode->GetChild(i);
        DisplayString("        ", lChildNode->GetName());
    }

    FbxLODGroup *lLodGroupAttr = (FbxLODGroup*)pNode->GetNodeAttribute();
    DisplayBool("    MinMaxDistance Enabled: ", lLodGroupAttr->MinMaxDistance.Get());
    if (lLodGroupAttr->MinMaxDistance.Get())
    {
        DisplayDouble("        Min Distance: ", lLodGroupAttr->MinDistance.Get());
        DisplayDouble("        Max Distance: ", lLodGroupAttr->MaxDistance.Get());
    }
    DisplayBool("    Is World Space: ", lLodGroupAttr->WorldSpace.Get());

    DisplayString("    Thresholds ");
    for (int i = 0; i < lLodGroupAttr->GetNumThresholds(); i++)
    {
        FbxDistance lThreshVal;
        if (lLodGroupAttr->GetThreshold(i, lThreshVal))
            DisplayDouble("        ", lThreshVal.value());
    }

    DisplayString("    DisplayLevels");
    for (int i = 0; i < lLodGroupAttr->GetNumDisplayLevels(); i++)
    {
        FbxLODGroup::EDisplayLevel lLevel;
        if (lLodGroupAttr->GetDisplayLevel(i, lLevel))
            DisplayString("        ", lDisplayLevels[lLevel]);
    }
}
