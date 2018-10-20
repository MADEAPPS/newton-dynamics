/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbximplementationutils.h
#ifndef _FBXSDK_SCENE_SHADING_IMPLEMENTATION_UTILS_H_
#define _FBXSDK_SCENE_SHADING_IMPLEMENTATION_UTILS_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/scene/shading/fbximplementation.h>
#include <fbxsdk/scene/shading/fbxbindingoperator.h>
#include <fbxsdk/scene/shading/fbxoperatorentryview.h>
#include <fbxsdk/scene/shading/fbxpropertyentryview.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/** Get FbxImplementation from FbxObject.
  * \param pObject                  FbxObject to get FbxImplementation.
  * \param pImplementationTarget    Name of the implementation property to get.
  * \return FbxImplementation      Pointer to FbxImplementation.
  */
FBXSDK_DLL FbxImplementation const* GetImplementation( FbxObject const* pObject, char const* pImplementationTarget );

/** Get bound property value from FbxBindingTable.
  * \param pBindingTable            FbxBindingTable to get bound property value.
  * \param pEntryName               Name of the Entry type to get.
  * \param pImplementation          FbxImplementation of the bound property value to get if the Entry type is FbxOperatorEntryView::sEntryType.
  * \param pBoundObject             FbxObject of the bound property value to get if the Entry type is FbxPropertyEntryView::sEntryType.
  * \param pValue                   Pointer to bound property value from FbxBindingTable.
  * \return Whether get bound property value success or not.
  */
template <typename T> bool GetBoundPropertyValue(FbxBindingTable const* pBindingTable,
                                                 char const* pEntryName, 
                                                 FbxImplementation const* pImplementation,
                                                 FbxObject const* pBoundObject,
                                                 T& pValue)
{
    if ((NULL != pImplementation) && (NULL != pBindingTable) && (NULL != pBoundObject) && (NULL != pEntryName))
    {
        FbxBindingTableEntry const* lEntry = pBindingTable->GetEntryForDestination(pEntryName);

        if (NULL != lEntry)
        {
            if (strcmp(lEntry->GetEntryType(true), FbxPropertyEntryView::sEntryType) == 0)
            {
                char const* lPropName = lEntry->GetSource();
                FbxProperty lProp = pBoundObject->FindPropertyHierarchical(lPropName);
                if (lProp.IsValid())
                {
					pValue = lProp.Get<T>();
                    return true;
                }
            }
            else if (strcmp(lEntry->GetEntryType(true), FbxOperatorEntryView::sEntryType) == 0)
            {
                char const* lOperatorName = lEntry->GetSource();
                FbxBindingOperator const* lOp = pImplementation->GetOperatorByTargetName(lOperatorName);
                if (lOp)
                {
                    return lOp->Evaluate(pBoundObject, &pValue);
                }
            }
        }
    }

    return false;
}

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_SCENE_SHADING_IMPLEMENTATION_UTILS_H_ */
