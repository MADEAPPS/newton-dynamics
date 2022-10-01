package com.javaNewton;

import com.newton.ndMeshEffectGlue;

public class nMeshEffect
{
    public nMeshEffect()
    {
        m_nativeObject = new ndMeshEffectGlue();
    }

    public nMeshEffect(nShapeInstance shapeInstance)
    {
        m_nativeObject = new ndMeshEffectGlue(shapeInstance.GetNativeObject());
    }

    public int GetVertextCount()
    {
        return m_nativeObject.GetVertexSize();
    }

    private ndMeshEffectGlue m_nativeObject;
}
