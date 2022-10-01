package com.javaNewton;

import com.newton.ndMeshEffect;

public class nMeshEffect
{
    public nMeshEffect()
    {
        m_nativeObject = new ndMeshEffect();
    }

    public nMeshEffect(nShapeInstance shapeInstance)
    {
        m_nativeObject = new ndMeshEffect(shapeInstance.GetNativeObject());
    }

    private ndMeshEffect m_nativeObject;
}
