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

    public void GetVertexPosit(float[] buffer, int startOffsetInFloats, int strideInFloats)
    {
        m_nativeObject.GetVertexPosit(buffer, startOffsetInFloats, strideInFloats);
    }

    public void GetVertexNormal(float[] buffer, int startOffsetInFloats, int strideInFloats)
    {
        m_nativeObject.GetVertexNormal(buffer, startOffsetInFloats, strideInFloats);
    }

    public void GetVertexUV0(float[] buffer, int startOffsetInFloats, int strideInFloats)
    {
        m_nativeObject.GetVertexUV0(buffer, startOffsetInFloats, strideInFloats);
    }

    private ndMeshEffectGlue m_nativeObject;
}
