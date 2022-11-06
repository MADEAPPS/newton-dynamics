/* Copyright (c) <2003-2022> <Newton Game Dynamics>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely
 */

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

    public void MaterialBegin()
    {
        m_nativeObject.MaterialBegin();
    }

    public int GetFirstMaterial()
    {
        return m_nativeObject.GetFirstMaterial();
    }

    public int GetNextMaterial(int currentMaterialIndex)
    {
        return m_nativeObject.GetNextMaterial(currentMaterialIndex);
    }

    public int GetMaterialIndexCount(int materialIndex)
    {
        return m_nativeObject.GetMaterialIndexCount(materialIndex);
    }

    public void GetMaterialGetIndexStream(int materialIndex, short[] data, int startOffsetInShorts)
    {
        m_nativeObject.GetMaterialGetIndexStream(materialIndex, data, startOffsetInShorts);
    }

    public void MaterialEnd()
    {
        m_nativeObject.MaterialEnd();
    }

    public void SphericalMapping(int materialId, nMatrix matrix)
    {
        m_nativeObject.SphericalMapping(materialId, matrix.CreateNative());
    }

    public void UniformBoxMapping(int materialId, nMatrix matrix)
    {
        m_nativeObject.UniformBoxMapping(materialId, matrix.CreateNative());
    }

    private ndMeshEffectGlue m_nativeObject;
}
