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

package com.example.androidapp;

import com.javaNewton.nVector;

public class SceneMeshSegment
{
    public SceneMeshSegment(int indexOffset, int indexCount)
    {
        m_indexCount = indexCount;
        m_indexOffset = indexOffset;
        m_ambient = new nVector(0.8f, 0.8f, 0.8f, 1.0f);
        m_diffuse = new nVector(0.8f, 0.8f, 0.8f, 1.0f);
        m_specular = new nVector(1.0f, 1.0f, 1.0f, 1.0f);
        m_opacity = 1.0f;
        m_shiness = 100.0f;
        m_texture = null;
    }

    public nVector m_ambient;
    public nVector m_diffuse;
    public nVector m_specular;
    public float m_opacity;
    public float m_shiness;
    public SceneMeshTexture m_texture;

    public int m_indexCount;
    public int m_indexOffset;
}
