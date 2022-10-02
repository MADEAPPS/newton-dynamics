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

import com.newton.ndVectorGlue;

public class nVector
{
    public nVector()
    {
        m_data = new float[4];
    }

    public nVector(float x, float y, float z, float w)
    {
        m_data = new float[4];
        Set(x, y, z, w);
    }

    public nVector (nVector v)
    {
        m_data = new float[4];
        Set(v);
    }

    public nVector (ndVectorGlue v)
    {
        m_data = new float[4];
        Set(v);
    }

    public void Set(float x, float y, float z, float w)
    {
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
        m_data[3] = w;
    }

    public void Set(ndVectorGlue v)
    {
        m_data[0] = v.Get(0);
        m_data[1] = v.Get(1);
        m_data[2] = v.Get(2);
        m_data[3] = v.Get(3);
    }

    public void Set(nVector v)
    {
        m_data[0] = v.m_data[0];
        m_data[1] = v.m_data[1];
        m_data[2] = v.m_data[2];
        m_data[3] = v.m_data[3];
    }

    public ndVectorGlue CreateNative()
    {
        return new ndVectorGlue(m_data[0], m_data[1], m_data[2], m_data[3]);
    }

    public nVector Scale (float s)
    {
        return  new nVector(m_data[0] * s, m_data[1] * s, m_data[2] * s, m_data[3] * s);
    }

    public nVector MulScale (nVector v, float s)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[0] + v.m_data[0] * s;
        tmp.m_data[1] = m_data[1] + v.m_data[1] * s;
        tmp.m_data[2] = m_data[2] + v.m_data[2] * s;
        tmp.m_data[3] = m_data[3] + v.m_data[3] * s;
        return tmp;
    }

    public nVector Add (nVector v)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[0] + v.m_data[0];
        tmp.m_data[1] = m_data[1] + v.m_data[1];
        tmp.m_data[2] = m_data[2] + v.m_data[2];
        tmp.m_data[3] = m_data[3] + v.m_data[3];
        return tmp;
    }

    public nVector Sub (nVector v)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[0] - v.m_data[0];
        tmp.m_data[1] = m_data[1] - v.m_data[1];
        tmp.m_data[2] = m_data[2] - v.m_data[2];
        tmp.m_data[3] = m_data[3] - v.m_data[3];
        return tmp;
    }

    public nVector Mul (nVector v)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[0] * v.m_data[0];
        tmp.m_data[1] = m_data[1] * v.m_data[1];
        tmp.m_data[2] = m_data[2] * v.m_data[2];
        tmp.m_data[3] = m_data[3] * v.m_data[3];
        return tmp;
    }

    public nVector MulAdd (nVector v, nVector w)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[0] + v.m_data[0] * w.m_data[0];
        tmp.m_data[1] = m_data[1] + v.m_data[1] * w.m_data[1];
        tmp.m_data[2] = m_data[2] + v.m_data[2] * w.m_data[2];
        tmp.m_data[3] = m_data[3] + v.m_data[3] * w.m_data[3];
        return tmp;
    }

    public float DotProduct (nVector v)
    {
        return m_data[0] *  v.m_data[0] + m_data[1] *  v.m_data[1] + m_data[2] *  v.m_data[2] + m_data[3] *  v.m_data[3];
    }

    public nVector CrossProduct (nVector v)
    {
        nVector tmp = new nVector();
        tmp.m_data[0] = m_data[1] * v.m_data[2] - m_data[2] * v.m_data[1];
        tmp.m_data[1] = m_data[2] * v.m_data[0] - m_data[0] * v.m_data[2];
        tmp.m_data[2] = m_data[0] * v.m_data[1] - m_data[1] * v.m_data[0];
        tmp.m_data[3] = m_data[3];
        return tmp;
    }

    public nVector Normalize ()
    {
        nVector tmp = new nVector();
        float invMag = 1.0f / (float)Math.sqrt(DotProduct (this));
        tmp.m_data[0] = m_data[0] * invMag;
        tmp.m_data[1] = m_data[1] * invMag;
        tmp.m_data[2] = m_data[2] * invMag;
        tmp.m_data[3] = m_data[3] * invMag;
        return tmp;
    }

    public float[] m_data;
}
