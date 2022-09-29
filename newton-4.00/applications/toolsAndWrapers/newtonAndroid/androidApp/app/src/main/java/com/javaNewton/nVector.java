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
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
        m_data[3] = w;
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

    public float[] m_data;
}
