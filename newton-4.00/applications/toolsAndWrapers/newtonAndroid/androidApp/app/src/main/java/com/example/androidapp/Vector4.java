package com.example.androidapp;

import com.javaNewton.nVector;

public class Vector4
{
    Vector4 ()
    {
        m_data = new float[4];
    }

    Vector4 (float x, float y, float z, float w)
    {
        m_data = new float[4];
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
        m_data[3] = w;
    }

    Vector4 (nVector v)
    {
        m_data = new float[4];
        m_data[0] = v.Get(0);
        m_data[1] = v.Get(1);
        m_data[2] = v.Get(2);
        m_data[3] = v.Get(3);
    }

    public float[] m_data;
}
