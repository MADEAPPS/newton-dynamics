package com.example.androidapp;

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

    public float[] m_data;
}
