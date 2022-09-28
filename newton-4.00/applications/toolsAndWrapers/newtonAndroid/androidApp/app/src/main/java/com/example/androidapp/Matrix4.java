package com.example.androidapp;

public class Matrix4
{
    Matrix4()
    {
        m_data = new Vector4[4];
        for (int i = 0; i < 4; i ++)
        {
            for (int j = 0; j < 4; j ++)
            {
                m_data[i].m_data[j] = 0.0f;
            }
            m_data[i].m_data[i] = 1.0f;
        }
    }

    Matrix4 Mul(Matrix4 other)
    {
        Matrix4 matrix = new Matrix4();

        for (int i = 0; i < 4; i ++)
        {
            for (int j = 0; j < 4; j ++)
            {
                float a = 0.0f;
                for (int k = 0; k < 4; k ++)
                {
                    a = a + m_data[i].m_data[k] * other.m_data[k].m_data[j];
                }
                matrix.m_data[i].m_data[j] = a;
            }
        }
        return matrix;
    }

    public Vector4[] m_data;
}
