package com.javaNewton;


import com.newton.ndMatrixGlue;

public class nMatrix
{
    public nMatrix()
    {
        m_data = new nVector[4];
        for (int i = 0; i < 4; i ++)
        {
            m_data[i] = new nVector();
            for (int j = 0; j < 4; j ++)
            {
                m_data[i].m_data[j] = 0.0f;
            }
            m_data[i].m_data[i] = 1.0f;
        }
    }

    public nMatrix(ndMatrixGlue matrix)
    {
        m_data = new nVector[4];
        for (int i = 0; i < 4; i++)
        {
            m_data[i] = new nVector();
            nVector row = new nVector(matrix.Get(i));
            for (int j = 0; j < 4; j++)
            {
                m_data[i].m_data[j] = row.m_data[j];
            }
        }
    }

    public nMatrix(nMatrix matrix)
    {
        m_data = new nVector[4];
        for (int i = 0; i < 4; i++)
        {
            m_data[i] = new nVector(m_data[i]);
        }
    }

    public void Set (int i, nVector v)
    {
        m_data[i].Set(v);
    }

    public nMatrix Mul(nMatrix other)
    {
        nMatrix matrix = new nMatrix();

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

    public nVector[] m_data;
}
