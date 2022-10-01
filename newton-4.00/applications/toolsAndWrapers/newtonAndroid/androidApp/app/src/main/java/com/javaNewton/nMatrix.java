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

import com.newton.ndMatrixGlue;
import com.newton.ndVectorGlue;

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
        }
        Set (matrix);
    }

    public nMatrix(nMatrix matrix)
    {
        m_data = new nVector[4];
        for (int i = 0; i < 4; i++)
        {
            m_data[i] = new nVector(m_data[i]);
        }
        Set (matrix);
    }

    public void Set (ndMatrixGlue matrix)
    {
        for (int i = 0; i < 4; i++)
        {
            m_data[i].Set(matrix.Get(i));
        }
    }

    public void Set (nMatrix matrix)
    {
        for (int i = 0; i < 4; i++)
        {
            m_data[i].Set(matrix.m_data[i]);
        }
    }

    public void SetRow (int i, nVector v)
    {
        m_data[i].Set(v);
    }

    public void SetPosition (nVector posit)
    {
        SetRow (3, posit);
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

    public ndMatrixGlue CreateNative()
    {
        return new ndMatrixGlue (m_data[0].CreateNative(), m_data[1].CreateNative(), m_data[2].CreateNative(), m_data[3].CreateNative());
    }

    public nVector[] m_data;
}
