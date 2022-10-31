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
        }
        SetIdentity ();
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

    public void SetIdentity ()
    {
        for (int i = 0; i < 4; i ++)
        {
            for (int j = 0; j < 4; j ++)
            {
                m_data[i].m_data[j] = 0.0f;
            }
            m_data[i].m_data[i] = 1.0f;
        }
    }

    public void Set (float[] data)
    {
        for (int i = 0; i < 4; i++)
        {
            int base = i * 4;
            m_data[i].Set(data[base + 0], data[base + 1], data[base + 2], data[base + 3]);
        }
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

    public nVector GetRow (int i)
    {
        return m_data[i];
    }

    public void SetRow (int i, nVector v)
    {
        m_data[i].Set(v);
    }

    public void SetPosition (nVector posit)
    {
        SetRow (3, posit);
    }

    public nVector GetPosition ()
    {
        return GetRow(3);
    }

    public ndMatrixGlue CreateNative()
    {
        return new ndMatrixGlue (m_data[0].CreateNative(), m_data[1].CreateNative(), m_data[2].CreateNative(), m_data[3].CreateNative());
    }

    public void GetFlatArray(float[] data)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                data[i * 4 + j] = m_data[i].m_data[j];
            }
        }
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

    public nMatrix Inverxe()
    {
        nMatrix matrix = new nMatrix();
        for (int i = 0; i < 3; i ++)
        {
            for (int j = 0; j < 3; j ++)
            {
                matrix.m_data[i].m_data[j] = m_data[j].m_data[i];
            }
            matrix.m_data[3].m_data[i] = -m_data[i].DotProduct(m_data[3]);
        }
        return matrix;
    }

    public nVector RotateVector(nVector v)
    {
        nVector tmp = new nVector(0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; i ++)
        {
            tmp = tmp.MulScale (m_data[i], v.m_data[i]);
        }
        tmp.m_data[3] = v.m_data[3];
        return tmp;
    }

    public nVector UnrotateVector(nVector v)
    {
        nVector tmp = new nVector(0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; i ++)
        {
            tmp.m_data[i] = m_data[i].DotProduct(v);
        }
        tmp.m_data[3] = v.m_data[3];
        return tmp;
    }

    public nVector TransformVector(nVector v)
    {
        nVector tmp = new nVector(m_data[3].Scale (v.m_data[3]));
        for (int i = 0; i < 3; i ++)
        {
            tmp = tmp.MulScale (m_data[i], v.m_data[i]);
        }
        return tmp;
    }

    public nVector UntransformVector(nVector v)
    {
        nVector x = v.Sub(m_data[3]);
        nVector tmp = UnrotateVector(x);
        tmp.m_data[3] = 1.0f;
        return tmp;
    }

    static public nMatrix PitchMatrix(float angle)
    {
        nMatrix matrix = new nMatrix();
        float cos = (float)Math.cos(angle);
        float sin = (float)Math.cos(angle);
        matrix.m_data[1].m_data[1] = cos;
        matrix.m_data[1].m_data[2] = sin;
        matrix.m_data[2].m_data[1] = -sin;
        matrix.m_data[2].m_data[2] = cos;
        return matrix;
    }

    static public nMatrix YawMatrix(float angle)
    {
        nMatrix matrix = new nMatrix();
        float cos = (float)Math.cos(angle);
        float sin = (float)Math.cos(angle);
        matrix.m_data[0].m_data[0] = cos;
        matrix.m_data[0].m_data[2] = -sin;
        matrix.m_data[2].m_data[0] = sin;
        matrix.m_data[2].m_data[2] = cos;
        return matrix;
    }

    static public nMatrix RollMatrix(float angle)
    {
        nMatrix matrix = new nMatrix();
        float cos = (float)Math.cos(angle);
        float sin = (float)Math.cos(angle);
        matrix.m_data[0].m_data[0] = cos;
        matrix.m_data[0].m_data[1] = sin;
        matrix.m_data[1].m_data[0] = -sin;
        matrix.m_data[1].m_data[1] = cos;
        return matrix;
    }

    public nVector[] m_data;
}
