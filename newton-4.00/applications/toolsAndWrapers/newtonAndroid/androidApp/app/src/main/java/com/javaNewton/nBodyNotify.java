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

import android.util.Log;

import com.newton.ndVectorGlue;
import com.newton.ndMatrixGlue;
import com.newton.ndBodyNotifyGlue;

public class nBodyNotify extends ndBodyNotifyGlue
{
    public nBodyNotify()
    {
        m_cacheMatrix = new nMatrix();
    }

    public void OnTransform(nMatrix matrix)
    {
        Log.i("ndNewton", "OnTransform!!!");
    }

    public void SetGravity(nVector v)
    {
        SetGravity(new ndVectorGlue(v.m_data[0], v.m_data[1], v.m_data[2], v.m_data[3]));
    }

    public void OnTransform(ndMatrixGlue matrix)
    {
        m_cacheMatrix.Set(matrix);
        OnTransform(m_cacheMatrix);
    }

    private nMatrix m_cacheMatrix;
}
