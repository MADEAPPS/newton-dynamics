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

public class nBodyNotify
{
    public nBodyNotify()
    {
        m_nativeObject = new NativeBodyNotify(this);
    }

    public void OnTransform(nMatrix matrix)
    {
        Log.i("ndNewton", "OnTransform!!!");
    }

    public void SetGravity(nVector v)
    {
        m_nativeObject.SetGravity(new ndVectorGlue(v.m_data[0], v.m_data[1], v.m_data[2], v.m_data[3]));
    }

    public void SetBody(nRigidBody body)
    {
        m_nativeObject.SetBody(body);
    }

    private class NativeBodyNotify extends ndBodyNotifyGlue
    {
        public NativeBodyNotify(nBodyNotify owner)
        {
            super();
            m_owner = owner;
            m_cacheMatrix = new nMatrix();
        }

        public void OnTransform(ndMatrixGlue matrix)
        {
            m_cacheMatrix.Set(matrix);
            m_owner.OnTransform(m_cacheMatrix);
        }

        public void SetBody(nRigidBody body)
        {
            body.GetNativeObject().SetNotifyCallback(this);
        }

        private nMatrix m_cacheMatrix;
    }

    public NativeBodyNotify GetNativeObject()
    {
        return m_nativeObject;
    }

    private nBodyNotify m_owner;
    private NativeBodyNotify m_nativeObject;
}
