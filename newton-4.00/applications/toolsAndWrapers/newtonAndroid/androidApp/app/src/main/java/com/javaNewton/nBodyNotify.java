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
        }

        public void OnTransform(ndMatrixGlue nativeMatrix)
        {
            m_owner.OnTransform(new nMatrix(nativeMatrix));
        }

        public void SetBody(nRigidBody body)
        {
            m_body = body;
            body.SetNotifyCallback(this);
        }

        private nRigidBody m_body;
    }

    private nBodyNotify m_owner;
    private NativeBodyNotify m_nativeObject;
}
