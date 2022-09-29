package com.javaNewton;

import android.util.Log;

import com.newton.ndVectorGlue;
import com.newton.ndMatrixGlue;
import com.newton.ndBodyNotifyGlue;

public class nBodyNotify extends ndBodyNotifyGlue
{
    public nBodyNotify()
    {
        super();
    }

    public void OnTransform(ndMatrixGlue matrix)
    {
        Log.i("ndNewton", "OnTransform!!!");
    }

    public void SetGravity(nVector v)
    {
        ndVectorGlue gravity = new ndVectorGlue(v.m_data[0], v.m_data[1], v.m_data[2], v.m_data[3]);
        super.SetGravity(gravity);
    }
}
