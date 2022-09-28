package com.javaNewton;

import com.newton.ndVectorGlue;

public class nVector extends ndVectorGlue
{
    public nVector(float x, float y, float z, float w)
    {
        super(x, y, z, w);
    }

    nVector (ndVectorGlue v)
    {
        super(v);
    }

}
