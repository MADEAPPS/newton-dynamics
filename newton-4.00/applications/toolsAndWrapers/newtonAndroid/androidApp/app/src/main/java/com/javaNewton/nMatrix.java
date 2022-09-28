package com.javaNewton;


import com.newton.ndMatrixGlue;

public class nMatrix extends ndMatrixGlue
{
    public nVector Get(int i)
    {
        return new nVector(super.Get(i));
    }
}
