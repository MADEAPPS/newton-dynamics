package com.javaNewton;

import com.newton.ndShapeBoxGlue;

public class nShapeBox extends nShape
{
    public nShapeBox(float size_x, float size_y, float size_z)
    {
        super(new ndShapeBoxGlue(size_x, size_y, size_z));
    }
}
