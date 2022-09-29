package com.javaNewton;


import com.newton.ndShapeGlue;
import com.newton.nRigidBodyType;
import com.newton.ndRigidBodyGlue;
import com.newton.ndShapeInstanceGlue;
import com.newton.ndVectorGlue;
import com.newton.ndMatrixGlue;

public class nRigidBody extends ndRigidBodyGlue
{
    public nRigidBody(nRigidBodyType type)
    {
        super(type);
        m_notify = null;
    }

    public void SetMatrix(nMatrix matrix)
    {
        ndVectorGlue front = new ndVectorGlue(matrix.m_data[0].m_data[0], matrix.m_data[0].m_data[1], matrix.m_data[0].m_data[2], matrix.m_data[0].m_data[3]);
        ndVectorGlue up    = new ndVectorGlue(matrix.m_data[1].m_data[0], matrix.m_data[1].m_data[1], matrix.m_data[1].m_data[2], matrix.m_data[1].m_data[3]);
        ndVectorGlue right = new ndVectorGlue(matrix.m_data[2].m_data[0], matrix.m_data[2].m_data[1], matrix.m_data[2].m_data[2], matrix.m_data[2].m_data[3]);
        ndVectorGlue posit = new ndVectorGlue(matrix.m_data[3].m_data[0], matrix.m_data[3].m_data[1], matrix.m_data[3].m_data[2], matrix.m_data[3].m_data[3]);
        super.SetMatrix(new ndMatrixGlue(front, up, right, posit));
    }

    public void SetNotify(nBodyNotify notify)
    {
        m_notify = notify;
        notify.SetBody(this);
    }

    @Override
    public void SetCollisionShape(ndShapeInstanceGlue shapeInstance)
    {
        super.SetCollisionShape(shapeInstance);
        m_shapeInstance = new nShapeInstance(new ndShapeGlue(GetShape()));
    }

    public ndShapeInstanceGlue GetCollisionShape()
    {
        return m_shapeInstance;
    }

    private nBodyNotify m_notify;
    private ndShapeInstanceGlue m_shapeInstance;
}
