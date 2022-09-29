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
        super.SetMatrix(matrix.CreateNative());
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
