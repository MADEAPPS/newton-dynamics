package com.javaNewton;


import com.newton.ndShapeGlue;
import com.newton.nRigidBodyType;
import com.newton.ndRigidBodyGlue;
import com.newton.ndBodyNotifyGlue;
import com.newton.ndShapeInstanceGlue;
import com.newton.ndVectorGlue;
import com.newton.ndMatrixGlue;

public class nRigidBody extends ndRigidBodyGlue
{
    private ndBodyNotifyGlue m_notify;
    private ndShapeInstanceGlue m_shapeInstance;

    public nRigidBody(nRigidBodyType type)
    {
        super(type);
    }

    public void SetMatrix(nMatrix matrix)
    {
        ndVectorGlue front = new ndVectorGlue(matrix.m_data[0].m_data[0], matrix.m_data[0].m_data[1], matrix.m_data[0].m_data[2], matrix.m_data[0].m_data[3]);
        ndVectorGlue up    = new ndVectorGlue(matrix.m_data[1].m_data[0], matrix.m_data[1].m_data[1], matrix.m_data[1].m_data[2], matrix.m_data[1].m_data[3]);
        ndVectorGlue right = new ndVectorGlue(matrix.m_data[2].m_data[0], matrix.m_data[2].m_data[1], matrix.m_data[2].m_data[2], matrix.m_data[2].m_data[3]);
        ndVectorGlue posit = new ndVectorGlue(matrix.m_data[3].m_data[0], matrix.m_data[3].m_data[1], matrix.m_data[3].m_data[2], matrix.m_data[3].m_data[3]);
        super.SetMatrix(new ndMatrixGlue(front, up, right, posit));
    }

    @Override
    public void SetNotifyCallback(ndBodyNotifyGlue notifyCallBack)
    {
        m_notify = notifyCallBack;
        super.SetNotifyCallback(m_notify);
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
}
