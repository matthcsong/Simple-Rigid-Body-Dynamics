#include "RigidBody.h"
#include <iostream>
#include <algorithm>

RigidBody::RigidBody(double mass, const Eigen::MatrixXd& V, double m)
{
    this->m_rVertices = V;
    this->m_mass = mass;
    this->m_aVelocity = {0.0, 0.0, 0.0};
    this->m_velocity = {0.0, 0.0, 0.0};
    this->ComputeMassCenter();
    this->m_force = {0, -10, 0};
    this->m_force = mass * this->m_force;
    this->m_quaternion = this->m_quaternion.Identity();
    this->ComputeReferenceInertia(m);
    this->m_torque = {0, 0, 0};
    this->ComputeInertia();
}

void RigidBody::EulerStep(double timestep)
{
    double restitution = 0.5;
    Eigen::Vector3d floor = {0, -0.15, 0};
    Eigen::Vector3d normal = {0, 1, 0};

    Eigen::Vector3d floor2 = {-1, 0, 0};
    Eigen::Vector3d normal2 = {1, 0, 0};
    // update velocity
    this->m_velocity = this->m_velocity + timestep * this->m_force / this->m_mass;
    
    // update angular velocity
    this->m_aVelocity = this->m_aVelocity + timestep * this->m_Inertia.inverse() * this->m_torque;

    this->ComputeCollisionAndResponse(floor, normal, timestep, restitution);
    this->ComputeCollisionAndResponse(floor2, normal2, timestep, restitution);
    restitution *= 0.5;

    // update mass center
    this->m_barycenter = this->m_barycenter + timestep * this->m_velocity;

    // update quaternion
    Eigen::Vector3d vec = timestep * this->m_aVelocity / 2;
    Eigen::Quaternion<double> tmp(0, vec(0), vec(1), vec(2));
    Eigen::Quaternion<double> tmp2 = tmp * this->m_quaternion;
    this->m_quaternion.w() = this->m_quaternion.w() + tmp2.w();
    this->m_quaternion.vec() = this->m_quaternion.vec() + tmp2.vec();
    std::cout << "Debug q---------------------" << std::endl;
    std::cout << this->m_quaternion << std::endl;
    this->m_quaternion = this->m_quaternion.normalized();
    std::cout << "Debug q---------------------" << std::endl;
    std::cout << this->m_quaternion << std::endl;
}

void RigidBody::ComputeReferenceInertia(double m)
{
    Eigen::Matrix3d refI = Eigen::Matrix3d::Zero();
    for (int i = 0; i < this->m_rVertices.rows(); i++)
    {
        Eigen::Vector3d v = this->m_rVertices.row(i);
        Eigen::Matrix3d tmp = v.transpose()* v * Eigen::Matrix3d::Identity() + v * v.transpose();
        refI = refI + tmp;
    }

    this->m_rInertia = m * refI;
    
}

void RigidBody::ComputeInertia(Eigen::Matrix3d& Rotation)
{
    this->m_Inertia = Rotation * this->m_rInertia * Rotation.transpose();
}

void RigidBody::ComputeInertia()
{
    Eigen::Matrix3d rotation = this->m_quaternion.toRotationMatrix();
    ComputeInertia(rotation);
}

void RigidBody::ComputeTorque(Eigen::Matrix3d& Rotation)
{
    Eigen::Vector3d torq = Eigen::Vector3d::Zero();
    for (int i = 0; i < this->m_rVertices.rows(); i++)
    {
        Eigen::Vector3d tmp = this->m_rVertices.row(i);
        Eigen::Vector3d r = Rotation * tmp;
        torq += r.cross(this->m_force);
    }
    this->m_torque = torq;
}

void RigidBody::ComputeTorque()
{
    Eigen::Matrix3d rotation = this->m_quaternion.toRotationMatrix();
    ComputeTorque(rotation);
}

void RigidBody::Simulate(Eigen::MatrixXd& V)
{
    Eigen::Matrix3d Rotation = this->m_quaternion.toRotationMatrix();
    Eigen::MatrixXd rV = Rotation * this->m_rVertices.transpose();
    rV = rV.colwise() + this->m_barycenter;
    V = rV.transpose();
}

void RigidBody::ComputeMassCenter()
{
    Eigen::Vector3d c = {0, 0, 0};
    for (int i = 0; i < this->m_rVertices.rows(); i++)
    {
        Eigen::Vector3d vi = this->m_rVertices.row(i);
        c = c + vi;
    }
    c = c / this->m_rVertices.rows();
    this->m_barycenter = c;
}

void RigidBody::SetPosition(Eigen::Vector3d& pos)
{
    this->m_barycenter = pos;
}

void RigidBody::SetQuaternion(Eigen::Quaternion<double>& quat)
{
    this->m_quaternion = quat;
}

void RigidBody::SetAngularVelocity(Eigen::Vector3d& w)
{
    this->m_aVelocity = w;
}

void RigidBody::SetVelocity(Eigen::Vector3d& v)
{
    this->m_velocity = v;
}

void RigidBody::Reset(Eigen::MatrixXd& v)
{
    this->m_aVelocity = {0, 0, 0};
    this->m_velocity = {0, 0, 0};
    this->ComputeMassCenter();
    this->m_force = {0, -10, 0};
    this->m_force = this->m_mass * this->m_force;
    this->m_quaternion = this->m_quaternion.Identity();
    this->ComputeReferenceInertia(1.0);
    this->m_torque = {0, 0, 0};
    this->ComputeInertia();
    v = this->m_rVertices;
}

Eigen::Vector3d RigidBody::GetVelocity()
{
    return this->m_velocity;
}

Eigen::Vector3d RigidBody::GetAngularVelocity()
{
    return this->m_aVelocity;
}

Eigen::Quaternion<double> RigidBody::GetQuaternion()
{
    return this->m_quaternion;
}

Eigen::Vector3d RigidBody::GetTorque()
{
    return this->m_torque;
}

Eigen::Matrix3d RigidBody::GetInertia()
{
    return this->m_Inertia;
}

Eigen::Vector3d RigidBody::GetMassCenter()
{
    return this->m_barycenter;
}

void RigidBody::ComputeCollisionAndResponse(Eigen::Vector3d P, Eigen::Vector3d N, double dt, double restitution)
{
    Eigen::Matrix3d crossM;
    antisymetric(this->m_aVelocity, crossM);
    Eigen::Vector3d avgV = {0, 0, 0};
    Eigen::Vector3d avgP = {0, 0, 0};
    Eigen::Matrix3d Rotation = this->m_quaternion.toRotationMatrix();
    int count = 0;

    for (int i = 0; i < this->m_rVertices.rows(); i++)
    {
        Eigen::Vector3d ri = this->m_rVertices.row(i);
        Eigen::Vector3d Rri = Rotation * ri;
        Eigen::Vector3d xi = Rri + this->m_barycenter;
        Eigen::Vector3d xp = xi - P;
        double dist = xp.dot(N);
        if (dist < 0)
        {
            Eigen::Vector3d Rwi = crossM * Rri;
            Eigen::Vector3d vi = this->m_velocity + Rwi;

            if (vi.dot(N) < 0)
            {
                count++;
                avgV += vi;
                avgP += Rri;
            }
        }
    }

    if (count > 0)
    {
        avgV = avgV / count;
        avgP = avgP / count;
        Eigen::Vector3d vn = avgV.dot(N) * N;
        Eigen::Vector3d vt = avgV - vn;

        Eigen::Vector3d vn_new = -restitution * vn;
        double a = std::max(0.0, 1 - dt * (1 + restitution) * vn.norm() / vt.norm());
        Eigen::Vector3d vt_new = a * vt;
        Eigen::Vector3d v_new = vt_new + vn_new;

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d CRri;
        antisymetric(avgP, CRri);
        Eigen::Matrix3d K = (1 / this->m_mass) * I - CRri * I.inverse() * CRri;
        Eigen::Vector3d j = K.inverse() * (v_new - avgV);

        this->m_velocity = this->m_velocity + (1 / this->m_mass) * j;
        this->m_aVelocity = this->m_aVelocity + I.inverse() * CRri * j;
    }
}