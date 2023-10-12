#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class RigidBody
{
    public:
        RigidBody(double mass, const Eigen::MatrixXd& V, double m);
        void EulerStep(double timestep);
        void Simulate(Eigen::MatrixXd& V);
        void ComputeInertia(Eigen::Matrix3d& Rotation);
        void ComputeInertia();
        void ComputeReferenceInertia(double m);
        void ComputeAngularVelocity();
        void ComputeTorque(Eigen::Matrix3d& Rotation);
        void ComputeTorque();
        void ComputeMassCenter();
        void SetPosition(Eigen::Vector3d& pos);
        void SetQuaternion(Eigen::Quaternion<double>& quat);
        void SetVelocity(Eigen::Vector3d& v);
        void SetAngularVelocity(Eigen::Vector3d& w);
        void Reset(Eigen::MatrixXd& v);

        void ComputeCollisionAndResponse(Eigen::Vector3d P, Eigen::Vector3d N, double dt, double restitution);

        Eigen::Vector3d GetVelocity();
        Eigen::Vector3d GetAngularVelocity();
        Eigen::Quaternion<double> GetQuaternion();
        Eigen::Vector3d GetTorque();
        Eigen::Matrix3d GetInertia();
        Eigen::Vector3d GetMassCenter();

    
    private:
        Eigen::Vector3d m_barycenter;
        double m_mass;
        Eigen::Vector3d m_velocity;
        Eigen::Vector3d m_aVelocity;
        Eigen::Quaternion<double> m_quaternion;
        Eigen::Vector3d m_force;
        Eigen::Matrix3d m_rInertia;
        Eigen::Matrix3d m_Inertia;
        Eigen::Vector3d m_torque;
        Eigen::MatrixXd m_rVertices;
};


inline void antisymetric(const Eigen::Vector3d& w, Eigen::Matrix3d& a)
{
    a(0, 0) = 0.0;      a(0, 1) = -w(2);    a(0, 2) = w(1);
    a(1, 0) = w(2);     a(1, 1) = 0.0;      a(1, 2) = -w(0);
    a(2, 0) = -w(1);    a(2, 1) = w(0);     a(2, 2) = 0.0;

}
#endif