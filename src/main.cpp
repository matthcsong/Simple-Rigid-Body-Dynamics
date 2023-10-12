#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "igl/readOBJ.h"
#include <Eigen/Dense>
#include "RigidBody.h"

Eigen::MatrixXd meshV;
Eigen::MatrixXi meshF;

Eigen::MatrixXd meshV2;
Eigen::MatrixXi meshF2;

double nloop = 1.0000001;
bool start = false;
double m = 1;
double position_x = 0.0;
double position_y = 1.0;
double position_z = 0.0;
double velocity_x = 0.0;
double velocity_y = 0.0;
double velocity_z = 0.0;
double angular_v_x = 0.0;
double angular_v_y = 0.0;
double angular_v_z = 0.0;


Eigen::Vector3d position(0, 2, 0);
Eigen::Vector3d position2(-1, 2, 0);
double dt = 1.0 / 100;
int nTimesteps = 200;
RigidBody *RB;
// RigidBody *RB2;

double ComputeMass(Eigen::MatrixXd& V)
{
    int n = V.rows();

    return m*n;
}

void mySubroutine()
{
    RB->ComputeTorque();
    RB->ComputeInertia();
    RB->EulerStep(dt);
    RB->Simulate(meshV);
    polyscope::getSurfaceMesh("Input mesh")->updateVertexPositions(meshV);


    // RB2->ComputeTorque();
    // RB2->ComputeInertia();
    // RB2->EulerStep(dt);
    // RB2->Simulate(meshV2);
    // polyscope::getSurfaceMesh("Input mesh2")->updateVertexPositions(meshV2);
        // Eigen::Vector3d t = rb->GetAngularVelocity();
        // std::cout << "----------------------" << std::endl;
        // std::cout << t.transpose() << std::endl;
    // polyscope::getSurfaceMesh("Input mesh")->updateVertexPositions(meshV);
    // auto *surf = polyscope::registerSurfaceMesh("Input Mesh", meshV, meshF);
}

void myCallback()
{
    ImGui::PushItemWidth(100);
    ImGui::InputInt("time steps", &nTimesteps);

    ImGui::InputDouble("pos.x", &position_x);
    ImGui::SameLine();
    ImGui::InputDouble("V.x", &velocity_x);
    ImGui::SameLine();
    ImGui::InputDouble("W.x", &angular_v_x);

    ImGui::InputDouble("pos.y", &position_y);
    ImGui::SameLine();
    ImGui::InputDouble("V.y", &velocity_y);
    ImGui::SameLine();
    ImGui::InputDouble("W.y", &angular_v_y);

    ImGui::InputDouble("pos.z", &position_z);
    ImGui::SameLine();
    ImGui::InputDouble("V.z", &velocity_z);
    ImGui::SameLine();
    ImGui::InputDouble("W.z", &angular_v_z);


    if (ImGui::Button("Simulate"))
    { 
        mySubroutine();
        start = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        Eigen::Vector3d pos = {position_x, position_y, position_z};
        Eigen::Vector3d vel = {velocity_x, velocity_y, velocity_z};
        Eigen::Vector3d w = {angular_v_x, angular_v_y, angular_v_z};
        RB->Reset(meshV);
        RB->SetPosition(pos);
        RB->SetAngularVelocity(w);
        RB->SetVelocity(vel);
        RB->Simulate(meshV);
        polyscope::getSurfaceMesh("Input mesh")->updateVertexPositions(meshV);
        start = true;
    }

    if (start)
    {
        if (nTimesteps > 0)
        {
            nTimesteps -= 1;
        }
        else
        {
            start = false;
        }

        mySubroutine();
    }

    ImGui::PopItemWidth();
}

int main()
{
    // Initialize polyscope
    polyscope::init();
    //polyscope::options::invokeUserCallbackForNestedShow = true;

    igl::readOBJ("/Users/hcsong/Workspaces/rgb580/data/bunny.obj", meshV, meshF);
    polyscope::registerSurfaceMesh("Input mesh", meshV, meshF);

    // igl::readOBJ("../data/bunny.obj", meshV2, meshF2);
    // polyscope::registerSurfaceMesh("Input mesh2", meshV2, meshF2);

    // initialization
    double mass = ComputeMass(meshV);
    RigidBody *rb = new RigidBody(mass, meshV, 1.0);
    // RigidBody *rb2 = new RigidBody(mass, meshV2, 1.0);
    RB = rb;
    // RB2 = rb2;
    rb->SetPosition(position);
    rb->Simulate(meshV);

    // rb2->SetPosition(position2);
    // rb2->Simulate(meshV2);

    polyscope::getSurfaceMesh("Input mesh")->updateVertexPositions(meshV);
    // polyscope::getSurfaceMesh("Input mesh2")->updateVertexPositions(meshV2);

    polyscope::state::userCallback = myCallback;
    polyscope::show();
    delete rb;
    return 0;
}

