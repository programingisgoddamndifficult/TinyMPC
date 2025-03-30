// Quadrotor hovering example

// This script is just to show how to use the library, the data for this example is not tuned for our Crazyflie demo. Check the firmware code for more details.

// - NSTATES = 12
// - NINPUTS = 4
// - NHORIZON = anything you want
// - tinytype = float if you want to run on microcontrollers
// States: x (m), y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
// phi, theta, psi are NOT Euler angles, they are Rodiguez parameters
// check this paper for more details: https://ieeexplore.ieee.org/document/9326337
// Inputs: u1, u2, u3, u4 (motor thrust 0-1, order from Crazyflie)

#define NSTATES 12
#define NINPUTS 4

#define NHORIZON 10

#include <iostream>
#include <tinympc/admm.hpp>
#include <tinympc/tiny_api.hpp>
#include "problem_data/quadrotor_100hz_params.hpp"
#include <fstream>
#include <random>

extern "C" {
typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

int main() {
    TinySolver *solver;

    tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
    tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
    tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
    tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-5);
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(5);
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-0.3);
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(0.3);

    int status = tiny_setup(&solver,
                            Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                            rho_value, NSTATES, NINPUTS, NHORIZON,
                            x_min, x_max, u_min, u_max, 1);
    
    // Update whichever settings we'd like
    solver->settings->max_iter = 100;
    
    // Alias solver->work for brevity
    TinyWorkspace *work = solver->work;

    // 随机初始化器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> uniform_dist(-0.1, 0.1);
    std::uniform_real_distribution<> uniform_z(0.5, 1.5);
    std::uniform_real_distribution<> uniform_theta(-1.0, 1.0);

    // 初始状态
    tiny_VectorNx x0;
    x0 << uniform_dist(gen),     // x
          uniform_dist(gen),     // y
          uniform_z(gen),        // z
          uniform_theta(gen),    // phi
          uniform_theta(gen),              // theta (仰角 90°)
          M_PI / 2,    // psi
          uniform_dist(gen),     // dx
          uniform_dist(gen),     // dy
          uniform_dist(gen),     // dz
          uniform_dist(gen),     // dphi
          uniform_dist(gen),     // dtheta
          uniform_dist(gen);     // dpsi

    // 目标轨迹
    tiny_VectorNx Xref_origin;
    Xref_origin << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    work->Xref = Xref_origin.replicate<1, NHORIZON>();

    // 打开文件以保存轨迹数据和控制输入数据
    std::ofstream trajectory_file("trajectory_data.txt");
    std::ofstream control_input_file("control_input_data.txt");

    if (!trajectory_file.is_open() || !control_input_file.is_open()) {
        std::cerr << "Failed to open trajectory or control input data file." << std::endl;
        return -1;
    }

    // 重力加速度常量
    const tinytype g = 9.81;
    for (int k = 0; k < 250; ++k) {
        // 1. Update measurement
        tiny_set_x0(solver, x0);

	
        // 2. Solve MPC problem
        tiny_solve(solver);

        // 3. Simulate forward
        x0 = work->Adyn * x0 + work->Bdyn * work->u.col(0);
        
            // 手动调整 dz，加入重力影响
        x0(8) -= g * 0.01 * 0.27;  // 假设仿真时间步长为 0.01 秒 (100 Hz)

        // 记录位置 (x, y, z)
        trajectory_file << x0(0) << " " << x0(1) << " " << x0(2) << std::endl;

        // 记录控制输入 (u1, u2, u3, u4)
        control_input_file << work->u(0, 0) << " "
                           << work->u(1, 0) << " "
                           << work->u(2, 0) << " "
                           << work->u(3, 0) << std::endl;

        // 可选：打印追踪误差
        printf("tracking error at step %2d: %.4f\n", k, (x0 - work->Xref.col(1)).norm());
    }

    trajectory_file.close();
    control_input_file.close();

    return 0;
}
} /* extern "C" */
