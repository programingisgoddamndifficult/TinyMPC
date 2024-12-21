
// 修改轨迹数据的总点数和状态维度
#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 10
#define NTOTAL 301

#include <iostream>
#include <fstream>  // 用于文件输出
#include <tinympc/tiny_api.hpp>
#include <cmath>    // 用于 sin 和 cos 函数

extern "C" {

#include "problem_data/quadrotor_20hz_params.hpp"

// 定义8字形轨迹数据
tinytype Xref_data[NTOTAL * NSTATES];

typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

int main() {
    // 设置8字形轨迹的数据
    const double A = 1.0;  // 轨迹幅度
    const double dt = 0.1;  // 时间步长
    for (int i = 0; i < NTOTAL; ++i) {
        double t = i * dt;

        // 8字形轨迹：x(t) = A * sin(t), y(t) = A * cos(t) * sin(t)
        double x = A * sin(t);
        double y = A * cos(t) * sin(t);
        double z = 1.0;  // 假设固定在 z = 1 的高度

        // 填充 Xref_data 数组
        Xref_data[i * NSTATES + 0] = x;  // x
        Xref_data[i * NSTATES + 1] = y;  // y
        Xref_data[i * NSTATES + 2] = z;  // z

        // 其他状态（例如角度等）假设为零
        for (int j = 3; j < NSTATES; ++j) {
            Xref_data[i * NSTATES + j] = 0.0;
        }
    }

    // 设置优化器
    TinySolver *solver;
    tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
    tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
    tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
    tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-5);
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(5);
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-0.5);
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(0.5);

    int status = tiny_setup(&solver,
                            Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                            rho_value, NSTATES, NINPUTS, NHORIZON,
                            x_min, x_max, u_min, u_max, 1);
    
    // Update settings
    solver->settings->max_iter = 100;

    // Alias solver->work for brevity
    TinyWorkspace *work = solver->work;

    // Map数据
    Matrix<tinytype, NSTATES, NTOTAL> Xref_total = Eigen::Map<Matrix<tinytype, NSTATES, NTOTAL>>(Xref_data);
    work->Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);

    // 初始状态
    tiny_VectorNx x0;
    x0 = work->Xref.col(0);

    // 打开文件用于写入轨迹数据
    std::ofstream trajectory_file("trajectory_data_tracking.txt");

    if (!trajectory_file.is_open()) {
        std::cerr << "Error: Unable to open file for writing!" << std::endl;
        return -1;
    }

    // 写入文件头
    trajectory_file << "x, y, z, time" << std::endl;

    // 仿真循环
    for (int k = 0; k < NTOTAL - NHORIZON; ++k) {
        // 将当前状态写入文件 (x0 包含 [x, y, z, phi, theta, psi, ...])
        trajectory_file << x0[0] << ", " << x0[1] << ", " << x0[2] << ", " << k * 0.1 << std::endl;

        // 1. 更新测量
        tiny_set_x0(solver, x0);

        // 2. 更新参考轨迹
        work->Xref = Xref_total.block<NSTATES, NHORIZON>(0, k);

        // 3. 如果需要，重置对偶变量
        work->y = tiny_MatrixNuNhm1::Zero();
        work->g = tiny_MatrixNxNh::Zero();

        // 4. 求解MPC问题
        tiny_solve(solver);

        // 5. 向前仿真
        x0 = work->Adyn * x0 + work->Bdyn * work->u.col(0);
    }

    trajectory_file.close();  // 关闭文件
    return 0;
}

} /* extern "C" */
