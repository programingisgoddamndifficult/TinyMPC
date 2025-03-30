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

// 定义 8 字形轨迹数据
tinytype Xref_data[NTOTAL * NSTATES];

// 类型定义
typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

// 生成 8 字形轨迹的函数
void compute_figure8_trajectory(double t, double traj_period, double scaling, double &coords_a, double &coords_b) {
    double traj_freq = 2.0 * M_PI / traj_period;
    coords_a = scaling * sin(traj_freq * t);
    coords_b = scaling * sin(traj_freq * t) * cos(traj_freq * t);
}

int main() {
    // 设置 8 字形轨迹的数据
    const double A = 1.0;       // 轨迹幅度
    const double dt = 0.1;      // 时间步长
    const double traj_period = 10.0; // 轨迹周期
    const double theta = M_PI / 4;   // 绕 x 轴旋转的角度（45°）
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);

    for (int i = 0; i < NTOTAL; ++i) {
        double t = i * dt;

        // 使用 _figure8 逻辑生成轨迹
        double x = 0.0;
        double y = 0.0;
        compute_figure8_trajectory(t, traj_period, A, x, y);
        double z = 0.5;  // 假设固定在 z = 0.5 的高度

        // 绕 x 轴旋转变换
        double x_rot = x;
        double y_rot = cos_theta * y - sin_theta * z;
        double z_rot = sin_theta * y + cos_theta * z;

        // 填充 Xref_data 数组
        Xref_data[i * NSTATES + 0] = x_rot;  // x'
        Xref_data[i * NSTATES + 1] = y_rot;  // y'
        Xref_data[i * NSTATES + 2] = z_rot;  // z'

        // 其他状态（例如角度等）假设为零
        for (int j = 3; j < NSTATES; ++j) {
            Xref_data[i * NSTATES + j] = 0.0;
        }
    }

    // 初始化优化器
    TinySolver *solver;

    // 映射参数矩阵和向量
    tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
    tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
    tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
    tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

    // 设置状态和输入约束
    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-5);  // 状态最小值
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(5);   // 状态最大值
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-0.5); // 输入最小值
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(0.5);  // 输入最大值

    // 配置优化器
    int status = tiny_setup(&solver,
                            Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                            rho_value, NSTATES, NINPUTS, NHORIZON,
                            x_min, x_max, u_min, u_max, 1);

    if (status != 0) {
        std::cerr << "Error: Failed to set up the solver!" << std::endl;
        return -1;
    }

    // 设置优化器求解器参数
    solver->settings->max_iter = 100;

    // Alias solver->work for brevity
    TinyWorkspace *work = solver->work;

    // 映射参考轨迹数据
    Matrix<tinytype, NSTATES, NTOTAL> Xref_total = Eigen::Map<Matrix<tinytype, NSTATES, NTOTAL>>(Xref_data);
    work->Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);

    // 初始状态
    tiny_VectorNx x0 = work->Xref.col(0);

    // 打开文件用于写入轨迹数据
    std::ofstream trajectory_file("trajectory_data_tracking.txt");
    if (!trajectory_file.is_open()) {
        std::cerr << "Error: Unable to open file for writing!" << std::endl;
        return -1;
    }

    // 打开文件用于写入控制输入数据
    std::ofstream control_input_file("control_inputs.txt");
    if (!control_input_file.is_open()) {
        std::cerr << "Error: Unable to open control input file for writing!" << std::endl;
        return -1;
    }

    // 写入文件头
    trajectory_file << "x, y, z, time" << std::endl;
    control_input_file << "u1, u2, u3, u4, time" << std::endl;

    // 仿真主循环
    for (int k = 0; k < NTOTAL - NHORIZON; ++k) {
        // 将当前状态写入文件 (x0 包含 [x, y, z, phi, theta, psi, ...])
        trajectory_file << x0[0] << ", " << x0[1] << ", " << x0[2] << ", " << k * dt << std::endl;

        // 更新测量
        tiny_set_x0(solver, x0);

        // 更新参考轨迹
        work->Xref = Xref_total.block<NSTATES, NHORIZON>(0, k);

        // 重置对偶变量
        work->y = tiny_MatrixNuNhm1::Zero();
        work->g = tiny_MatrixNxNh::Zero();

        // 求解 MPC 问题
        int solve_status = tiny_solve(solver);
        if (solve_status != 0) {
            std::cerr << "MPC Solver Failed at step: " << k << std::endl;
            break;
        }

        // 记录控制输入
        control_input_file << work->u(0, 0) << ", " << work->u(1, 0) << ", "
                           << work->u(2, 0) << ", " << work->u(3, 0) << ", "
                           << k * dt << std::endl;

        // 计算下一状态
        x0 = work->Adyn * x0 + work->Bdyn * work->u.col(0);
    }

    // 关闭文件
    control_input_file.close();
    trajectory_file.close();

    return 0;
}

} /* extern "C" */
