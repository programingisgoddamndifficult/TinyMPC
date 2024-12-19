#include <iostream>
#include <Eigen/Dense>
#include <tinympc/tiny_api.hpp>
#include <tinympc/tiny_api_constants.hpp>


// 障碍物的初始位置和速度
Eigen::VectorXd obstacle_position(3);  // x, y, z
Eigen::VectorXd obstacle_velocity(3);  // vx, vy, vz
double obstacle_radius = 1.0;  // 障碍物半径

// 定义一个障碍物的位置和速度
void initialize_obstacle() {
    obstacle_position << 5.0, 5.0, 1.0;  // 初始位置 (5, 5, 1)
    obstacle_velocity << -0.1, 0.0, 0.0;  // 速度 (vx, vy, vz)
}

// 更新障碍物位置
void update_obstacle_position(double dt) {
    obstacle_position += obstacle_velocity * dt;
}

// 计算并添加动态障碍物约束
int add_obstacle_constraint(TinySolver* solver) {
    // 获取当前系统的位置
    Eigen::VectorXd x_state = solver->work->x.col(0);  // 当前状态位置

    // 计算障碍物与当前位置的距离
    double distance = (x_state.head(3) - obstacle_position).norm();

    if (distance < obstacle_radius) {
        // 计算新的障碍物约束
        Eigen::MatrixXd A = (x_state.head(3) - obstacle_position).transpose();
        A = A / distance;  // 计算单位向量
        Eigen::VectorXd b(1);
        b << obstacle_radius - distance;

        // 添加新的约束到工作空间
        solver->work->x_min = x_state - obstacle_radius * Eigen::VectorXd::Ones(3);
        solver->work->x_max = x_state + obstacle_radius * Eigen::VectorXd::Ones(3);

        std::cout << "Obstacle avoidance constraint added" << std::endl;
        return 0;  // 成功添加约束
    }

    return 1;  // 没有碰撞，无需添加约束
}

// 设置初始状态和参考轨迹
void set_initial_conditions(TinySolver* solver) {
    // 假设初始位置为 [0, 0, 0]，速度为 [0, 0, 0]
    Eigen::VectorXd x0(3);
    x0 << 0.0, 0.0, 0.0;

    tiny_set_x0(solver, x0);

    // 设置参考轨迹 (假设为一个简单的直线轨迹)
    Eigen::MatrixXd x_ref(3, 10);  // 3D 状态，10 个时间步
    x_ref.setZero();
    for (int i = 0; i < 10; i++) {
        x_ref(0, i) = i * 0.5;  // x 位置逐渐增大
    }
    tiny_set_x_ref(solver, x_ref);

    Eigen::MatrixXd u_ref(3, 9);  // 输入参考轨迹
    u_ref.setZero();
    tiny_set_u_ref(solver, u_ref);
}

#include <Eigen/Dense>
#include <tinympc/tiny_api.hpp>
#include <tinympc/tiny_api_constants.hpp>

int setup_mpc(TinySolver** solverp) {
    TinySolver* solver = nullptr;
    tinyMatrix Adyn = tinyMatrix::Identity(3, 3);  // 状态转移矩阵
    tinyMatrix Bdyn = tinyMatrix::Identity(3, 3);  // 输入矩阵
    tinyMatrix Q = tinyMatrix::Identity(3, 3);  // 状态代价
    tinyMatrix R = tinyMatrix::Identity(3, 3);  // 输入代价
    tinytype rho = 1.0;
    int nx = 3, nu = 3, N = 10;

    // 使用 Eigen 动态矩阵并设置常数值
    Eigen::MatrixXd x_min(3, 10);  // 3 rows (state dimension), 10 columns (horizon)
    x_min.setConstant(-5.0);        // 设置状态的下限

    Eigen::MatrixXd x_max(3, 10);  // 3 rows, 10 columns
    x_max.setConstant(5.0);         // 设置状态的上限

    Eigen::MatrixXd u_min(3, 9);   // 3 rows (input dimension), 9 columns (horizon - 1)
    u_min.setConstant(-0.5);       // 设置控制输入的下限

    Eigen::MatrixXd u_max(3, 9);   // 3 rows, 9 columns
    u_max.setConstant(0.5);        // 设置控制输入的上限


    // 调用 tiny_setup 函数，传递所有参数
    int status = tiny_setup(&solver, Adyn, Bdyn, Q, R, rho, nx, nu, N, x_min, x_max, u_min, u_max, 1);

    if (status != 0) {
        std::cout << "Error in setting up TinyMPC solver!" << std::endl;
        return status;
    }

    *solverp = solver;
    return 0;
}

// 求解 MPC 问题
void solve_mpc(TinySolver* solver, double dt) {
    // 更新障碍物位置
    update_obstacle_position(dt);

    // 添加动态障碍物约束
    int status = add_obstacle_constraint(solver);
    if (status == 0) {
        std::cout << "Obstacle avoidance constraint added" << std::endl;
    }

    // 求解 MPC 问题
    int solve_status = tiny_solve(solver);
    if (solve_status == 0) {
        std::cout << "MPC solved successfully" << std::endl;
    } else {
        std::cout << "MPC solve failed" << std::endl;
    }
}

int main() {
    // 初始化障碍物
    initialize_obstacle();

    // 设置 MPC 求解器
    TinySolver* solver = nullptr;
    int setup_status = setup_mpc(&solver);
    if (setup_status != 0) {
        std::cout << "Error in MPC setup!" << std::endl;
        return setup_status;
    }

    // 设置初始状态和参考轨迹
    set_initial_conditions(solver);

    // 控制循环
    double dt = 0.1;  // 时间步长
    for (int i = 0; i < 100; ++i) {
        std::cout << "Iteration " << i << std::endl;
        solve_mpc(solver, dt);
    }

    return 0;
}

