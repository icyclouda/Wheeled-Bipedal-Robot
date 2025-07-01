#include <iostream>
#include <cmath>
#include <iomanip>

// 3x3矩阵类，简化矩阵运算
class Matrix3x3
{
private:
    double data[3][3];

public:
    // 默认构造函数，初始化为零矩阵
    Matrix3x3()
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                data[i][j] = 0.0;
    }

    // 元素访问
    double &operator()(int i, int j) { return data[i][j]; }
    const double &operator()(int i, int j) const { return data[i][j]; }

    // 矩阵乘法
    Matrix3x3 operator*(const Matrix3x3 &other) const
    {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    result(i, j) += data[i][k] * other(k, j);
        return result;
    }

    // 矩阵转置
    Matrix3x3 transpose() const
    {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result(i, j) = data[j][i];
        return result;
    }

    // 打印矩阵（用于调试）
    void print() const
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
                std::cout << std::fixed << std::setprecision(4) << data[i][j] << "\t";
            std::cout << std::endl;
        }
    }
};

// 3D向量类，简化向量运算
class Vector3
{
private:
    double data[3];

public:
    // 默认构造函数
    Vector3(double x = 0.0, double y = 0.0, double z = 0.0)
    {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    // 元素访问
    double &operator[](int i) { return data[i]; }
    const double &operator[](int i) const { return data[i]; }

    // 向量与矩阵乘法
    Vector3 operator*(const Matrix3x3 &mat) const
    {
        Vector3 result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result[i] += data[j] * mat(j, i);
        return result;
    }

    // 打印向量
    void print() const
    {
        std::cout << "[" << std::fixed << std::setprecision(6)
                  << data[0] << ", " << data[1] << ", " << data[2] << "]" << std::endl;
    }
};

// 角度转换：度转弧度
double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

// 欧拉角转旋转矩阵（ZYX顺序：Yaw-Pitch-Roll）
Matrix3x3 eulerToRotation(double roll, double pitch, double yaw)
{
    // 计算各个旋转矩阵
    Matrix3x3 Rx; // 绕X轴旋转（Roll）
    Rx(0, 0) = 1.0;
    Rx(1, 1) = cos(roll);
    Rx(1, 2) = -sin(roll);
    Rx(2, 1) = sin(roll);
    Rx(2, 2) = cos(roll);

    Matrix3x3 Ry; // 绕Y轴旋转（Pitch）
    Ry(0, 0) = cos(pitch);
    Ry(0, 2) = sin(pitch);
    Ry(1, 1) = 1.0;
    Ry(2, 0) = -sin(pitch);
    Ry(2, 2) = cos(pitch);

    Matrix3x3 Rz; // 绕Z轴旋转（Yaw）
    Rz(0, 0) = cos(yaw);
    Rz(0, 1) = -sin(yaw);
    Rz(1, 0) = sin(yaw);
    Rz(1, 1) = cos(yaw);
    Rz(2, 2) = 1.0;

    // 组合旋转矩阵（ZYX顺序）
    return Rx * Ry * Rz;
}

// 欧拉角转重力矢量
Vector3 eulerToGravity(double roll, double pitch, double yaw, double g = 9.81)
{
    // 计算旋转矩阵
    Matrix3x3 R = eulerToRotation(roll, pitch, yaw);

    // 世界坐标系中的重力矢量 [0, 0, -g]
    Vector3 gravity_world(0.0, 0.0, -g);

    // 转换到机器人坐标系（旋转矩阵转置）
    return gravity_world * R.transpose();
}

int main()
{
    std::cout << "=== 欧拉角转重力矢量计算器 ===" << std::endl;

    double roll_deg, pitch_deg, yaw_deg;
    std::cout << "请输入欧拉角（单位：度）：" << std::endl;
    std::cout << "Roll (横滚角): ";
    std::cin >> roll_deg;
    std::cout << "Pitch (俯仰角): ";
    std::cin >> pitch_deg;
    std::cout << "Yaw (偏航角): ";
    std::cin >> yaw_deg;

    // 转换为弧度
    double roll_rad = degToRad(roll_deg);
    double pitch_rad = degToRad(pitch_deg);
    double yaw_rad = degToRad(yaw_deg);

    // 计算重力矢量
    Vector3 gravity = eulerToGravity(roll_rad, pitch_rad, yaw_rad);

    // 输出结果
    std::cout << "\n机器人坐标系中的重力矢量 (m/s²):" << std::endl;
    std::cout << "gx = " << std::fixed << std::setprecision(6) << gravity[0] << std::endl;
    std::cout << "gy = " << std::fixed << std::setprecision(6) << gravity[1] << std::endl;
    std::cout << "gz = " << std::fixed << std::setprecision(6) << gravity[2] << std::endl;

    // 计算重力大小（验证）
    double magnitude = std::sqrt(
        gravity[0] * gravity[0] +
        gravity[1] * gravity[1] +
        gravity[2] * gravity[2]);
    std::cout << "重力大小: " << magnitude << " m/s²" << std::endl;

    return 0;
}