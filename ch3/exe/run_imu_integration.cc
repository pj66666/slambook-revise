#include <glog/logging.h>
#include <iomanip>

#include "imu_integration.h"
#include "io_utils.h"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入文本文件，同时它将状态输出到data/ch3/state.txt中
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    pj::TxtIO io(FLAGS_imu_txt_path);  // 读取数据文本文件，并调用回调函数.数据文本文件主要提供IMU/Odom/GNSS读数

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    pj::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, 
                            const Vec3d& v,const Vec3d& p) 
    {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) 
        { 
            fout << v[0] << " " << v[1] << " " << v[2] << " "; 
        };

        auto save_quat = [](std::ofstream& fout, const Quatd& q) 
        {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    std::ofstream fout("./data/ch3/state.txt");
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout](const pj::IMU& imu) 
    {
        imu_integ.AddIMU(imu);
        save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
    }).Go();

    return 0;
}