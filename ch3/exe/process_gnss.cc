//
// Created by xiang on 2022/1/4.
//

#include <glog/logging.h>
#include <iomanip>
#include <memory>

#include "gnss.h"
#include "io_utils.h"
#include "utm_convert.h"

DEFINE_string(txt_path, "./data/ch3/10.txt", "数据文件路径");

// 以下参数仅针对本书提供的数据
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");

/**
 * 本程序演示如何处理GNSS数据
 * 我们将GNSS原始读数处理成能够进行后续处理的6自由度Pose
 * 需要处理UTM转换、RTK天线外参、坐标系转换三个步骤
 *
 * 我们将结果保存在文件中，然后用python脚本进行可视化
 */

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    pj::TxtIO io(fLS::FLAGS_txt_path);

    std::ofstream fout("./data/ch3/gnss_output.txt");
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    // 和imu那个一样
    auto save_result = [](std::ofstream& fout, double timestamp, const SE3& pose) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, pose.translation());
        save_quat(fout, pose.unit_quaternion());
        fout << std::endl;
    };



    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();
    io.SetGNSSProcessFunc([&](const pj::GNSS& gnss) {
          pj::GNSS gnss_out = gnss;
          // 读取的数据转换为GNSS
          /*gnss_reading – 输入gnss读数 antenna_pos – 安装偏移
            antenna_angle – 安装偏角 map_origin – 地图原点，指定时，将从UTM位置中减掉坐标原点
            */
          if (pj::ConvertGps2UTM(gnss_out, antenna_pos, FLAGS_antenna_angle)) {
              if (!first_gnss_set) {
                  // 把第一次读取数据位置记为原点，后续数据都减去它，也就是从0开始
                  origin = gnss_out.utm_pose_.translation();
                  first_gnss_set = true;
              }

              /// 减掉一个原点
              gnss_out.utm_pose_.translation() -= origin;

              save_result(fout, gnss_out.unix_time_, gnss_out.utm_pose_);              
          }
      }).Go();

    return 0;
}