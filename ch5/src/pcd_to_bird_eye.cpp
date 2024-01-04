#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 使用gflag库可以方便的在命令行更改相应的参数
DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, 0.2, "俯视图最低高度");
DEFINE_double(max_z, 2.5, "俯视图最高高度");

/// 本节演示如何将一个点云转换为俯视图像
void GenerateBEVImage(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    // 计算点云边界
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const pcl::PointXYZI& p1, const pcl::PointXYZI& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const pcl::PointXYZI& p1, const pcl::PointXYZI& p2) { return p1.y < p2.y; });
    
    // 点云边界
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    // 俯视图分辨率，单位：m/像素
    const double inv_r = 1.0 / FLAGS_image_resolution;

    // 图像行列数
    const int image_rows = int((max_y - min_y) * inv_r);
    const int image_cols = int((max_x - min_x) * inv_r);

    // 点云中心
    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    // 图像中心
    float x_center_image = image_cols / 2;
    float y_center_image = image_rows / 2;

    // 生成图像   cv::Scalar(255, 255, 255) 是用于初始化图像的颜色，这里是白色
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points) {
        // 图像中心与点云中心重合，计算点云对应的图像像素坐标
        int x = int((pt.x - x_center) * inv_r + x_center_image);
        int y = int((pt.y - y_center) * inv_r + y_center_image);
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }

    cv::imwrite("./bev.png", image);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 读取点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points: " << cloud->size();
    GenerateBEVImage(cloud);

    return 0;
}

