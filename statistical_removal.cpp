#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char *argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    const std::string PATH = "../pointclouds/";

    /* read the pointcloud data */
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>(PATH + "table_scene_lms400.pcd", *cloud);

    /* display cloud data before filtering (debug) */
    std::cerr << "Cloud before filtering" << std::endl;
    std::cerr << *cloud << std::endl;

    const double NEIGHBORS = 50.0f;
    const double STDDEV_MULT = 1.0f;

    /* create the filtering object */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(NEIGHBORS);
    sor.setStddevMulThresh(STDDEV_MULT);
    sor.filter(*cloud_filtered);

    /* display cloud data after filtering (debug) */
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    /* save the inliers pointcloud to file */
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(PATH + "table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    /* save the outlier pointcloud to file */
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>(PATH + "table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return(0);
}
