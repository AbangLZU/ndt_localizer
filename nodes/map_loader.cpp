#include "map_loader.h"

MapLoader::MapLoader(ros::NodeHandle &nh){
    std::string pcd_file_path, map_topic;
    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("map_topic", map_topic, "point_map");

    init_tf_params(nh);

    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);

    file_list_.push_back(pcd_file_path);

    auto pc_msg = CreatePcd();
    
    auto out_msg = TransformMap(pc_msg);

    if (out_msg.width != 0) {
		out_msg.header.frame_id = "map";
		pc_map_pub_.publish(out_msg);
	}

}

void MapLoader::init_tf_params(ros::NodeHandle &nh){
    nh.param<float>("x", tf_x_, 0.0);
    nh.param<float>("y", tf_y_, 0.0);
    nh.param<float>("z", tf_z_, 0.0);
    nh.param<float>("roll", tf_roll_, 0.0);
    nh.param<float>("pitch", tf_pitch_, 0.0);
    nh.param<float>("yaw", tf_yaw_, 0.0);
    ROS_INFO_STREAM("x" << tf_x_ <<"y: "<<tf_y_<<"z: "<<tf_z_<<"roll: "
                        <<tf_roll_<<" pitch: "<< tf_pitch_<<"yaw: "<<tf_yaw_);
}

sensor_msgs::PointCloud2 MapLoader::TransformMap(sensor_msgs::PointCloud2 & in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in, *in_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);                 // tl: translation
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*in_pc, *transformed_pc_ptr, tf_m2w);

    SaveMap(transformed_pc_ptr);
    
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_pc_ptr, output_msg);
    return output_msg;
}

void MapLoader::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr){
    pcl::io::savePCDFile("/tmp/transformed_map.pcd", *map_pc_ptr);
}



sensor_msgs::PointCloud2 MapLoader::CreatePcd()
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : file_list_) {
		// Following outputs are used for progress bar of Runtime Manager.
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;
	}

	return pcd;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}
