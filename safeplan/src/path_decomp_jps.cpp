#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include "jps_planner/jps_planner/jps_planner.h"
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_msgs/Path.h>

typedef double decimal_t;
template <int N> using Vecf = Eigen::Matrix<decimal_t, N, 1>;
typedef Vecf<3> Vec3f;

using namespace JPS;

planning_ros_msgs::VoxelMap 
generate_voxel_map(const sensor_msgs::PointCloud& cloud, const float res = 0.1, const Vec3f ori = Vec3f(0, 0, 0), const Vec3i dim = Vec3i(10, 10, 10)){
    planning_ros_msgs::VoxelMap voxelmap;
    voxelmap.header = cloud.header;
    voxelmap.resolution = res;
    geometry_msgs::Point origin;
    origin.x = ori[0]; origin.y = ori[1]; origin.z = ori[2];
    voxelmap.origin = origin;
    geometry_msgs::Point dimension;
    dimension.x = dim[0]; dimension.y = dim[1]; dimension.z = dim[2];
    voxelmap.dim = dimension;
    voxelmap.data.resize(voxelmap.dim.x * voxelmap.dim.y * voxelmap.dim.z, 0);

    for (const auto& pt: cloud.points ){
        Vec3f p(pt.x, pt.y, pt.z);
        Vec3i voxel_p((p[0]-ori[0])/res, (p[1]-ori[1])/res, (p[2]-ori[2])/res);

        if( voxel_p[0] >= 0 && voxel_p[0] < dim[0] && 
            voxel_p[1] >= 0 && voxel_p[1] < dim[1] && 
            voxel_p[2] >= 0 && voxel_p[2] < dim[2] )
            voxelmap.data[voxel_p[0] + voxel_p[1] * dim[0] + voxel_p[2] * dim[0] * dim[1]] = 100;
    }
    return voxelmap;
}

planning_ros_msgs::VoxelMap 
inflate_voxel_map(const planning_ros_msgs::VoxelMap& voxelmap, int inflation_radius) {
    planning_ros_msgs::VoxelMap inflated_map = voxelmap;
    inflated_map.header = voxelmap.header;
    inflated_map.resolution = voxelmap.resolution;
    inflated_map.origin = voxelmap.origin;
    inflated_map.dim = voxelmap.dim;

    std::vector<int8_t>& inflated_data = inflated_map.data;
    const std::vector<int8_t>& voxel_data = voxelmap.data;
    int dim_x = inflated_map.dim.x;
    int dim_y = inflated_map.dim.y;
    int dim_z = inflated_map.dim.z;

    for (int z = 0; z < dim_z; z++) {
        for (int y = 0; y < dim_y; y++) {
            for (int x = 0; x < dim_x; x++) {
                int index = x + y * dim_x + z * dim_x * dim_y;
                if (voxel_data[index] > 0) {
                    inflated_data[index] = 100;

                    for (int dz = -1; dz <= inflation_radius; dz++) {
                        for (int dy = -1; dy <= inflation_radius; dy++) {
                            for (int dx = -1; dx <= inflation_radius; dx++) {
                                int nx = x + dx;
                                int ny = y + dy;
                                int nz = z + dz;
                                if (nx >= 0 && nx < dim_x && ny >= 0 && ny < dim_y && nz >= 0 && nz < dim_z) {
                                    int neighbor_index = nx + ny * dim_x + nz * dim_x * dim_y;
                                    inflated_data[neighbor_index] = 100;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return inflated_map;
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
    ros::Publisher voxel_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
    ros::Publisher inflate_voxel_pub = nh.advertise<planning_ros_msgs::VoxelMap>("inflate_voxel_map", 1, true);

    ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);

    std::string file_name, cloud_topic_name, inflate_cloud_topic_name;
    float start_x, start_y, start_z, goal_x, goal_y, goal_z;
    nh.param("bag_file", file_name, std::string("output.bag"));
    nh.param("cloud_topic", cloud_topic_name, std::string("cloud"));
    nh.param("inflate_cloud_topic", inflate_cloud_topic_name, std::string("inflate_cloud"));
    nh.param("start_x", start_x, 0.1f);
    nh.param("start_y", start_y, 0.1f);
    nh.param("start_z", start_z, 0.1f);
    nh.param("goal_x", goal_x, 5.0f);
    nh.param("goal_y", goal_y, 6.0f);
    nh.param("goal_z", goal_z, 1.0f);

    float res, ori_x, ori_y, ori_z;
    int dim_x, dim_y, dim_z;
    float range_x, range_y, range_z;
    int inflate_radius;
    nh.param("resolution", res, 0.1f);
    nh.param("origin_x", ori_x, 0.0f);
    nh.param("origin_y", ori_y, 0.0f);
    nh.param("origin_z", ori_z, 0.0f);
    nh.param("range_x", range_x, 10.0f);
    nh.param("range_y", range_y, 10.0f);
    nh.param("range_z", range_z, 10.0f);
    nh.param("inflate_radius", inflate_radius, 1);
    printf("inflate_radius: %d\n", inflate_radius);
    printf("resolution: %f\n", res);

    dim_x = int((range_x - ori_x) / res);
    dim_y = int((range_y - ori_y) / res);
    dim_z = int((range_z - ori_z) / res);
    sensor_msgs::PointCloud cloud = read_bag<sensor_msgs::PointCloud>(file_name, cloud_topic_name);
    sensor_msgs::PointCloud inflate_cloud = read_bag<sensor_msgs::PointCloud>(file_name, inflate_cloud_topic_name);
    planning_ros_msgs::VoxelMap voxelmap = generate_voxel_map(cloud, res, Vec3f(ori_x, ori_y, ori_z), Vec3i(dim_x, dim_y, dim_z));
    planning_ros_msgs::VoxelMap inflate_voxelmap = inflate_voxel_map(voxelmap, inflate_radius);
    printf("origin: %f, %f, %f\n", voxelmap.origin.x, voxelmap.origin.y, voxelmap.  origin.z);
    printf("dim: %f, %f, %f\n", voxelmap.dim.x, voxelmap.dim.y, voxelmap.dim.z);
    printf("resolution: %f\n", voxelmap.resolution);
    printf("data size: %d\n", voxelmap.data.size());

    std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
    map_util->setMap( Vec3f(voxelmap.origin.x, voxelmap.origin.y, voxelmap.origin.z), 
                      Vec3i(voxelmap.dim.x, voxelmap.dim.y, voxelmap.dim.z), 
                      inflate_voxelmap.data, inflate_voxelmap.resolution );

    std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true));
    planner_ptr->setMapUtil(map_util);
    planner_ptr->updateMap();

    const Vec3f start(start_x, start_y, start_z);
    const Vec3f goal(goal_x, goal_y, goal_z);
    printf("start: %f, %f, %f\n", start(0), start(1), start(2));
    printf("goal: %f, %f, %f\n", goal(0), goal(1), goal(2));
    planner_ptr->plan(start, goal, 1, true);
    auto path = planner_ptr->getPath();

    path[0] = start;
    path.back() = goal;
    decltype(path) new_path;
    for(size_t i = 0; i < path.size(); i++) {
        if (i == 0)
            new_path.push_back(path[i]);
        else if (i != 0){
            float distance = std::sqrt(std::pow(path[i][0] - path[i-1][0], 2) + std::pow(path[i][1] - path[i-1][1], 2) + std::pow(path[i][2] - path[i-1][2], 2));
            if (distance > 0.3){
                new_path.push_back(path[i]);
                printf("distance: %f\n", distance);
            }
            else
                continue;
        }
    }

    path = new_path;

    cloud.header.frame_id = "map";
    cloud_pub.publish(cloud);

    voxelmap.header.frame_id = "map";
    voxel_pub.publish(voxelmap);

    vec_Vec3f obs = DecompROS::cloud_to_vec(cloud);

    nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);

    //Using ellipsoid decomposition
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(obs);
    decomp_util.set_local_bbox(Vec3f(1.2, 2, 1.6));
    decomp_util.dilate(path);

    //Publish visualization msgs
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);

    // Convert to inequality constraints Ax < b
    auto polys = decomp_util.get_polyhedrons();
    for(size_t i = 0; i < path.size() - 1; i++) {
        const auto pt_inside = (path[i] + path[i+1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
        printf("i: %zu\n", i);
        std::cout << "point: " << path[i].transpose();
        if(cs.inside(path[i]))
            std::cout << " is inside!" << std::endl;
        else
            std::cout << " is outside!" << std::endl;
        std::cout << "point: " << path[i+1].transpose();
        if(cs.inside(path[i+1]))
            std::cout << " is inside!" << std::endl;
        else
            std::cout << " is outside!" << std::endl;
    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        cloud_pub.publish(cloud);
        voxel_pub.publish(voxelmap);
        inflate_voxel_pub.publish(inflate_voxelmap);
        es_pub.publish(es_msg);
        poly_pub.publish(poly_msg);
        path_pub.publish(path_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }

  return 0;
}
