/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>

namespace velodyne_pointcloud {
    /** @brief Constructor. */
    Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :
            data_(new velodyne_rawdata::RawData()) {
        data_->setup(private_nh);


        // advertise output point cloud (before subscribing to input data)
        output_ =
                node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

        srv_ = boost::make_shared < dynamic_reconfigure::Server < velodyne_pointcloud::
        CloudNodeConfig > > (private_nh);
        dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
        CallbackType f;
        f = boost::bind(&Convert::callback, this, _1, _2);
        srv_->setCallback(f);

        // subscribe to VelodyneScan packets
        velodyne_scan_ =
                node.subscribe("velodyne_packets", 10,
                               &Convert::processScan, (Convert *) this,
                               ros::TransportHints().tcpNoDelay(true));
    }

    void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                           uint32_t level) {
        ROS_INFO("Reconfigure Request");
        data_->setParameters(config.min_range, config.max_range, config.view_direction,
                             config.view_width);
    }

    /** @brief Callback for raw scan messages. */
    void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg) {
        if (output_.getNumSubscribers() == 0)         // no one listening?
            return;                                     // avoid much work

        // allocate a point cloud with same time and frame ID as raw data
        velodyne_rawdata::VPointCloud::Ptr
                outMsg(new velodyne_rawdata::VPointCloud());
        //   velodyne_rawdata::XYZIRBPointCloud::Ptr
        //   outMsg(new velodyne_rawdata::XYZIRBPointCloud());
        // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment

        //outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
        outMsg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        outMsg->header.frame_id = scanMsg->header.frame_id;
        outMsg->height = 1;

        velodyne_rawdata::Cloud::Ptr cloud_pcl(new velodyne_rawdata::Cloud());

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
            data_->unpack(scanMsg->packets[i], *outMsg, *cloud_pcl);
        }

        // publish the accumulated cloud message
        ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                                       << " Velodyne points, time: " << outMsg->header.stamp);

        //Downsample
        velodyne_rawdata::Cloud::Ptr cloud_filtered(new velodyne_rawdata::Cloud());
        pcl::VoxelGrid <velodyne_rawdata::Point> grid;
        grid.setLeafSize(0.02f, 0.02f, 0.02f);
        grid.setInputCloud(cloud_pcl);
        grid.filter(*cloud_filtered);

        //Re convert pointcloud msg
        sensor_msgs::PointCloud2 msg_cloud_filtered_ros;
        pcl::toROSMsg(*cloud_filtered, msg_cloud_filtered_ros);
        msg_cloud_filtered_ros.header = scanMsg->header;

        output_.publish(msg_cloud_filtered_ros);
    }

} // namespace velodyne_pointcloud
