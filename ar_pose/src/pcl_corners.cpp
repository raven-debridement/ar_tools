/*
 *  Multi Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2010, CCNY Robotics Lab, 2011 ILS Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 * 
 *  Michael Ferguson <ferguson@cs.albany.edu>
 *  http://robotics.ils.albany.edu
 *
 *  Anna Lee <leeanna@berkeley.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> TransformationEstimation;

const double AR_TO_ROS = 0.001;
const double WIDTH = 5.16;

class PclCorners {
  private:
    ros::NodeHandle n_;
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher arMarkerPub_;
    ros::Subscriber arMarkerSub_;
    ros::Publisher rvizMarkerPub_;
    visualization_msgs::Marker rvizMarker_;
    ar_pose::ARMarkers arPoseMarkers_;

  public:
  PclCorners (ros::NodeHandle &n):n_ (n) {
    arMarkerPub_ = n_.advertise < ar_pose::ARMarkers > ("stereo_pose",0);
    arMarkerSub_ = n_.subscribe("stereo_corners", 1, &PclCorners::processMarkers, this);
    
    rvizMarkerPub_ = n_.advertise < visualization_msgs::Marker > ("visualization_marker_stereo", 0);
  }

  btTransform tfFromEigen(Eigen::Matrix4f trans)
  {
    btMatrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
  }

  pcl::PointXYZRGB makePoint( float x, float y, float z )
  {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  void processMarkers(const ar_pose::ARMarkersConstPtr & msg) {
    arPoseMarkers_.markers.clear();
    for (int i = 0; i < msg->markers.size(); i++) {
      processMarker(msg->markers[i]);
    }
    arMarkerPub_.publish( arPoseMarkers_ );
    ROS_DEBUG ("Published ar marker");
  }

  void processMarker(const ar_pose::ARMarker_<std::allocator<void> >& msg) {
    float tlx = msg.corners[0];
    float tly = msg.corners[1];
    float tlz = msg.corners[2];
    float trx = msg.corners[3];
    float tr_y = msg.corners[4];
    float trz = msg.corners[5];
    float brx = msg.corners[6];
    float bry = msg.corners[7];
    float brz = msg.corners[8];
    float blx = msg.corners[9];
    float bly = msg.corners[10];
    float blz = msg.corners[11];

    PointCloud marker;
    marker.push_back( makePoint(tlx, tly, tlz) );
    marker.push_back( makePoint(trx, tr_y, trz) );
    marker.push_back( makePoint(brx, bry, brz) );
    marker.push_back( makePoint(blx, bly, blz) );

    float w = WIDTH;

    PointCloud ideal;
    ideal.push_back( makePoint(-w/2,w/2,0) ); 
    ideal.push_back( makePoint(w/2,w/2,0) );
    ideal.push_back( makePoint(w/2,-w/2,0) );
    ideal.push_back( makePoint(-w/2,-w/2,0) );

    /* get transformation */
      Eigen::Matrix4f t;
      TransformationEstimation obj;
      obj.estimateRigidTransformation( marker, ideal, t );


      /* get final transformation */
      btTransform transform = tfFromEigen(t.inverse());

      // any(transform == nan)
      btMatrix3x3  m = transform.getBasis();
      btVector3    p = transform.getOrigin();
      bool invalid = false;
      for(int i=0; i < 3; i++)
        for(int j=0; j < 3; j++)
          invalid = (invalid || isnan(m[i][j]) || fabs(m[i][j]) > 1.0);

      for(int i=0; i < 3; i++)
          invalid = (invalid || isnan(p[i]));

      if(invalid) {
        ROS_INFO("Invalid transform");
        return;
      }

      ros::Time time = ros::Time::now();

      /* publish the marker */
      ar_pose::ARMarker ar_pose_marker;
      ar_pose_marker.header.frame_id = msg.header.frame_id;
      ar_pose_marker.header.stamp = time;
      ar_pose_marker.id = msg.id;
      ar_pose_marker.confidence = msg.confidence;

      ar_pose_marker.pose.pose.position.x = transform.getOrigin().getX();
      ar_pose_marker.pose.pose.position.y = transform.getOrigin().getY();
      ar_pose_marker.pose.pose.position.z = transform.getOrigin().getZ();

      ar_pose_marker.pose.pose.orientation.x = transform.getRotation().getAxis().getX();
      ar_pose_marker.pose.pose.orientation.y = transform.getRotation().getAxis().getY();
      ar_pose_marker.pose.pose.orientation.z = transform.getRotation().getAxis().getZ();
      ar_pose_marker.pose.pose.orientation.w = transform.getRotation().getW();

      arPoseMarkers_.markers.push_back (ar_pose_marker);

      /* publish transform */
      char name[20];
      sprintf(name, "ar_stereo_%d", msg.id);
      broadcaster_.sendTransform(tf::StampedTransform(transform, time, msg.header.frame_id, name));

      /* publish visual marker */
        btVector3 markerOrigin (0, 0, 0.25 * w * AR_TO_ROS);
        btTransform m2 (btQuaternion::getIdentity (), markerOrigin);
        btTransform markerPose = transform * m2; // marker pose in the camera frame

        tf::poseTFToMsg (markerPose, rvizMarker_.pose);

        rvizMarker_.header.frame_id = msg.header.frame_id;
        rvizMarker_.header.stamp = time;
        rvizMarker_.id = msg.id;

        rvizMarker_.scale.x = 1.0 * w * AR_TO_ROS;
        rvizMarker_.scale.y = 1.0 * w * AR_TO_ROS;
        rvizMarker_.scale.z = 0.5 * w * AR_TO_ROS;
        rvizMarker_.ns = "basic_shapes";
        rvizMarker_.type = visualization_msgs::Marker::CUBE;
        rvizMarker_.action = visualization_msgs::Marker::ADD;
        switch (msg.id)
        {
          case 0:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 1.0f;
            rvizMarker_.color.a = 1.0;
            break;
          case 1:
            rvizMarker_.color.r = 1.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
            break;
          default:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 1.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
        }
        rvizMarker_.lifetime = ros::Duration ();

        rvizMarkerPub_.publish (rvizMarker_);
        ROS_DEBUG ("Published visual marker");
  }
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_corners");
  ros::NodeHandle n;
  PclCorners pcl_corners(n);
  ros::spin ();
  return 0;
}

