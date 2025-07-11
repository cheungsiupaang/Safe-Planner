#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <planning_ros_msgs/Trajectory.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include "trajectory_visual.h"

namespace planning_rviz_plugins {
class TrajectoryDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::Trajectory> {
  Q_OBJECT
 public:
  TrajectoryDisplay();
  virtual ~TrajectoryDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updatePosColorAndAlpha();
  void updateVelColorAndAlpha();
  void updateAccColorAndAlpha();
  void updateJrkColorAndAlpha();

  void updatePosColorAndAlpha_quad();
  void updateYawColorAndAlpha();
  void updatePosScale();
  void updateVelScale();
  void updateAccScale();
  void updateJrkScale();
  void updateYawScale();
  void updateYawTriangleScale();
  void updateYawTriangleAngle();
  void updateVelVis();
  void updateAccVis();
  void updateJrkVis();

  //@note: for quadrotor
  void updatePosVis_quad();
  void updateVelVis_quad();
  void updateAccVis_quad();
  void updateJrkVis_quad();
  void updateCableVis_quad();

  void updateYawVis();
  void updateNum();
  void updateYawNum();

 private:
  void processMessage(const planning_ros_msgs::Trajectory::ConstPtr &msg);
  void visualizeMessage();

  std::shared_ptr<TrajectoryVisual> visual_;

  rviz::ColorProperty *pos_color_property_;
  rviz::ColorProperty *vel_color_property_;
  rviz::ColorProperty *acc_color_property_;
  rviz::ColorProperty *jrk_color_property_;

  rviz::ColorProperty *pos_color_property_quad;
  rviz::ColorProperty *yaw_color_property_;
  rviz::FloatProperty *pos_scale_property_;
  rviz::FloatProperty *vel_scale_property_;
  rviz::FloatProperty *acc_scale_property_;
  rviz::FloatProperty *jrk_scale_property_;
  rviz::FloatProperty *yaw_scale_property_;
  rviz::FloatProperty *yaw_triangle_scale_property_;
  rviz::FloatProperty *yaw_triangle_angle_property_;
  rviz::BoolProperty *vel_vis_property_;
  rviz::BoolProperty *acc_vis_property_;
  rviz::BoolProperty *jrk_vis_property_;

  //@note:for quadrotor
  rviz::BoolProperty *quad_pos_vis_property_;
  rviz::BoolProperty *quad_vel_vis_property_;
  rviz::BoolProperty *quad_acc_vis_property_;
  rviz::BoolProperty *quad_jrk_vis_property_;
  rviz::BoolProperty *quad_cable_vis_property_;

  rviz::BoolProperty *yaw_vis_property_;
  rviz::IntProperty *num_property_;
  rviz::IntProperty *yaw_num_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  planning_ros_msgs::Trajectory trajectory_;
};
}  // namespace planning_rviz_plugins
