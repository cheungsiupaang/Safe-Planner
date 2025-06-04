#ifndef TRAJECTORY_VISUAL_H
#define TRAJECTORY_VISUAL_H

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace planning_rviz_plugins {
class TrajectoryVisual {
 public:
  TrajectoryVisual(Ogre::SceneManager *scene_manager,
                   Ogre::SceneNode *parent_node);

  virtual ~TrajectoryVisual();

  void setMessage(const planning_ros_msgs::Trajectory &msg);

  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setNum(int n);
  void setYawNum(int n);
  void setPosColor(float r, float g, float b, float a);
  void setVelColor(float r, float g, float b, float a);
  void setAccColor(float r, float g, float b, float a);
  void setJrkColor(float r, float g, float b, float a);

  void setPosColor_quad(float r, float g, float b, float a);

  void setYawColor(float r, float g, float b, float a);
  void setPosScale(float s);
  void setVelScale(float s);
  void setAccScale(float s);
  void setJrkScale(float s);
  void setYawScale(float s);
  void setYawTriangleScale(float s);
  void setYawTriangleAngle(float a);
  void setVelVis(bool vis);
  void setAccVis(bool vis);
  void setJrkVis(bool vis);
  void setYawVis(bool vis);
  void setPosVis_quad(bool vis);
  void setVelVis_quad(bool vis);
  void setAccVis_quad(bool vis);
  void setJrkVis_quad(bool vis);
  void setCableVis_quad(bool vis);

 private:
  std::vector<std::unique_ptr<rviz::BillboardLine>> poss_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> vels_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> accs_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> jrks_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> yaws_;
  
  //@note:for quadrotor
  std::vector<std::unique_ptr<rviz::BillboardLine>> quad_poss_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> quad_vels_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> quad_accs_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> quad_jrks_;

  std::vector<std::unique_ptr<rviz::BillboardLine>> cable_;

  Ogre::SceneNode *frame_node_;
  Ogre::SceneManager *scene_manager_;

  int num_;
  int yaw_num_;
  decimal_t dyaw_{M_PI / 6};
  decimal_t syaw_{0.25};
  bool vel_vis_;
  bool acc_vis_;
  bool jrk_vis_;
  bool yaw_vis_;

  //@note:for quadrotor
  bool quad_pos_vis_;
  bool quad_vel_vis_;
  bool quad_acc_vis_;
  bool quad_jrk_vis_;
  bool quad_cable_vis_;
};
}  // namespace planning_rviz_plugins

#endif
