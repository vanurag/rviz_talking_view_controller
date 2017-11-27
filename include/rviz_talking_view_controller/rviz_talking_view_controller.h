/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_TALKING_VIEW_CONTROLLER_H
#define RVIZ_TALKING_VIEW_CONTROLLER_H

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include "rviz_talking_view_controller/CLIEngine.h"

#include "rviz/frame_position_tracking_view_controller.h"

namespace rviz {
class FloatProperty;
class Shape;
class SceneNode;
class VectorProperty;
}

namespace rviz_talking_view_controller
{

using namespace rviz;
/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class TalkingViewController: public FramePositionTrackingViewController
{
Q_OBJECT
public:
  TalkingViewController();
  virtual ~TalkingViewController();

  /** @brief Do subclass-specific initialization.  Called by
   * ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set. */
  virtual void onInitialize();

  void yaw( float angle );
  void pitch( float angle );
  void roll( float angle );
  void move( float x, float y, float z );

  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  virtual void lookAt( const Ogre::Vector3& point );

  virtual void reset();

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic( ViewController* source_view );

  virtual void update(float dt, float ros_dt);

protected:
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  void updateRvizViewPose();

  void updateCamera();

  Ogre::Quaternion getOrientation();

  FloatProperty* yaw_property_;
  FloatProperty* pitch_property_;
  FloatProperty* roll_property_;
  VectorProperty* position_property_;

  ros::NodeHandle nh_;
  CLIEngine cli_engine_msg_;
  ros::Publisher pub_cli_;

  rviz::BoolProperty* freeview_enabled_property_, *cli_abort_property_;
  rviz::BoolProperty* cli_pause_property, *cli_save_mesh_property, *cli_visualize_scene_property, *cli_visualize_mesh_property;
  rviz::BoolProperty* cli_update_ref_point_property, *cli_update_ref_pattern_property, *cli_stop_integration_property;
  rviz::BoolProperty* cli_relocalize_property, *cli_save_session_property, *cli_load_session_property, *cli_save_waypoints_property, *cli_load_waypoints_property;
};

}

#endif // RVIZ_TALKING_VIEW_CONTROLLER_H
