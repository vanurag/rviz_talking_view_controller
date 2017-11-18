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

#include <stdint.h>

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include "rviz/display_context.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

#include "rviz_talking_view_controller/rviz_talking_view_controller.h"

namespace rviz_talking_view_controller
{

using namespace rviz;

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
    Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Y ) *
    Ogre::Quaternion( Ogre::Radian( -Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_Z );

TalkingViewController::TalkingViewController()
  : nh_("")
{
  yaw_property_ = new FloatProperty( "Yaw", 0, "Rotation of the camera around the Y axis.", this );
  pitch_property_ = new FloatProperty( "Pitch", 0, "How much the camera is tipped downward.", this );
  roll_property_ = new FloatProperty( "Roll", Ogre::Math::HALF_PI, "Rotation of the camera around the Z axis.", this );

  pitch_property_->setMax( 0.001 );
  pitch_property_->setMin( -Ogre::Math::PI + 0.001 );
  yaw_property_->setMax( Ogre::Math::HALF_PI - 0.001 );
  yaw_property_->setMin( -yaw_property_->getMax() );
  roll_property_->setMin( 0.001 );
  roll_property_->setMax( Ogre::Math::PI - 0.001 );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3( 0, 0, -10 ), "Position of the camera.", this );

  freeview_enabled_property_ = new BoolProperty("CLI: Freeview Enabled", false, "Enables mouse control of the reconstruction view.", this);
  cli_abort_property_ = new BoolProperty("CLI: Abort", false, "Aborts CLI script.", this);
  cli_pause_property = new BoolProperty("CLI: Pause", true, "Pauses CLI script.", this);
  cli_save_mesh_property = new BoolProperty("CLI: Save Mesh", false, "Saves scene as mesh.", this);
  cli_visualize_scene_property = new BoolProperty("CLI: Visualize", false, "Visualizes scene being reconstructed.", this);
  cli_update_ref_point_property = new BoolProperty("CLI: Update Reference Point", false, "Chooses current point being pointed as reference point to publish.", this);
  cli_update_ref_pattern_property = new BoolProperty("CLI: Update Reference Pattern", false, "Projects point pattern onto surface and generates reference points.", this);
  cli_stop_integration_property = new BoolProperty("CLI: Stop Depth Integration", false, "Stops fusing additional depth info. Only tracking performed.", this);
  cli_relocalize_property = new BoolProperty("CLI: Relocalize", false, "Triggers the relocalization script.", this);
  cli_save_session_property = new BoolProperty("CLI: Save Session", false, "Saves the current session.", this);
  cli_load_session_property = new BoolProperty("CLI: Load Session", false, "Loads the saved session from disk.", this);
  cli_save_waypoints_property = new BoolProperty("CLI: Save Waypoints", false, "Saves the MAV waypoints to disk.", this);
  cli_load_waypoints_property = new BoolProperty("CLI: Load Waypoints", false, "Load the MAV waypoints to disk.", this);

  cli_engine_msg_.rviz_pose.header.frame_id = subProp("Target Frame")->getValue().toString().toStdString();
  cli_engine_msg_.rviz_pose.child_frame_id = "rviz_view";
  cli_engine_msg_.abort = false;
  cli_engine_msg_.freeview_enabled = true;
  pub_cli_ = nh_.advertise<CLIEngine>("itm/cli/config", 1);
}

void TalkingViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
}

TalkingViewController::~TalkingViewController()
{
}

void TalkingViewController::reset()
{
  camera_->setPosition( Ogre::Vector3( 0, 0, -10 ));
  lookAt( Ogre::Vector3(0, 0, 0) );
}

void TalkingViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if ( event.shift() )
  {
    setStatus( "<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else
  {
    setStatus( "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  <b>Shift</b>: More options." );
  }

  bool moved = false;

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if( event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    if (event.control()) {
      roll( diff_x*0.005 );
    } else {
      pitch( diff_y*0.005 );
      yaw( diff_x*0.005 );
    }
  }
  else if( event.middle() || ( event.shift() && event.left() ))
  {
    setCursor( MoveXY );
    move( diff_x*0.01, -diff_y*0.01, 0.0f );
  }
  else if( event.right() )
  {
    setCursor( MoveZ );
    move( 0.0f, 0.0f, diff_y*0.1 );
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  if ( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    move( 0.0f, 0.0f, -diff * 0.01 );

    moved = true;
  }

  if (moved)
  {
    context_->queueRender();
  }
}

void TalkingViewController::setPropertiesFromCamera( Ogre::Camera* source_camera )
{
  Ogre::Quaternion quat = source_camera->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse();
  float yaw = quat.getRoll( false ).valueRadians(); // OGRE camera frame looks along -Z, so they call rotation around Z "roll".
  float pitch = quat.getYaw( false ).valueRadians(); // OGRE camera frame has +Y as "up", so they call rotation around Y "yaw".
  float roll = quat.getRoll( false ).valueRadians();

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    if ( pitch > Ogre::Math::HALF_PI )
    {
      pitch -= Ogre::Math::PI;
    }
    else if ( pitch < -Ogre::Math::HALF_PI )
    {
      pitch += Ogre::Math::PI;
    }

    yaw = -yaw;

    if ( direction.dotProduct( Ogre::Vector3::UNIT_X ) < 0 )
    {
      yaw -= Ogre::Math::PI;
    }
    else
    {
      yaw += Ogre::Math::PI;
    }
  }

  pitch_property_->setFloat( pitch );
  yaw_property_->setFloat( yaw );
  roll_property_->setFloat( roll );
  position_property_->setVector( source_camera->getPosition() );
}

void TalkingViewController::mimic( ViewController* source_view )
{
  FramePositionTrackingViewController::mimic( source_view );
  setPropertiesFromCamera( source_view->getCamera() );
}

void TalkingViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  updateCamera();
  cli_engine_msg_.abort = cli_abort_property_->getBool();
  cli_engine_msg_.pause = cli_pause_property->getBool();
  cli_engine_msg_.save_mesh = cli_save_mesh_property->getBool();
  cli_engine_msg_.visualize_scene = cli_visualize_scene_property->getBool();
  cli_engine_msg_.freeview_enabled = freeview_enabled_property_->getBool();
  cli_engine_msg_.update_reference_point = cli_update_ref_point_property->getBool();
  cli_engine_msg_.update_reference_pattern = cli_update_ref_pattern_property->getBool();
  cli_engine_msg_.stop_integration = cli_stop_integration_property->getBool();
  cli_engine_msg_.relocalize = cli_relocalize_property->getBool();
  cli_engine_msg_.save_session = cli_save_session_property->getBool();
  cli_engine_msg_.load_session = cli_load_session_property->getBool();
  cli_engine_msg_.save_waypoints = cli_save_waypoints_property->getBool();
  cli_engine_msg_.load_waypoints = cli_load_waypoints_property->getBool();
  pub_cli_.publish(cli_engine_msg_);
  // default back to not saving mesh (acts like a button)
  cli_save_mesh_property->setBool(false);
  cli_update_ref_point_property->setBool(false);
  cli_update_ref_pattern_property->setBool(false);
  cli_relocalize_property->setBool(false);
  cli_save_session_property->setBool(false);
  cli_load_session_property->setBool(false);
  cli_save_waypoints_property->setBool(false);
  cli_load_waypoints_property->setBool(false);
}

void TalkingViewController::lookAt( const Ogre::Vector3& point )
{
  camera_->lookAt( point );
  setPropertiesFromCamera( camera_ );
  roll(Ogre::Math::HALF_PI);
}

void TalkingViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  cli_engine_msg_.rviz_pose.header.frame_id = subProp("Target Frame")->getValue().toString().toStdString();
  position_property_->add( old_reference_position - reference_position_ );
}

void TalkingViewController::updateCamera()
{
  camera_->setOrientation( getOrientation() );
  camera_->setPosition( position_property_->getVector() );

  updateRvizViewPose();
}

void TalkingViewController::updateRvizViewPose()
{
  cli_engine_msg_.rviz_pose.header.stamp = ros::Time::now();

  Ogre::Quaternion cam_orientation = camera_->getOrientation();
  Ogre::Vector3 cam_pos = camera_->getPosition();
  if (freeview_enabled_property_->getBool()) {
    cli_engine_msg_.rviz_pose.transform.translation.x = cam_pos.x;
    cli_engine_msg_.rviz_pose.transform.translation.y = cam_pos.y;
    cli_engine_msg_.rviz_pose.transform.translation.z = cam_pos.z;
    cli_engine_msg_.rviz_pose.transform.rotation.x = cam_orientation.x;
    cli_engine_msg_.rviz_pose.transform.rotation.y = cam_orientation.y;
    cli_engine_msg_.rviz_pose.transform.rotation.z = cam_orientation.z;
    cli_engine_msg_.rviz_pose.transform.rotation.w = cam_orientation.w;
  } else {
    cli_engine_msg_.rviz_pose.transform.translation.x = 0;
    cli_engine_msg_.rviz_pose.transform.translation.y = 0;
    cli_engine_msg_.rviz_pose.transform.translation.z = 0;
    cli_engine_msg_.rviz_pose.transform.rotation.x = 0;
    cli_engine_msg_.rviz_pose.transform.rotation.y = 0;
    cli_engine_msg_.rviz_pose.transform.rotation.z = 0;
    cli_engine_msg_.rviz_pose.transform.rotation.w = 0;
  }
}

void TalkingViewController::yaw( float angle )
{
  yaw_property_->add( angle );
}

void TalkingViewController::pitch( float angle )
{
  pitch_property_->add( angle );
}

void TalkingViewController::roll( float angle )
{
  roll_property_->add( angle );
}

Ogre::Quaternion TalkingViewController::getOrientation()
{
  Ogre::Quaternion pitch, yaw, roll;

  yaw.FromAngleAxis( Ogre::Radian( yaw_property_->getFloat() ), Ogre::Vector3::UNIT_X );
  pitch.FromAngleAxis( Ogre::Radian( pitch_property_->getFloat() ), Ogre::Vector3::UNIT_Y );
  roll.FromAngleAxis( Ogre::Radian( roll_property_->getFloat() ), Ogre::Vector3::UNIT_Z );

  return roll* yaw * pitch * ROBOT_TO_CAMERA_ROTATION;
}

void TalkingViewController::move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  position_property_->add( getOrientation() * translate );
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_talking_view_controller::TalkingViewController, rviz::ViewController )
