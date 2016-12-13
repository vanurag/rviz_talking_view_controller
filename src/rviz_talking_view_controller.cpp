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

static const float PITCH_START = Ogre::Math::HALF_PI / 2.0;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5;
static const float DISTANCE_START = 10;

namespace rviz_talking_view_controller
{

using namespace rviz;

TalkingViewController::TalkingViewController()
  : nh_(""), dragging_( false )
{
  distance_property_ = new FloatProperty( "Distance", DISTANCE_START, "Distance from the focal point.", this );
  distance_property_->setMin( 0.01 );

  yaw_property_ = new FloatProperty( "Yaw", YAW_START, "Rotation of the camera around the Z (up) axis.", this );

  pitch_property_ = new FloatProperty( "Pitch", PITCH_START, "How much the camera is tipped downward.", this );
  pitch_property_->setMax( Ogre::Math::HALF_PI - 0.001 );
  pitch_property_->setMin( -pitch_property_->getMax() );

  focal_point_property_ = new VectorProperty( "Focal Point", Ogre::Vector3::ZERO, "The center point which the camera orbits.", this );

  freeview_enabled_property_ = new BoolProperty("Freeview Enabled", true, "Enables mouse control of the reconstruction view.", this);
  cli_abort_property_ = new BoolProperty("Abort CLI", false, "Aborts CLI script.", this);
  cli_pause_property = new BoolProperty("Pause CLI", true, "Pauses CLI script.", this);
  cli_save_mesh_property = new BoolProperty("Save Mesh", false, "Saves scene as mesh.", this);

  cli_engine_msg_.rviz_pose.header.frame_id = "world";
  cli_engine_msg_.rviz_pose.child_frame_id = "rviz_view";
  cli_engine_msg_.abort = false;
  cli_engine_msg_.freeview_enabled = true;
  pub_cli_ = nh_.advertise<CLIEngine>("itm/cli/config", 1);
}

void TalkingViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );

  focal_shape_ = new Shape(Shape::Sphere, context_->getSceneManager(), target_scene_node_);
  focal_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
  focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_shape_->getRootNode()->setVisible(false);
}

TalkingViewController::~TalkingViewController()
{
  delete focal_shape_;
}

void TalkingViewController::reset()
{
  dragging_ = false;
  yaw_property_->setFloat( YAW_START );
  pitch_property_->setFloat( PITCH_START );
  distance_property_->setFloat( DISTANCE_START );
  focal_point_property_->setVector( Ogre::Vector3::ZERO );
  freeview_enabled_property_->setBool(true);
  cli_abort_property_->setBool(false);
  cli_pause_property->setBool(true);
  cli_save_mesh_property->setBool(false);
}

void TalkingViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if ( event.shift() )
  {
    setStatus( "<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z.  <b>Mouse Wheel:</b>: Zoom.  " );
  }
  else
  {
    setStatus( "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click/Mouse Wheel:</b>: Zoom.  <b>Shift</b>: More options." );
  }

  float distance = distance_property_->getFloat();

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  bool moved = false;

  if( event.type == QEvent::MouseButtonPress )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
    dragging_ = true;
  }
  else if( event.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
    dragging_ = false;
  }
  else if( dragging_ && event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  // regular left-button drag
  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    yaw( diff_x*0.005 );
    pitch( -diff_y*0.005 );
  }
  // middle or shift-left drag
  else if( event.middle() || (event.shift() && event.left()) )
  {
    setCursor( MoveXY );
    float fovY = camera_->getFOVy().valueRadians();
    float fovX = 2.0f * atan( tan( fovY / 2.0f ) * camera_->getAspectRatio() );

    int width = camera_->getViewport()->getActualWidth();
    int height = camera_->getViewport()->getActualHeight();

    move( -((float)diff_x / (float)width) * distance * tan( fovX / 2.0f ) * 2.0f,
          ((float)diff_y / (float)height) * distance * tan( fovY / 2.0f ) * 2.0f,
          0.0f );
  }
  else if( event.right() )
  {
    if( event.shift() )
    {
      // move in z direction
      setCursor( MoveZ );
      move(0.0f, 0.0f, diff_y * 0.1 * (distance / 10.0f));
    }
    else
    {
      // zoom
      setCursor( Zoom );
      zoom( -diff_y * 0.1 * (distance / 10.0f) );
    }
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  moved = true;

  if( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    if( event.shift() )
    {
      move( 0, 0, -diff * 0.001 * distance );
    }
    else
    {
      zoom( diff * 0.001 * distance );
    }

    moved = true;
  }

  if( moved )
  {
    context_->queueRender();
  }
}

void TalkingViewController::mimic( ViewController* source_view )
{
  FramePositionTrackingViewController::mimic( source_view );

  Ogre::Camera* source_camera = source_view->getCamera();
  Ogre::Vector3 position = source_camera->getPosition();
  Ogre::Quaternion orientation = source_camera->getOrientation();

  if( source_view->getClassId() == "rviz_talking_view_controller/Talking" )
  {
    // If I'm initializing from another instance of this same class, get the distance exactly.
    distance_property_->setFloat( source_view->subProp( "Distance" )->getValue().toFloat() );
  }
  else
  {
    // Determine the distance from here to the reference frame, and use
    // that as the distance our focal point should be at.
    distance_property_->setFloat( position.length() );
  }

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_property_->getFloat() );
  focal_point_property_->setVector( position + direction );

  calculatePitchYawFromPosition( position );
}

void TalkingViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  updateCamera();
  cli_engine_msg_.abort = cli_abort_property_->getBool();
  cli_engine_msg_.pause = cli_pause_property->getBool();
  cli_engine_msg_.save_mesh = cli_save_mesh_property->getBool();
  cli_engine_msg_.freeview_enabled = freeview_enabled_property_->getBool();
  pub_cli_.publish(cli_engine_msg_);
  // default back to not saving mesh
  cli_save_mesh_property->setBool(false);
}

void TalkingViewController::lookAt( const Ogre::Vector3& point )
{
  Ogre::Vector3 camera_position = camera_->getPosition();
  focal_point_property_->setVector( target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition()) );
  distance_property_->setFloat( focal_point_property_->getVector().distance( camera_position ));

  calculatePitchYawFromPosition(camera_position);
}

void TalkingViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  focal_point_property_->add( old_reference_position - reference_position_ );
}

void TalkingViewController::updateCamera()
{
  float distance = distance_property_->getFloat();
  float yaw = yaw_property_->getFloat();
  float pitch = pitch_property_->getFloat();

  Ogre::Vector3 focal_point = focal_point_property_->getVector();

  float x = distance * cos( yaw ) * cos( pitch ) + focal_point.x;
  float y = distance * sin( yaw ) * cos( pitch ) + focal_point.y;
  float z = distance *              sin( pitch ) + focal_point.z;

  Ogre::Vector3 pos( x, y, z );

  camera_->setPosition(pos);
  camera_->setFixedYawAxis(true, target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
  camera_->setDirection(target_scene_node_->getOrientation() * (focal_point - pos));

  focal_shape_->setPosition( focal_point );

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
  yaw_property_->setFloat( mapAngleTo0_2Pi( yaw_property_->getFloat() - angle ));
}

void TalkingViewController::pitch( float angle )
{
  pitch_property_->add( -angle );
}

void TalkingViewController::calculatePitchYawFromPosition( const Ogre::Vector3& position )
{
  Ogre::Vector3 diff = position - focal_point_property_->getVector();
  pitch_property_->setFloat( asin( diff.z / distance_property_->getFloat() ));
  yaw_property_->setFloat( atan2( diff.y, diff.x ));
}

void TalkingViewController::zoom( float amount )
{
  distance_property_->add( -amount );
}

void TalkingViewController::move( float x, float y, float z )
{
  focal_point_property_->add( camera_->getOrientation() * Ogre::Vector3( x, y, z ));
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_talking_view_controller::TalkingViewController, rviz::ViewController )
