/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltags2_ros/continuous_detector.h"
#include <std_msgs/String.h>

namespace apriltags2_ros
{

ContinuousDetector::ContinuousDetector (ros::NodeHandle& nh,
                                        ros::NodeHandle& pnh) :
    tag_detector_(pnh),
    draw_tag_detections_image_(
        getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false)),
    it_(nh)
{
  switch_sub_ = nh.subscribe("apriltag_detector_node/switch",1,&ContinuousDetector::switchCB, this);
  camera_image_subscriber_ =
      it_.subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  subprocess_timings_publisher_ =
      nh.advertise<std_msgs::String>("subprocess_timings", 1);

  draw_decimated_raw_image_ =
      it_.advertise("decimated_raw_image", 1);

  //on_switch = false;
  on_switch = true;

  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_.advertise("tag_detections_image", 1);
  }
}
void ContinuousDetector::switchCB(const duckietown_msgs::BoolStamped::ConstPtr& switch_msg){
     on_switch=switch_msg->data;
   }

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  if (!on_switch) return;
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTags 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect,
                                    sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTags 2
  cv::Mat decimated_raw_image_;
  tag_detections_publisher_.publish(
      tag_detector_.detectTags(cv_image_,camera_info, decimated_raw_image_));
  sensor_msgs::ImagePtr decimated_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", decimated_raw_image_).toImageMsg();
  draw_decimated_raw_image_.publish(decimated_image_msg);

  subprocess_timings_publisher_.publish(tag_detector_.timings_);
  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_.drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltags2_ros
