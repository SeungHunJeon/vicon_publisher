//
// Created by oem on 24. 7. 25..
//

// vicon.cpp

#include "vicon/vicon.hpp"

Vicon::Vicon(std::string host_address, int buffer_size) 
  : host_address_(host_address), buffer_size_(buffer_size) {
}

void Vicon::initVicon() {
  while (!client_.IsConnected().Connected) {
    std::cout << "Connecting" << std::endl;
    client_.Connect(host_address_);
  }

  client_.EnableSegmentData();
  client_.EnableMarkerData();
  if (client_.EnableMarkerData().Result != Result::Success) {
    throw std::runtime_error("Failed to enable marker data.");
  }
  if (client_.EnableSegmentData().Result != Result::Success) {
    throw std::runtime_error("Failed to enable segment data.");
  }
  client_.SetStreamMode(StreamMode::ServerPush);
  client_.SetBufferSize(buffer_size_);
}

void Vicon::viconUpdate() {
  auto Output = client_.GetFrame();
  if(Output.Result == Result::Success) {
    Output_GetFrameNumber frame_number = client_.GetFrameNumber();
    unsigned int subject_num = client_.GetSubjectCount().SubjectCount;
    for (unsigned int subject_idx = 0; subject_idx < subject_num; subject_idx++) {
      std::string subject_name = client_.GetSubjectName(subject_idx).SubjectName;
      unsigned int segment_num = client_.GetSegmentCount(subject_name).SegmentCount;
      std::string segment_name = client_.GetSegmentName(subject_name, 0).SegmentName;
      Output_GetTimecode outputTimeCode = client_.GetTimecode();
      Output_GetFrameRate outputFrameRate = client_.GetFrameRate();
      Output_GetSegmentGlobalTranslation trans =
          client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      Output_GetSegmentGlobalRotationMatrix rot =
          client_.GetSegmentGlobalRotationMatrix(subject_name, segment_name);
      Output_GetSegmentGlobalRotationQuaternion quat =
          client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = trans.Translation[0];
      pose_msg.position.y = trans.Translation[1];
      pose_msg.position.z = trans.Translation[2];      
      pose_msg.orientation.x = quat.Rotation[0];
      pose_msg.orientation.y = quat.Rotation[1];
      pose_msg.orientation.z = quat.Rotation[2];
      pose_msg.orientation.w = quat.Rotation[3];

      std::stringstream ss;
      ss << "Subject: " << subject_name << ", Segment: " << segment_name << ", Translation: "
         << trans.Translation[0] << ", " << trans.Translation[1] << ", " << trans.Translation[2];
      std::string data_str = ss.str();
      std::cout << data_str << std::endl;
      publishData(subject_name, pose_msg);
    }
  }
}

void Vicon::publishData(const std::string& name, const geometry_msgs::msg::Pose& msg) {
  if (name == "robot") {
    robot_pose_publisher_->publish(msg);
  }
  else if (name == "object") {
    object_pose_publisher_->publish(msg);
  }
}

ViconPublisherNode::ViconPublisherNode()
  : Node("vicon_publisher_node"), vicon_("localhost:801", 10) {
  robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("vicon_robot_pose", 10);
  object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("vicon_object_pose", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&ViconPublisherNode::timerCallback, this));
  vicon_.initVicon();
}

void ViconPublisherNode::timerCallback() {
  vicon_.viconUpdate();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
