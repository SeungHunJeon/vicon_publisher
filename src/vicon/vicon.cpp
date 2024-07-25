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

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = segment_name;
      pose_msg.pose.position.x = trans.Translation[0];
      pose_msg.pose.position.y = trans.Translation[1];
      pose_msg.pose.position.z = trans.Translation[2];      
      pose_msg.pose.orientation.x = quat.Rotation[0];
      pose_msg.pose.orientation.y = quat.Rotation[1];
      pose_msg.pose.orientation.z = quat.Rotation[2];
      pose_msg.pose.orientation.w = quat.Rotation[3];

      // std::stringstream ss;
      // ss << "Subject: " << subject_name << ", Segment: " << segment_name << ", Translation: "
      //    << trans.Translation[0] << ", " << trans.Translation[1] << ", " << trans.Translation[2];
      // std::string data_str = ss.str();
      // std::cout << data_str << std::endl;
      publishData(pose_msg);
    }
  }
}

ViconPublisherNode::ViconPublisherNode()
  : Node("vicon_publisher_node"), Vicon("192.168.1.5", 10) {
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vicon_pose", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&ViconPublisherNode::timerCallback, this));
  initVicon();
}

void ViconPublisherNode::timerCallback() {
  viconUpdate();
}

void ViconPublisherNode::publishData(const geometry_msgs::msg::PoseStamped& msg) {
  publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
