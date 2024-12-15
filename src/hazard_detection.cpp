#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <vector>

class HazardDetection {
public:
    HazardDetection(ros::NodeHandle& nh) {
        // Load parameters
        nh.param("safe_distance", safe_distance_, 20.0f); // Safe distance in meters
        nh.param("min_speed", min_speed_, 5.56f);         // Minimum speed in m/s (20 km/h)
        nh.param("radius", radius_, 1.5f);               // Radius for target point proximity

        // Initialize publishers
        hazard_pub_ = nh.advertise<std_msgs::Bool>("/mobinha/hazard_warning", 10);
        circle_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mobinha/circles_viz", 10);
        heading_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/mobinha/heading_markers", 10);

        // Initialize subscribers
        track_box_sub_ = nh.subscribe("/mobinha/perception/lidar/track_box", 10, 
                                      &HazardDetection::trackBoxCallback, this);
        target_points_sub_ = nh.subscribe<visualization_msgs::MarkerArray>(
            "/target_points", 10, &HazardDetection::targetPointsCallback, this);
    }

private:
    ros::Publisher hazard_pub_;
    ros::Publisher circle_viz_pub_;
    ros::Publisher heading_marker_pub_;
    ros::Subscriber track_box_sub_;
    ros::Subscriber target_points_sub_;

    float safe_distance_; // Safe distance in meters
    float min_speed_;     // Minimum speed in m/s
    float radius_;        // Radius for target point proximity
    std::vector<std::pair<float, float>> target_points_; // List of target point coordinates

    // Callback to receive target points and visualize them
    void targetPointsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        target_points_.clear();
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < msg->markers.size(); ++i) {
            const auto& marker = msg->markers[i];
            float x = marker.pose.position.x;
            float y = marker.pose.position.y;
            target_points_.emplace_back(x, y);

            // Create a circular marker for each target point
            visualization_msgs::Marker circle_marker;
            circle_marker.header.frame_id = "ego_car";
            circle_marker.header.stamp = ros::Time::now();
            circle_marker.ns = "target_circles";
            circle_marker.id = i;
            circle_marker.type = visualization_msgs::Marker::CYLINDER;
            circle_marker.action = visualization_msgs::Marker::ADD;
            circle_marker.pose.position.x = x;
            circle_marker.pose.position.y = y;
            circle_marker.pose.position.z = 0.0;
            circle_marker.scale.x = radius_ * 2.0;
            circle_marker.scale.y = radius_ * 2.0;
            circle_marker.scale.z = 0.1;
            circle_marker.color.r = 255.0/255.0f;
            circle_marker.color.g = 216.0/255.0f;
            circle_marker.color.b = 216.0/255.0f;
            circle_marker.color.a = 0.8f;

            marker_array.markers.push_back(circle_marker);
        }

        circle_viz_pub_.publish(marker_array);
        // ROS_INFO("Target points updated and circles visualized: %lu points.", target_points_.size());
    }

    // 헤딩 시각화 함수
    void visualizeHeading(const jsk_recognition_msgs::BoundingBox& box) {
        visualization_msgs::Marker heading_marker;
        heading_marker.header.frame_id = "ego_car"; // Frame ID
        heading_marker.header.stamp = ros::Time::now(); // Timestamp
        heading_marker.ns = "heading_marker"; // Namespace
        heading_marker.id = box.label; // Unique ID for the marker
        heading_marker.type = visualization_msgs::Marker::ARROW; // Arrow marker
        heading_marker.action = visualization_msgs::Marker::ADD; // Add or modify the marker

        heading_marker.scale.x = 0.5;  // Arrow length
        heading_marker.scale.y = 0.2;  // Arrow width
        heading_marker.scale.z = 0.2;  // Arrow height
        heading_marker.color.r = 36.0/255.0f; // Red color
        heading_marker.color.g = 252.0/255.0f;
        heading_marker.color.b = 255.0/255.0f;
        heading_marker.color.a = 1.0f; // Opaque

        // 시작 점 (BoundingBox 중심)
        geometry_msgs::Point start, end;
        start.x = box.pose.position.x;
        start.y = box.pose.position.y;
        start.z = box.pose.position.z;

        // Orientation에서 헤딩 계산 (Yaw 추출)
        tf2::Quaternion quat(box.pose.orientation.x, 
                             box.pose.orientation.y, 
                             box.pose.orientation.z, 
                             box.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // 진행 방향 계산
        float heading_x = std::cos(yaw);
        float heading_y = std::sin(yaw);

        // 끝 점 (헤딩 방향으로 연장)
        end.x = start.x + heading_x * 8.0; // Extend by 8 meters
        end.y = start.y + heading_y * 8.0;
        end.z = start.z;

        heading_marker.points.push_back(start);
        heading_marker.points.push_back(end);

        heading_marker_pub_.publish(heading_marker);
    }

    // 장애물 판단 콜백
    void trackBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) {
        bool hazard_detected = false;
        std_msgs::Bool hazard_msg_speed_distance;
        std_msgs::Bool hazard_msg_proximity;

        // 속도 및 예측 거리 기반 위험 판단
        for (const auto& box : msg->boxes) {
            int cls = box.label;
            if (cls < 1 || cls > 3) continue; // Process only labels 1, 2, 3

            // 헤딩 시각화
            visualizeHeading(box);

            // 객체 위치 및 거리 계산
            float x = box.pose.position.x;
            float y = box.pose.position.y;
            float distance = std::sqrt(x * x + y * y);

            // Orientation에서 Yaw 추출
            tf2::Quaternion quat(box.pose.orientation.x, 
                                 box.pose.orientation.y, 
                                 box.pose.orientation.z, 
                                 box.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // 진행 방향 계산
            float heading_x = std::cos(yaw);
            float heading_y = std::sin(yaw);

            // 상대 속도 추출 
            float relative_speed = box.value;

            // 1초 뒤 예상 위치 계산
            float predicted_x = x + heading_x * relative_speed * 1.0; // speed * time
            float predicted_y = y + heading_y * relative_speed * 1.0;
            float predicted_distance = std::sqrt(predicted_x * predicted_x + predicted_y * predicted_y);

            // 위험 판단
            if (relative_speed >= min_speed_ && predicted_distance <= safe_distance_) {
                hazard_detected = true;
                ROS_WARN("Speed/Distance-based hazard detected! Label: %d, Predicted Distance: %.2f m, Speed: %.2f m/s, Yaw: %.2f rad", 
                         cls, predicted_distance, relative_speed, yaw);
                break; // Exit if any hazard is detected
            }
        }

        // 위험 판단
        if (!hazard_detected) {
            evaluateHazards(*msg, target_points_, radius_, hazard_msg_proximity);
            if (hazard_msg_proximity.data) {
                hazard_detected = true;
            }
        }

        // 최종 위험 판단 결과 퍼블리쉬
        std_msgs::Bool hazard_msg_final;
        hazard_msg_final.data = hazard_detected;
        hazard_pub_.publish(hazard_msg_final);

        if (hazard_detected) {
            ROS_WARN("Hazard detected!");
        } else {
            // ROS_INFO("No hazards detected.");
        }
    }

    // 경로 중심점과의 근접성 기반 위험 판단 함수
    void evaluateHazards(const jsk_recognition_msgs::BoundingBoxArray& track_box,
                         const std::vector<std::pair<float, float>>& target_points,
                         float radius, std_msgs::Bool& hazard_msg) {
        hazard_msg.data = false;

        for (const auto& box : track_box.boxes) {
            float x = box.pose.position.x;
            float y = box.pose.position.y;

            for (const auto& target : target_points) {
                float dx = x - target.first;
                float dy = y - target.second;
                if (std::sqrt(dx * dx + dy * dy) <= radius) {
                    ROS_WARN("Circle - Track Box ! : [%.2f, %.2f]", x, y);
                    hazard_msg.data = true;
                    return; // Exit early if any hazard is detected
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hazard_detection");
    ros::NodeHandle nh("~");

    HazardDetection hazard_detection(nh);

    ROS_INFO("Hazard Detection node has started.");

    ros::spin();

    return 0;
}
