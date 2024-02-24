#include <sqlite_lecture/relation_data.hpp>
#include <sqlite_lecture/file_watcher.hpp>

#include <sqlite3.h>
#include <unistd.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using namespace std::chrono_literals;

class SqliteStore : public rclcpp::Node
{
public:
  SqliteStore()
  : Node("sqlite_store")
  {
    initializeRos();
    initializeDB("data.db");

    printf("start\n");
    updatePoses();
  }

  void onTimer()
  {
    if (file_watcger_.checkIfFileModified()) {
      printf("update\n");
      updatePoses();
    }
  }

private:
  bool initializeRos(void)
  {
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "stored_pose_list", rclcpp::QoS(
        1).transient_local());
    timer_ = this->create_wall_timer(500ms, std::bind(&SqliteStore::onTimer, this));
    return true;
  }

  bool initializeDB(const std::string filename)
  {
    // open DB
    if (sqlite3_open(filename.c_str(), &db_) != SQLITE_OK) {
      printf("%s[%s]\n", __func__, sqlite3_errmsg(db_));
      return false;
    }
    // create Table
    if (sqlite3_exec(
        db_, "CREATE TABLE IF NOT EXISTS PositionList (name PRIMARY KEY, x, y);", NULL,
        NULL, NULL) != SQLITE_OK)
    {
      printf("%s[%s]\n", __func__, sqlite3_errmsg(db_));
      return false;
    }
    // initialize inotify
    if (!file_watcger_.setFilePath(filename)) {
      printf("inotify open error\n");
      return false;
    }
    return true;
  }

  void updatePoses(void)
  {
    // get & view record
    RelationData relation_data;
    if (sqlite3_exec(
        db_, "SELECT * FROM PositionList", SqliteStore::sqlExecuteCallback,
        (void *)&relation_data, NULL))
    {
      printf("%s[%s]\n", __func__, sqlite3_errmsg(db_));
      return;
    }
    relation_data.print();

    // convert
    std::vector<geometry_msgs::msg::Pose> poses;
    for (size_t i = 0; i < relation_data.size(); i++) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = std::stod(relation_data.get("x", i));
      pose.position.y = std::stod(relation_data.get("y", i));
      pose.position.z = 0.0f;
      pose.orientation.w = 1.0f;
      poses.push_back(pose);
    }

    // publish Msg
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.poses = poses;
    pose_array_pub_->publish(msg);
  }

  static int sqlExecuteCallback(void * data, int argc, char ** argv, char ** azColName)
  {
    RelationData * relation_data = (RelationData *)data;
    relation_data->increment();
    for (int i = 0; i < argc; i++) {
      relation_data->setAtBack(azColName[i], argv[i]);
    }
    return 0;
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_{nullptr};
  FileWatcher file_watcger_;
  sqlite3 * db_{NULL};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto sqlite_store = std::make_shared<SqliteStore>();
  rclcpp::spin(sqlite_store);
  rclcpp::shutdown();
  return 0;
}
