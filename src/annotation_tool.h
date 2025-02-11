#ifndef ANNOTATION_TOOL_H
#define ANNOTATION_TOOL_H

#include <QWidget>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QLineEdit>
#include <QInputDialog>

#include <boost/filesystem.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>

#define COLOR true; //true for colored point cloud
#ifdef COLOR
typedef pcl::PointXYZRGB PCType;
#else
typedef pcl::PointXYZ PCType;
#endif

namespace rviz
{
  class Display;
  class RenderPanel;
  class VisualizationManager;
} // namespace rviz

class AnnotationTool : public QWidget
{
  Q_OBJECT
public:
  AnnotationTool(QWidget *parent = 0);
  virtual ~AnnotationTool();

public:
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg);
  void make6DofMarker(std::string name, unsigned int interaction_mode, bool show_6dof);
  void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void PublishPointCloud(pcl::PointCloud<PCType>::Ptr cloud);
  void loadPointCloud();

private Q_SLOTS:
  void loadPointCloudDir();
  void addMarker();
  void removeMarker();
  void saveAnnotation();
  void moveToFrame();
  void loadAnnotation();
  void setLabel();
  void splitName(std::string, std::string, std::string &, std::string &);

private:
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;

  QLineEdit *move_to_frame;
  QLineEdit *set_label;

  //parameters
  pcl::PointCloud<PCType>::Ptr current_cloud;
  // visualization_msgs::marker *current_mesh;
  int num_marker;
  float pre_marker_x;
  float pre_marker_y;
  float pre_marker_z;
  float pre_marker_qx;
  float pre_marker_qy;
  float pre_marker_qz;
  float pre_marker_qw;
  int num_annotated_cloud;
  int pose_label;
  float marker_scale;

  std::string marker_mesh_resource;
  std::string current_marker_type;
  std::string base_dir;
  std::vector<std::string> files;

  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  std::vector<std::vector<float>> label;
  ros::Publisher marker_pub;
  ros::Publisher pointcloud_dataset_pub;
  ros::Publisher mesh_pub;
  ros::Subscriber pointcloud_dataset_sub;
};

#endif // ANNOTATION_TOOL_H
