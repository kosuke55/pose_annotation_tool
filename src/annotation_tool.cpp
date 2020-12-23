#include "annotation_tool.h"

AnnotationTool::AnnotationTool(QWidget *parent)
    : QWidget(parent), current_cloud(new pcl::PointCloud<PCType>)
{
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QHBoxLayout *H_layout = new QHBoxLayout;
  QWidget *HR_widget = new QWidget;
  QGridLayout *subG_layout = new QGridLayout();

  QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
  spLeft.setHorizontalStretch(3);
  render_panel_->setSizePolicy(spLeft);
  H_layout->addWidget(render_panel_);
  QSizePolicy spRight(QSizePolicy::Preferred, QSizePolicy::Preferred);
  spRight.setHorizontalStretch(1);
  HR_widget->setSizePolicy(spRight);
  H_layout->addWidget(HR_widget);


  QVBoxLayout *HRV_layout = new QVBoxLayout;
  QPushButton *add_marker_button = new QPushButton("add marker", this);
  QPushButton *remove_marker_button = new QPushButton("remove marker", this);
  QPushButton *load_annotation_button = new QPushButton("load annotation", this);
  QPushButton *save_annotation_button = new QPushButton("save label and move to next frame", this);
  QPushButton *load_point_cloud_button = new QPushButton("load point cloud directory", this);
  QPushButton *move_to_frame_button = new QPushButton("move to frame", this);
  QPushButton *set_label_button = new QPushButton("set label", this);

  move_to_frame = new QLineEdit;
  QLabel *frame_num_label = new QLabel("frame No. : ");
  set_label = new QLineEdit;
  QLabel *pose_qlabel = new QLabel("label : ");

  subG_layout->addWidget(pose_qlabel, 0, 0);
  subG_layout->addWidget(set_label, 0, 1);
  subG_layout->addWidget(set_label_button, 0, 2);

  subG_layout->addWidget(frame_num_label, 1, 0);
  subG_layout->addWidget(move_to_frame, 1, 1);
  subG_layout->addWidget(move_to_frame_button, 1, 2);

  HRV_layout->addWidget(load_point_cloud_button);
  HRV_layout->addWidget(add_marker_button);
  HRV_layout->addWidget(remove_marker_button);
  HRV_layout->addWidget(save_annotation_button);
  HRV_layout->addWidget(load_annotation_button);
  HRV_layout->addLayout(subG_layout);
  HR_widget->setLayout(HRV_layout);

  setLayout(H_layout);

  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->setFixedFrame("annotation");
  manager_->initialize();
  manager_->startUpdate();

  rviz::Display *interactive_marker = manager_->createDisplay("rviz/InteractiveMarkers", "marker", true);
  rviz::Display *fix_marker = manager_->createDisplay("rviz/MarkerArray", "markerarray", true);
  rviz::Display *mesh_data = manager_->createDisplay("rviz/Marker", "marker", true);
  rviz::Display *pointcloud_data = manager_->createDisplay("rviz/PointCloud2", "pointcloud", true);

  interactive_marker->subProp("Update Topic")->setValue("/Annotation_tool/update");
  interactive_marker->initialize(manager_);
  interactive_marker->setEnabled(true);

  //tool manager is needed for interactive marker
  rviz::ToolManager *tool_manager = manager_->getToolManager();
  tool_manager->initialize();
  rviz::Tool *interact_tool = tool_manager->addTool("rviz/Interact");
  tool_manager->setCurrentTool(interact_tool);

  fix_marker->subProp("Marker Topic")->setValue("/visualization_marker");
  mesh_data->subProp("Marker Topic")->setValue("/dataset_mesh");

  pointcloud_data->subProp("Topic")->setValue("dataset_points");
  pointcloud_data->subProp("Style")->setValue("Points");
  pointcloud_data->subProp("Size (Pixels)")->setValue("2");
#ifdef COLOR
  pointcloud_data->subProp("Color Transformer")->setValue("RGB8");
#else
  pointcloud_data->subProp("Color Transformer")->setValue("Intensity");
#endif
  pointcloud_data->subProp("Invert Rainbow")->setValue("true");

  connect(load_point_cloud_button, SIGNAL(released()), this, SLOT(loadPointCloudDir()));
  connect(add_marker_button, SIGNAL(released()), this, SLOT(addMarker()));
  connect(remove_marker_button, SIGNAL(released()), this, SLOT(removeMarker()));
  connect(save_annotation_button, SIGNAL(released()), this, SLOT(saveAnnotation()));
  connect(move_to_frame_button, SIGNAL(released()), this, SLOT(moveToFrame()));
  connect(load_annotation_button, SIGNAL(released()), this, SLOT(loadAnnotation()));
  connect(set_label_button, SIGNAL(released()), this, SLOT(setLabel()));

  //ros stuff
  server.reset(new interactive_markers::InteractiveMarkerServer("Annotation_tool", "", false));
  marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  pointcloud_dataset_pub = nh_.advertise<sensor_msgs::PointCloud2>("dataset_points", 1);
  mesh_pub = nh_.advertise<visualization_msgs::Marker>("dataset_mesh", 1);

  //init
  num_marker = 0;
  pose_label = 0;
  pre_marker_x = 0;
  pre_marker_y = 0;
  pre_marker_z = 0;
  pre_marker_qx = 0;
  pre_marker_qy = 0;
  pre_marker_qz = 0;
  pre_marker_qw = 1;
  marker_mesh_resource = "package://annotation_tool/axis.stl";
  marker_scale = 0.1;
}

// Destructor.
AnnotationTool::~AnnotationTool()
{
  delete manager_;
}

void AnnotationTool::addMarker()
{

  std::ostringstream nummarker_to_string;
  nummarker_to_string << num_marker;
  std::string marker_name = nummarker_to_string.str();
  std::ostringstream pose_label_to_string;
  pose_label_to_string << pose_label;
  std::string pose_label_str = pose_label_to_string.str();
  marker_name += "_" + pose_label_str;

  make6DofMarker(marker_name, visualization_msgs::InteractiveMarkerControl::MOVE_3D, true);

  server->applyChanges();
  num_marker++;
  float pos[] = {pre_marker_x, pre_marker_y, pre_marker_z,
                 pre_marker_qw, pre_marker_qx, pre_marker_qy, pre_marker_qz, pose_label};
  label.push_back(std::vector<float>(pos, pos + sizeof(pos) / sizeof(float)));
}

visualization_msgs::Marker AnnotationTool::makeBox(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = marker_mesh_resource;
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = 0.1 * marker_scale;
  marker.scale.y = 0.1 * marker_scale;
  marker.scale.z = 0.1 * marker_scale;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::InteractiveMarkerControl &AnnotationTool::makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void AnnotationTool::make6DofMarker(std::string name, unsigned int interaction_mode, bool show_6dof)
{
  tf::Quaternion quaternion = tf::Quaternion(pre_marker_qx, pre_marker_qy, pre_marker_qz, pre_marker_qw);
  tf::Matrix3x3 m(quaternion);
  tf::Vector3 position = tf::Vector3(pre_marker_x,pre_marker_y, pre_marker_z) + m.getColumn(2) * 0.01;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "annotation";
  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg(quaternion, int_marker.pose.orientation);

  int_marker.scale = 0.005; //adjust this to adjust the control box's size
  int_marker.name = name;
  int_marker.description = name;
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&AnnotationTool::markerFeedback, this, _1));
}

void AnnotationTool::removeMarker()
{
  if (num_marker > 0)
  {
    std::vector<visualization_msgs::Marker> vMarker;
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < num_marker; i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "annotation";
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = i;
      vMarker.push_back(marker);
    }
    marker_array.markers = vMarker;
    marker_pub.publish(marker_array);
    if (server->size() > 0)
    {
      server->clear();
      server->applyChanges();
    }
    label.clear();
    num_marker = 0;
  }
}

void AnnotationTool::splitName(std::string s, std::string delimiter, std::string &first, std::string &second)
{
  size_t pos = s.find(delimiter);
  first = s.substr(0, pos);
  second = s.substr(pos + delimiter.length(), s.length());
}

void AnnotationTool::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // int num = num_marker - 1;
  std::string num_string, pose_label_string;
  splitName(feedback->marker_name, "_", num_string, pose_label_string);
  int num = std::atoi(num_string.c_str());

  label[num][0] = feedback->pose.position.x;
  label[num][1] = feedback->pose.position.y;
  label[num][2] = feedback->pose.position.z;
  label[num][3] = feedback->pose.orientation.w;
  label[num][4] = feedback->pose.orientation.x;
  label[num][5] = feedback->pose.orientation.y;
  label[num][6] = feedback->pose.orientation.z;
  label[num][7] = std::atoi(pose_label_string.c_str());

  pre_marker_x = label[num][0];
  pre_marker_y = label[num][1];
  pre_marker_z = label[num][2];
  pre_marker_qx = label[num][4];
  pre_marker_qy = label[num][5];
  pre_marker_qz = label[num][6];
  pre_marker_qw = label[num][3];
}

void AnnotationTool::PublishPointCloud(pcl::PointCloud<PCType>::Ptr cloud)
{
  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*cloud, pc_msg);
  pc_msg.header.frame_id = "annotation";
  pointcloud_dataset_pub.publish(pc_msg);
}


void AnnotationTool::loadPointCloudDir()
{
  QString filename = QFileDialog::getExistingDirectory();
  QDir *dir = new QDir(filename);
  QStringList filter;
  QList<QFileInfo> *fileInfo = new QList<QFileInfo>(dir->entryInfoList(filter));
  visualization_msgs::Marker current_mesh;
  for (int i = 0; i < fileInfo->count(); i++)
  {
    std::string file_path = fileInfo->at(i).filePath().toStdString();
    std::string suffix = file_path.substr(file_path.size() - 3);
    if (suffix == "ply" || suffix == "pcd" || suffix == "obj")
    {
      files.push_back(file_path);
    }
  }
  if (files.size() > 0)
  {
    std::string suffix = files[0].substr(files[0].size() - 3);
    if (suffix == "ply")
    {
      pcl::PLYReader Reader;
      Reader.read(files[0], *current_cloud);
      PublishPointCloud(current_cloud);
    }
    else if (suffix == "pcd")
    {
      pcl::io::loadPCDFile(files[0], *current_cloud);
      PublishPointCloud(current_cloud);
    }
    else if (suffix == "obj"){
      current_mesh.header.frame_id = "annotation";
      current_mesh.header.stamp = ros::Time::now();
      current_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
      current_mesh.mesh_resource = "file://" + files[0];
      current_mesh.action = visualization_msgs::Marker::ADD;
      current_mesh.scale.x = 1;
      current_mesh.scale.y = 1;
      current_mesh.scale.z = 1;
      current_mesh.color.a = 0.5;
      current_mesh.color.r = 0.5;
      current_mesh.color.g = 0.5;
      current_mesh.color.b = 0.5;
      mesh_pub.publish(current_mesh);
    }

    num_annotated_cloud = 0;
  }
}

void AnnotationTool::loadPointCloud()
{
  std::string path = files[num_annotated_cloud];
  std::string suffix = path.substr(path.size() - 3);
  visualization_msgs::Marker current_mesh;
  if (suffix == "ply")
  {
    pcl::PLYReader Reader;
    Reader.read(path, *current_cloud);
    PublishPointCloud(current_cloud);
  }
  else if (suffix == "pcd")
  {
    pcl::io::loadPCDFile(path, *current_cloud);
    PublishPointCloud(current_cloud);
  }
  else if (suffix == "obj")
  {
    current_mesh.header.frame_id = "annotation";
    current_mesh.header.stamp = ros::Time::now();
    current_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    current_mesh.mesh_resource =  "file://" + path;
    current_mesh.action = visualization_msgs::Marker::ADD;
    current_mesh.scale.x = 1;
    current_mesh.scale.y = 1;
    current_mesh.scale.z = 1;
    current_mesh.color.a = 0.5;
    current_mesh.color.r = 0.5;
    current_mesh.color.g = 0.5;
    current_mesh.color.b = 0.5;
    mesh_pub.publish(current_mesh);
  }
}

void AnnotationTool::saveAnnotation()
{
  std::string pc_path = files[num_annotated_cloud];
  std::string txt_path = pc_path.replace(pc_path.end() - 3, pc_path.end(), "txt");
  std::ofstream file;
  file.open(txt_path.c_str());
  for (int i = 0; i < label.size(); i++)
  {
    std::cout << "label:" << label[i][7] << " x: " << label[i][0] << " y: " << label[i][1] << " z: " << label[i][2] << " w: " << label[i][3] << " x: " << label[i][4] << " y: " << label[i][5] << " z: " << label[i][6] << std::endl;
    file << label[i][7] << " " << label[i][0] << " " << label[i][1] << " " << label[i][2] << " " << label[i][3] << " " << label[i][4] << " " << label[i][5] << " " << label[i][6] << std::endl;
  }
  file.close();
  num_annotated_cloud += 1;
  loadPointCloud();
  removeMarker();
}

void AnnotationTool::moveToFrame()
{
  if (!move_to_frame->text().isEmpty())
    num_annotated_cloud = move_to_frame->text().toInt();
  loadPointCloud();
}

void AnnotationTool::setLabel()
{
  if (!set_label->text().isEmpty())
    pose_label = set_label->text().toInt();
}

void AnnotationTool::loadAnnotation()
{
  if (files.size() == 0)
    return;
  std::string pc_path = files[num_annotated_cloud];
  std::string file_path = pc_path.replace(pc_path.end() - 3, pc_path.end(), "txt");
  std::ifstream file;

  std::vector<std::vector<float>> pose;
  int counter = 0;
  file.open(file_path.c_str(), std::ios_base::in);
  std::string line;
  std::vector<int> pose_labels;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::vector<float> element;
    bool is_label = true;
    while (getline(ss, line, ' '))
    {
      if (is_label)
      {
        pose_labels.push_back(std::atoi(line.c_str()));
        is_label = false;
        continue;
      }
      element.push_back(std::atof(line.c_str()));
    }
    pose.push_back(element); // x y z qw qx qy qz

    counter++;
  }

  for (int i = 0; i < counter; i++)
  {
    tf::Vector3 position = tf::Vector3(pose[i][0], pose[i][1], pose[i][2]);
    tf::Quaternion quaternion = tf::Quaternion(pose[i][4], pose[i][5], pose[i][6], pose[i][3]);
    float pos[] = {pose[i][0], pose[i][1], pose[i][2],
                   pose[i][3], pose[i][4], pose[i][5], pose[i][6], float(pose_labels[i])};
    label.push_back(std::vector<float>(pos, pos + sizeof(pos) / sizeof(float)));
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "annotation";
    tf::pointTFToMsg(position, int_marker.pose.position);
    tf::quaternionTFToMsg(quaternion, int_marker.pose.orientation);

    int_marker.scale = 0.005; //adjust this to adjust the control box's size
    int_marker.name = std::to_string(num_marker) + "_" + std::to_string(pose_labels[i]);
    int_marker.description = std::to_string(num_marker) + "_" + std::to_string(pose_labels[i]);
    num_marker++;
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, boost::bind(&AnnotationTool::markerFeedback, this, _1));
    server->applyChanges();
  }
}
