#include "modules/localization/shenlan_msf/visualization/pangolin_visualizer.h"

namespace apollo {
namespace localization {

PangolinViewer::PangolinViewer(){
  // create a window and bind its context to the main thread
  pangolin::CreateWindowAndBind(window_name, 1024, 768);

  // enable depth
  glEnable(GL_DEPTH_TEST);

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();

  render_loop = std::thread(&PangolinViewer::Run, this);
}

PangolinViewer::PangolinViewer(const YAML::Node& config_node) {
  // create a window and bind its context to the main thread
  pangolin::CreateWindowAndBind(window_name, 1024, 768);

  // enable depth
  glEnable(GL_DEPTH_TEST);

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();

  render_loop = std::thread(&PangolinViewer::Run, this);

  LoadGlobalMap(config_node);
}

// DestroyWindow
PangolinViewer::~PangolinViewer(){
  // 发送退出指令
  pangolin::Quit();
  // 堵塞线程直到线程结束
  render_loop.join();
  pangolin::DestroyWindow(window_name);
  std::cerr << "DestroyWindow Finished!" << std::endl;
}

bool PangolinViewer::LoadGlobalMap(const YAML::Node& config_node) {
  const std::string map_path_ = config_node["map_path"].as<std::string>();
  const std::string filter_mothod =
      config_node["global_map_filter"].as<std::string>();

  global_map_ptr_.reset(new CloudData::CloudType());
  pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
  AINFO << "Load global map, size:" << global_map_ptr_->points.size();

  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_ = nullptr;
  if (filter_mothod == "voxel_filter") {
    global_map_filter_ptr_ = std::shared_ptr<VoxelFilter>(
        new VoxelFilter(config_node[filter_mothod]["global_map"]));
  } else if (filter_mothod == "no_filter") {
    global_map_filter_ptr_ = std::shared_ptr<NoFilter>(new NoFilter);
  } else {
    AERROR << "Filter method " << filter_mothod << " for global_map NOT FOUND!";
    return false;
  }

  global_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
  AINFO << "Filtered global map, size:" << global_map_ptr_->points.size();

  return true;
}

void PangolinViewer::Run() {
  pangolin::BindToContext(window_name);
  glEnable(GL_BLEND);       // 在OpenGL中使用颜色混合
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // 选择混合选项

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(50, 50, 50, 0, 0, 0, pangolin::AxisY));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -640.0f / 480.0f)
                              .SetHandler(&handler);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menu_gnss_dominant("menu.Follow GNSS", true, true);
  pangolin::Var<bool> menu_lidar_dominant("menu.Follow Lidar", false, true);
  pangolin::Var<bool> menu_fusion_dominant("menu.Follow Fusion", false, true);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    d_cam.Activate(s_cam);


    if (global_map_ptr_ != nullptr){
      glPointSize(2);
      glBegin(GL_POINTS);
      glColor4d(0.0f, 1.0f, 0.0f, 0.8f);
      for (size_t j = 0; j < global_map_ptr_->points.size(); j++) {
        glVertex3d(global_map_ptr_->points[j].x, global_map_ptr_->points[j].y,
                   global_map_ptr_->points[j].z);
      }
      glEnd();
    }

    std::unique_lock<std::mutex> info_lock(info_list_mutex_);
    cur_draw = input_list;

    if (pangolin::Pushed(menu_lidar_dominant)) {
      menu_gnss_dominant = false;
      menu_lidar_dominant = true;
      menu_fusion_dominant = false;
    } else if (pangolin::Pushed(menu_gnss_dominant)) {
      menu_gnss_dominant = true;
      menu_lidar_dominant = false;
      menu_fusion_dominant = false;
    } else if (pangolin::Pushed(menu_fusion_dominant)) {
      menu_gnss_dominant = false;
      menu_lidar_dominant = false;
      menu_fusion_dominant = true;
    }

    CloudData::CloudTypePtr local_map_cloud_tmp(new CloudData::CloudType());
    size_t color_point_threshold = 10;
    if (global_map_ptr_ != nullptr){
      color_point_threshold = 50;
    }

    for (size_t i = 0; i < cur_draw.size(); ++i) {
      auto lidar_pose = cur_draw[i].lidar_pose.pose_;
      auto gnss_pose = cur_draw[i].gnss_pose.pose_;
      auto fusion_pose = cur_draw[i].fusion_pose.pose_;
      auto cloud = cur_draw[i].lidar_frame;

      bool show_flag = false;
      CloudData::CloudTypePtr output_cloud_ptr(new CloudData::CloudType());
      if (menu_gnss_dominant && cur_draw[i].gnss_pose.time_ > 10000) {
        Eigen::Affine3f transform(gnss_pose);
        pcl::transformPointCloud(*cloud.cloud_ptr_, *output_cloud_ptr,
                                 transform);
        show_flag = true;
      } else if (menu_lidar_dominant &&
                 cur_draw[i].lidar_pose.time_ > 10000) {
        Eigen::Affine3f transform(lidar_pose);
        pcl::transformPointCloud(*cloud.cloud_ptr_, *output_cloud_ptr,
                                 transform);
        show_flag = true;
      } else if (menu_fusion_dominant &&
                 cur_draw[i].fusion_pose.time_  > 10000) {
        Eigen::Affine3f transform(fusion_pose);
        pcl::transformPointCloud(*cloud.cloud_ptr_, *output_cloud_ptr,
                                 transform);
        show_flag = true;
      }

      if (show_flag) {
        bool case1 = cur_draw.size() < color_point_threshold;
        bool case2 = (cur_draw.size() > color_point_threshold) &&
                     (i >= cur_draw.size() - color_point_threshold);
        if (case1 || case2) {
          glPointSize(1);
          glBegin(GL_POINTS);
          glColor4f(0.0f, 0.0f, 0.8f, 0.8f);
          for (size_t j = 0; j < (size_t)(output_cloud_ptr->points.size());
               j++) {
            glVertex3d(output_cloud_ptr->points[j].x,
                       output_cloud_ptr->points[j].y,
                       output_cloud_ptr->points[j].z);
          }
          glEnd();
        }else{
          local_map_cloud_tmp->points.insert(local_map_cloud_tmp->end(),
                                             output_cloud_ptr->begin(),
                                             output_cloud_ptr->end());
        }
      }

      Eigen::Isometry3f gnss_T = Eigen::Isometry3f::Identity();
      {
        gnss_T.rotate(gnss_pose.block<3, 3>(0, 0));
        gnss_T.pretranslate(gnss_pose.block<3, 1>(0, 3));
        Eigen::Vector3f Ow = gnss_T.translation();
        Eigen::Vector3f Xw = gnss_T * (0.4 * Eigen::Vector3f(1, 0, 0));
        Eigen::Vector3f Yw = gnss_T * (0.4 * Eigen::Vector3f(0, 1, 0));
        Eigen::Vector3f Zw = gnss_T * (0.4 * Eigen::Vector3f(0, 0, 1));
        glLineWidth(5);
        glBegin(GL_LINE_STRIP);
        glColor3d(0.0, 1.0, 0.0);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Xw[0], Xw[1], Xw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Yw[0], Yw[1], Yw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Zw[0], Zw[1], Zw[2]);
        glEnd();
      }

      Eigen::Isometry3f lidar_T = Eigen::Isometry3f::Identity();
      {
        lidar_T.rotate(lidar_pose.block<3, 3>(0, 0));
        lidar_T.pretranslate(lidar_pose.block<3, 1>(0, 3));
        Eigen::Vector3f Ow = lidar_T.translation();
        Eigen::Vector3f Xw = lidar_T * (0.4 * Eigen::Vector3f(1, 0, 0));
        Eigen::Vector3f Yw = lidar_T * (0.4 * Eigen::Vector3f(0, 1, 0));
        Eigen::Vector3f Zw = lidar_T * (0.4 * Eigen::Vector3f(0, 0, 1));
        glLineWidth(5);
        glBegin(GL_LINES);
        glColor3d(1.0, 0.0, 0.0);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Xw[0], Xw[1], Xw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Yw[0], Yw[1], Yw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Zw[0], Zw[1], Zw[2]);
        glEnd();
      }

      Eigen::Isometry3f fusion_T = Eigen::Isometry3f::Identity();
      {
        fusion_T.rotate(fusion_pose.block<3, 3>(0, 0));
        fusion_T.pretranslate(fusion_pose.block<3, 1>(0, 3));
        Eigen::Vector3f Ow = fusion_T.translation();
        Eigen::Vector3f Xw = fusion_T * (0.4 * Eigen::Vector3f(1, 0, 0));
        Eigen::Vector3f Yw = fusion_T * (0.4 * Eigen::Vector3f(0, 1, 0));
        Eigen::Vector3f Zw = fusion_T * (0.4 * Eigen::Vector3f(0, 0, 1));
        glLineWidth(5);
        glBegin(GL_LINES);
        glColor3d(0.0, 0.0, 1.0);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Xw[0], Xw[1], Xw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Yw[0], Yw[1], Yw[2]);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Zw[0], Zw[1], Zw[2]);
        glEnd();
      }
    }

    cur_draw.clear();

    CloudData::CloudTypePtr local_map_cloud(new CloudData::CloudType());
    pcl::VoxelGrid<CloudData::PointType> sor;
    sor.setInputCloud(local_map_cloud_tmp);
    sor.setLeafSize(0.6f, 0.6f, 0.6f);
    sor.filter(*local_map_cloud);
    glPointSize(2);
    glBegin(GL_POINTS);
    glColor4d(0.0f, 1.0f, 0.0f, 0.8f);
    for (size_t j = 0; j < local_map_cloud->points.size(); j++) {
      glVertex3d(local_map_cloud->points[j].x, local_map_cloud->points[j].y,
                 local_map_cloud->points[j].z);
    }
    glEnd();

    pangolin::FinishFrame();
  }
  pangolin::GetBoundWindow()->RemoveCurrent();
}

void PangolinViewer::SendInfo(
  std::deque<Combination, Eigen::aligned_allocator<Combination>> & input) {
  std::unique_lock<std::mutex> info_lock(info_list_mutex_);
  input_list = input;
}

}  // namespace localization
}  // namespace apollo
