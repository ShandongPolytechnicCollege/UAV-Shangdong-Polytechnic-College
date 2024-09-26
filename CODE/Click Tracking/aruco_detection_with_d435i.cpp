#include <iostream>
#include <string>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <spirecv_msgs/Control.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

// 定义窗口名称
static const std::string RGB_WINDOW = "Image window";
// 框选到的矩形
cv::Rect rect_sel;
// 框选起始点
cv::Point pt_origin;
// 是否按下左键
bool b_clicked = false;
// 是否得到一个新的框选区域
bool b_renew_ROI = false;
// 是否开始跟踪
bool b_begin_TRACK = false;

bool b_clicked_state = false;
// 实现框选逻辑的回调函数
void onMouse(int event, int x, int y, int, void *);
void remoteMouse(spirecv_msgs::Control ctl_msg);
cv::Mat ros_img;
bool get_ros_img = false;

int img_width, img_height;

// 【发布】检测结果图像
image_transport::Publisher aruco_pub;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cam_image;
  try
  {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_image->image.copyTo(ros_img);

    get_ros_img = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

struct node
{
  double x, y;
};
node p1, p2, p3, p4;
node p;
double getCross(node p1, node p2, node p)
{
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

int aruco_tracked_id;

bool detect_tracking = true;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aruco_detection_with_d435i");
  ros::NodeHandle nh("~");
  // 更新频率为60HZ
  ros::Rate loop_rate(60);
  int uav_id = 1;
  std::string camera_params_yaml = "";
  std::string local_saving_path = "";
  std::string input_image_topic = "";
  std::string output_image_topic = "";

  nh.getParam("uav_id", uav_id);
  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("local_saving_path", local_saving_path);
  nh.getParam("input_image_topic", input_image_topic);
  nh.getParam("output_image_topic", output_image_topic);

  printf("UAV_ID: %d\n", uav_id);
  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("LOCAL_SAVING_PATH: %s\n", local_saving_path.c_str());
  printf("INPUT_IMAGE_TOPIC: %s\n", input_image_topic.c_str());
  printf("OUTPUT_IMAGE_TOPIC: %s\n", output_image_topic.c_str());

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_subscriber;
  image_subscriber = it.subscribe(input_image_topic, 10, cameraCallback);
  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/aruco_detection_with_d435i", 1);

  aruco_pub = it.advertise(output_image_topic.c_str(), 1);

  ros::Subscriber spirecv_ctl_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/spirecv/control", 10, remoteMouse);

  // 定义一个新的窗口，可在上面进行框选操作
  cv::namedWindow(RGB_WINDOW);
  // 设置窗口操作回调函数，该函数实现整个框选逻辑
  cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
  // 实例化 框选目标跟踪类
  sv::SingleObjectTracker sot;
  // 实例化Aruco检测器类
  sv::ArucoDetector ad;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  sot.loadCameraParams(camera_params_yaml);
  ad.loadCameraParams(camera_params_yaml);

  // // 打开摄像头
  // sv::Camera cap;
  // cap.setWH(640, 480);
  // cap.setFps(30);
  // cap.open(sv::CameraType::WEBCAM, 2);
  // 实例化OpenCV的Mat类，用于内存单帧图像
  cv::Mat img;
  int frame_id = 0;

  img_height = sot.image_height;
  img_width = sot.image_width;

  sv::VideoStreamer streamer;
  streamer.setup(cv::Size(img_width, img_height), 8554, 2);

  while (ros::ok())
  {
    if (get_ros_img == true)
    {
      get_ros_img = false;
      img = ros_img.clone();
      cv::resize(img, img, cv::Size(sot.image_width, sot.image_height));
      if (detect_tracking == true)
      {
        // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
        sv::TargetsInFrame tgts(frame_id++);

        // 执行Aruco二维码检测
        ad.detect(img, tgts);

        // 可视化检测结果，叠加到img上
        sv::drawTargetsInFrame(img, tgts);

        spirecv_msgs::TargetsInFrame ros_tgts;
        ros_tgts.header.frame_id = "frame";
        ros_tgts.header.stamp = ros::Time::now();
        ros_tgts.header.seq = 1;
        ros_tgts.frame_id = tgts.frame_id;
        ros_tgts.height = tgts.height;
        ros_tgts.width = tgts.width;
        ros_tgts.fps = tgts.fps;
        ros_tgts.fov_x = tgts.fov_x;
        ros_tgts.fov_y = tgts.fov_y;

        // 控制台打印Aruco检测结果
        printf("Frame-[%d]\n", frame_id);
        // 打印当前检测的FPS
        printf("  FPS = %.2f\n", tgts.fps);
        // 打印当前相机的视场角（degree）
        printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);

        for (int i = 0; i < tgts.targets.size(); i++)
        {
          spirecv_msgs::Target ros_target;
          ros_target.cx = tgts.targets[i].cx;
          ros_target.cy = tgts.targets[i].cy;
          ros_target.w = tgts.targets[i].w;
          ros_target.h = tgts.targets[i].h;

          ros_target.score = 1.0f;
          ros_target.category = tgts.targets[i].category;
          ros_target.category_id = tgts.targets[i].category_id;

          ros_target.yaw_a = tgts.targets[i].yaw_a;
          ros_target.tracked_id = tgts.targets[i].tracked_id;

          ros_target.los_ax = tgts.targets[i].los_ax;
          ros_target.los_ay = tgts.targets[i].los_ay;

          ros_target.px = tgts.targets[i].px;
          ros_target.py = tgts.targets[i].py;
          ros_target.pz = tgts.targets[i].pz;
          ros_target.mode = false;
          ros_tgts.targets.push_back(ros_target);

          printf("Frame-[%d], Aruco-[%d]\n", frame_id, i);
          // 打印每个二维码的中心位置，cx，cy的值域为[0, 1]
          printf("  Aruco Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[i].cx, tgts.targets[i].cy);
          // 打印每个二维码的外接矩形框的宽度、高度，w，h的值域为(0, 1]
          printf("  Aruco Size (w, h) = (%.3f, %.3f)\n", tgts.targets[i].w, tgts.targets[i].h);
          // 打印每个二维码的方位角，值域为[-180, 180]
          printf("  Aruco Yaw-angle = %.2f\n", tgts.targets[i].yaw_a);
          // 打印每个二维码的类别，字符串类型，"aruco-?"
          printf("  Aruco Category = %s\n", tgts.targets[i].category.c_str());
          // 打印每个二维码的ID号
          printf("  Aruco Tracked-ID = %d\n", tgts.targets[i].tracked_id);
          // 打印每个二维码的视线角，跟相机视场相关
          printf("  Aruco Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[i].los_ax, tgts.targets[i].los_ay);
          // 打印每个二维码的3D位置（在相机坐标系下），跟二维码实际边长、相机参数相关
          printf("  Aruco Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);
          // 打印每个目标的3D位置（在相机坐标系下），跟目标实际长宽、相机参数相关
          printf("  Object Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);

          p1.x = tgts.targets[i].cx * tgts.width - tgts.targets[i].w * tgts.width / 2;
          p1.y = tgts.targets[i].cy * tgts.height - tgts.targets[i].h * tgts.height / 2;
          p2.x = tgts.targets[i].cx * tgts.width + tgts.targets[i].w * tgts.width / 2;
          p2.y = tgts.targets[i].cy * tgts.height - tgts.targets[i].h * tgts.height / 2;
          p4.x = tgts.targets[i].cx * tgts.width - tgts.targets[i].w * tgts.width / 2;
          p4.y = tgts.targets[i].cy * tgts.height + tgts.targets[i].h * tgts.height / 2;
          p3.x = tgts.targets[i].cx * tgts.width + tgts.targets[i].w * tgts.width / 2;
          p3.y = tgts.targets[i].cy * tgts.height + tgts.targets[i].h * tgts.height / 2;
          p.x = pt_origin.x;
          p.y = pt_origin.y;
          // std::cout << "p.x " << p.x << "\t"
          //           << "p.y " << p.y << std::endl;
          if (getCross(p1, p2, p) * getCross(p3, p4, p) >= 0 && getCross(p2, p3, p) * getCross(p4, p1, p) >= 0)
          {
            b_begin_TRACK = false;
            detect_tracking = false;
            // pt_origin = cv::Point(nor_x, nor_p_y);
            // std::cout << "pt_origin  " <<nor_x<<"/t"<<nor_p_y<< std::endl;
            // rect_sel = cv::Rect(p1.x, p1.y, tgts.targets[i].w * tgts.width, tgts.targets[i].h * tgts.height);
            // std::cout << rect_sel << std::endl;
            aruco_tracked_id = tgts.targets[i].tracked_id;
            b_renew_ROI = true;
            frame_id = 0;
            printf("spirecv mode is tracked!\n");
          }
          else
          {
            printf("spirecv mode is detection!\n");
          }
        }
        spirecv_msg_pub.publish(ros_tgts);
      }
      else
      {
        /* code */
        // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
        sv::TargetsInFrame tgts(frame_id++);

        // 开始 单目标跟踪 逻辑
        // 是否有新的目标被手动框选
        if (b_renew_ROI)
        {
          // 拿新的框选区域 来 初始化跟踪器
          // sot.init(img, rect_sel);
          // 重置框选标志
          b_renew_ROI = false;
          // 开始跟踪
          b_begin_TRACK = true;
          b_clicked_state = true;
        }
        else if (b_begin_TRACK)
        {
          // 以前一帧的结果继续跟踪
          // sot.track(img, tgts);
          // 执行Aruco二维码检测
          ad.detect(img, tgts);

          // 可视化检测结果，叠加到img上
          sv::drawTargetsInFrame(img, tgts, aruco_tracked_id);

          spirecv_msgs::TargetsInFrame ros_tgts;
          ros_tgts.header.frame_id = "frame";
          ros_tgts.header.stamp = ros::Time::now();
          ros_tgts.header.seq = 1;
          ros_tgts.frame_id = tgts.frame_id;
          ros_tgts.height = tgts.height;
          ros_tgts.width = tgts.width;
          ros_tgts.fps = tgts.fps;
          ros_tgts.fov_x = tgts.fov_x;
          ros_tgts.fov_y = tgts.fov_y;

          // 控制台打印 单目标跟踪 结果
          printf("Frame-[%d]\n", frame_id);
          // 打印当前检测的FPS
          printf("  FPS = %.2f\n", tgts.fps);
          // 打印当前相机的视场角（degree）
          printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);
          for (int i = 0; i < tgts.targets.size(); i++)
          {
            if (tgts.targets[i].category_id == aruco_tracked_id)
            {
              spirecv_msgs::Target ros_target;
              ros_target.cx = tgts.targets[i].cx;
              ros_target.cy = tgts.targets[i].cy;
              ros_target.w = tgts.targets[i].w;
              ros_target.h = tgts.targets[i].h;

              ros_target.score = 1.0f;
              ros_target.category = tgts.targets[i].category;
              ros_target.category_id = tgts.targets[i].category_id;

              ros_target.yaw_a = tgts.targets[i].yaw_a;
              ros_target.tracked_id = tgts.targets[i].tracked_id;

              ros_target.los_ax = tgts.targets[i].los_ax;
              ros_target.los_ay = tgts.targets[i].los_ay;

              ros_target.px = tgts.targets[i].px;
              ros_target.py = tgts.targets[i].py;
              ros_target.pz = tgts.targets[i].pz;
              ros_target.mode = true;
              ros_tgts.targets.push_back(ros_target);

              printf("Frame-[%d], Aruco-[%d]\n", frame_id, i);
              // 打印每个二维码的中心位置，cx，cy的值域为[0, 1]
              printf("  Aruco Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[i].cx, tgts.targets[i].cy);
              // 打印每个二维码的外接矩形框的宽度、高度，w，h的值域为(0, 1]
              printf("  Aruco Size (w, h) = (%.3f, %.3f)\n", tgts.targets[i].w, tgts.targets[i].h);
              // 打印每个二维码的方位角，值域为[-180, 180]
              printf("  Aruco Yaw-angle = %.2f\n", tgts.targets[i].yaw_a);
              // 打印每个二维码的类别，字符串类型，"aruco-?"
              printf("  Aruco Category = %s\n", tgts.targets[i].category.c_str());
              // 打印每个二维码的ID号
              printf("  Aruco Tracked-ID = %d\n", tgts.targets[i].tracked_id);
              // 打印每个二维码的视线角，跟相机视场相关
              printf("  Aruco Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[i].los_ax, tgts.targets[i].los_ay);
              // 打印每个二维码的3D位置（在相机坐标系下），跟二维码实际边长、相机参数相关
              printf("  Aruco Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);
              // 打印每个目标的3D位置（在相机坐标系下），跟目标实际长宽、相机参数相关
              printf("  Object Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);
            }
          }
          spirecv_msg_pub.publish(ros_tgts);
        }
      }

      streamer.stream(img);

      // 显示检测结果img
      cv::imshow(RGB_WINDOW, img);
      cv::waitKey(1);

      // 显示检测结果img
      sensor_msgs::ImagePtr det_output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      aruco_pub.publish(det_output_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void onMouse(int event, int x, int y, int, void *)
{
  if (b_clicked)
  {
    // 更新框选区域坐标
    pt_origin.x = 0;
    pt_origin.y = 0;
  }
  // 左键按下
  if (event == cv::EVENT_LBUTTONDOWN && b_clicked_state == false)
  {
    detect_tracking = true;
    pt_origin = cv::Point(x, y);
  }

  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    detect_tracking = true;
    b_renew_ROI = false;
    b_begin_TRACK = false;
    b_clicked = true;
    b_clicked_state = false;
  }
}

void remoteMouse(spirecv_msgs::Control ctl_msg)
{
  if (b_clicked)
  {
    // 更新框选区域坐标
    pt_origin.x = 0;
    pt_origin.y = 0;
  }
  // 左键按下
  if (ctl_msg.mouse == spirecv_msgs::Control::MOUSE_LEFT  && b_clicked_state == false)
  {
    detect_tracking = true;
    pt_origin = cv::Point(ctl_msg.x * img_width, ctl_msg.y * img_height);
  }

  else if (ctl_msg.mouse == spirecv_msgs::Control::MOUSE_RIGHT)
  {
    detect_tracking = true;
    b_renew_ROI = false;
    b_begin_TRACK = false;
    b_clicked = true;
    b_clicked_state = false;
  }
}
