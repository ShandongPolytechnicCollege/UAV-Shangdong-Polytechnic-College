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

// 是否得到一个新的框选区域
bool b_renew_ROI = false;
// 是否开始跟踪
bool b_begin_TRACK = false;

cv::Mat ros_img;
bool get_ros_img = false;

int img_width, img_height;

//【发布】检测结果图像
image_transport::Publisher aruco_pub;
ros::Publisher spirecv_msg_pub;

// 实现框选逻辑的回调函数
void onMouse(int event, int x, int y, int, void *);
void remoteMouse(spirecv_msgs::Control ctl_msg);

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
// 是否按下左键
bool b_clicked = false;
bool detect_tracking = true;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "car_detection_with_tracking");
  ros::NodeHandle nh("~");
  // 更新频率为60HZ
  ros::Rate loop_rate(60);
  int uav_id = 1;
  std::string camera_params_yaml = "";
  std::string local_saving_path = "";
  std::string input_image_topic = "";
  std::string output_image_topic = "";

  pt_origin.x = -1;
  pt_origin.y = -1;

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
  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/car_detection_with_tracking", 1);

  ros::Subscriber spirecv_ctl_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/spirecv/control", 10, remoteMouse);

  aruco_pub = it.advertise(output_image_topic.c_str(), 1);

  // 定义一个新的窗口，可在上面进行框选操作
  cv::namedWindow(RGB_WINDOW);
  // 设置窗口操作回调函数，该函数实现整个框选逻辑
  cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
  // 实例化 框选目标跟踪类
  sv::SingleObjectTracker sot;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  sot.loadCameraParams(camera_params_yaml);
  // 实例化 通用目标 检测器类
  sv::CommonObjectDetector cod;
  cod.loadCameraParams(camera_params_yaml);

  // 打开摄像头
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
      if (detect_tracking == true)
      {
        // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
        sv::TargetsInFrame tgts(frame_id++);

        // 读取一帧图像到img
        // cap.read(img);

        cv::resize(img, img, cv::Size(cod.image_width, cod.image_height));
        
        rect_sel = cv::Rect(0, 0, 4, 4);  
        // 拿新的框选区域 来 初始化跟踪器
        sot.init(img, rect_sel);


        // 执行通用目标检测
        cod.detect(img, tgts);

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

        // 控制台打印通用目标检测结果
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
          ros_tgts.targets.push_back(ros_target);

          printf("Frame-[%d], Object-[%d]\n", frame_id, i);
          // 打印每个目标的中心位置，cx，cy的值域为[0, 1]
          printf("  Object Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[i].cx, tgts.targets[i].cy);
          // 打印每个目标的外接矩形框的宽度、高度，w，h的值域为(0, 1]
          printf("  Object Size (w, h) = (%.3f, %.3f)\n", tgts.targets[i].w, tgts.targets[i].h);
          // 打印每个目标的置信度
          printf("  Object Score = %.3f\n", tgts.targets[i].score);
          // 打印每个目标的类别，字符串类型
          printf("  Object Category = %s\n", tgts.targets[i].category.c_str());
          // 打印每个目标的视线角，跟相机视场相关
          printf("  Object Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[i].los_ax, tgts.targets[i].los_ay);
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
          std::cout << "p.x " << p.x << "\t"
                    << "p.y " << p.y << std::endl;
          if (getCross(p1, p2, p) * getCross(p3, p4, p) >= 0 && getCross(p2, p3, p) * getCross(p4, p1, p) >= 0)
          {
            b_begin_TRACK = false;
            detect_tracking = false;
            // pt_origin = cv::Point(nor_x, nor_p_y);
            // std::cout << "pt_origin  " <<nor_x<<"/t"<<nor_p_y<< std::endl;
            rect_sel = cv::Rect(p1.x, p1.y, tgts.targets[i].w * tgts.width, tgts.targets[i].h * tgts.height);
            // std::cout << rect_sel << std::endl;
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
        // 读取一帧图像到img
        // cap.read(img);
        cv::resize(img, img, cv::Size(sot.image_width, sot.image_height));
        // 开始 单目标跟踪 逻辑
        // 是否有新的目标被手动框选
        if (b_renew_ROI)
        {
          // 拿新的框选区域 来 初始化跟踪器
          sot.init(img, rect_sel);
          // 重置框选标志
          b_renew_ROI = false;
          // 开始跟踪
          b_begin_TRACK = true;
        }
        else if (b_begin_TRACK)
        {
          // 以前一帧的结果继续跟踪
          sot.track(img, tgts);

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

          // 控制台打印 单目标跟踪 结果
          printf("Frame-[%d]\n", frame_id);
          // 打印当前检测的FPS
          printf("  FPS = %.2f\n", tgts.fps);
          // 打印当前相机的视场角（degree）
          printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);
          if (tgts.targets.size() > 0)
          {
            spirecv_msgs::Target ros_target;
            ros_target.cx = tgts.targets[0].cx;
            ros_target.cy = tgts.targets[0].cy;
            ros_target.w = tgts.targets[0].w;
            ros_target.h = tgts.targets[0].h;

            ros_target.los_ax = tgts.targets[0].los_ax;
            ros_target.los_ay = tgts.targets[0].los_ay;

            ros_target.px = tgts.targets[0].px;
            ros_target.py = tgts.targets[0].py;
            ros_target.pz = tgts.targets[0].pz;
            ros_target.mode = b_begin_TRACK;

            ros_tgts.targets.push_back(ros_target);

            printf("Frame-[%d]\n", frame_id);
            // 打印 跟踪目标 的中心位置，cx，cy的值域为[0, 1]
            printf("  Tracking Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[0].cx, tgts.targets[0].cy);
            // 打印 跟踪目标 的外接矩形框的宽度、高度，w，h的值域为(0, 1]
            printf("  Tracking Size (w, h) = (%.3f, %.3f)\n", tgts.targets[0].w, tgts.targets[0].h);
            // 打印 跟踪目标 的视线角，跟相机视场相关
            printf("  Tracking Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[0].los_ax, tgts.targets[0].los_ay);
             // 打印 跟踪目标的3D位置（在相机坐标系下），跟目标实际长宽、相机参数相关
          printf("  Tracking Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[0].px, tgts.targets[0].py, tgts.targets[0].pz);

          }
          spirecv_msg_pub.publish(ros_tgts);
        }
      }

      streamer.stream(img);

      // 显示检测结果img
      cv::imshow(RGB_WINDOW, img);
      cv::waitKey(10);

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
    pt_origin.x = -1;
    pt_origin.y = -1;
  }
  // 左键按下
  if (event == cv::EVENT_LBUTTONDOWN)
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
  }
}

void remoteMouse(spirecv_msgs::Control ctl_msg)
{
  if (b_clicked)
  {
    // 更新框选区域坐标
    pt_origin.x = -1;
    pt_origin.y = -1;
  }
  // 左键按下
  if (ctl_msg.mouse == spirecv_msgs::Control::MOUSE_LEFT)
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
  }
}

