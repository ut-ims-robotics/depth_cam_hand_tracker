
//
//  As described in the readme file, there is no sophistication in the segmentation so use this program  
//  with just right hand in the depth camera view volume.
//  Note that results will vary from user to user based on how well it matches the hand model being used and 
//  the hands used in the datasets used to train the CNN.   
//
//  There are some runtime settings that will affect the tracking results, some are exposed as gui widgets.
//  For slower motion when the fingers are extended, its often best to only use the cnn results when 
//  there is a measurable better fit of the geometry to the point cloud.
//  for faster motions, or when there is less geometric features to track (such as a clenched rolling fist), it is usually better
//  to trust the cnn output. 
//

#include <cctype>                     // std::tolower

#include "third_party/geometric.h"    // also includes linalg.h and uses hlsl aliases
#include "third_party/mesh.h"         // a simple mesh class with vertex format structure (not tied to dx or opengl)
#include "third_party/glwin.h"        // does the win32 and opengl setup,  similar to glut or glfw,  header file only implementation
#include "third_party/misc_gl.h"      // simple mesh drawing, and to draw a scene (camera pose and array of meshes)
#include "third_party/json.h"    
#include "include/handtrack.h"        // HandTracker - the system for tracking the hand including eval of cnn in separate thread, physics update, model loading

#include "ros/ros.h"
#include "ros/package.h"
#include "human_msgs/Hands.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

/*
 * Some global variables. Probably should create a class
 */
ros::Publisher palm_pose_publisher;      // Publisher for the hand tracker messages
human_msgs::Hands hands_msg;

GLWin* glwin;
HandTracker* htk;

DCamera camera_info;
uint16_t downsample_scale = 2;
bool camera_info_initialized = false;

/*
 * Camera info callback
 */
void cameraInfoCallback(sensor_msgs::CameraInfo camera_info_msg)
{
  // If the camera paremeters are already received, then do nothing
  if (camera_info_initialized)
  {
    return;
  }

  // Get the intrinsic parameters of the camera
  int2 image_size = int2{camera_info_msg.width, camera_info_msg.height};
  float2 focal_lenghts = float2{camera_info_msg.K[0], camera_info_msg.K[4]};
  float2 principal_point = float2{camera_info_msg.K[2], camera_info_msg.K[5]};
  float depth_scale = 0.0001249; // TODO: No idea how its calculated, hence the hardcoded value for SR300

  // Construct a dcamera and downsample it
  camera_info = DCamera(image_size, focal_lenghts, principal_point, depth_scale);
  camera_info = camera_info/downsample_scale;

  // Indicate that the the intrinsic parameters are initialized
  camera_info_initialized = true;

  ROS_INFO("at cam info callback");
}

/*
 * Depth image callback
 */
void depthImageCallback(const sensor_msgs::Image& msg)
{
  /*
   * Get the depth image parameters
   */
  const uint16_t rows_pixel = msg.height;
  const uint16_t columns_pixel = msg.width;
  const uint16_t columns_byte = msg.step;
  const uint16_t bytes_per_pixel = columns_byte/columns_pixel;
  const uint16_t downsample_byte_step = bytes_per_pixel*downsample_scale;
  const uint16_t columns_d = camera_info.dim().x;
  const unsigned short constant_number = (unsigned short)(4.0f / camera_info.get_depth_scale());

  // Initialize the downsampled depth image
  std::vector<unsigned short> depth_image_d(product(camera_info.dim()));

  // If the encoding of the depth image is 32FC1
  if (msg.encoding == "32FC1")
  {
    // Downsample the data 2x2 times
    for (uint16_t row=0, row_d=0; row<rows_pixel; row+=downsample_scale, row_d++)
    {
      for (uint16_t column_byte=0, column_d=0;
           column_byte<columns_byte;
           column_byte+=downsample_byte_step, column_d++)
      {
        /*
         * The data is in a 32FC1 (single channel 32 bit float) format and the hand tracker
         * requires the data as an unsigned short
         */

        // Get the address of the first byte and perform a memcopy
        float data_float;
        std::memcpy(&data_float, &msg.data[row*columns_byte + column_byte], sizeof(data_float));

        // Convert the float to an unsigned short and scale it with a value
        // that I found by trial and error ... it was an "amusing" process indeed
        depth_image_d[row_d*columns_d + column_d] = static_cast<unsigned short>(data_float*6553.5);

        /*
         * If the pixel is dark (=0), then assign a constant
         * value. The hand tracking cnn loves it for some reason
         */
        if (depth_image_d[row_d*columns_d + column_d] == 0)
        {
          depth_image_d[row_d*columns_d + column_d] = constant_number;
        }
      }
    }
  }
  else
  {
    ROS_ERROR("The encoding (%s) of the depth image is unknown", msg.encoding.c_str());
    return;
  }

  // Move the downsampled image to the depth image container
  Image<unsigned short> depth_image(camera_info, std::move(depth_image_d));

  try
  {
    if (!glwin->WindowUp())
    {
      return;
    }

    // Create points and triangle list from the depth data
    auto dmesh = DepthMesh(depth_image, { 0.1f,0.7f }, 0.015f, 2);

    // From 3d points to vertices will all the usual attributes
    auto dxmesh = MeshSmoothish(dmesh.first, dmesh.second);

    // Project the vertices to 2D
    for (auto &v : dxmesh.verts)
    {
      v.texcoord = depth_image.cam.projectz(v.position) / float2(depth_image.cam.dim());
    }

    // Offset the hand mesh 15cm to the side
    // note this may reveal make the depth mesh look a bit more jagged due to z noise
    dxmesh.pose.position.x = 0.15f;

    // Update the hand tracking with the current depth camera input
    htk->update(std::move(depth_image));

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glViewport(0, 0, glwin->res.x, glwin->res.y);
    glClearColor(0.1f+0.2f*(htk->initializing==50) , 0.1f+0.1f*(htk->initializing>0), 0.15f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    {
      auto allmeshes = Addresses(htk->handmodel.sdmeshes);

      /*
       * Create a handtracker message out of the position and orientation
       * data provided by the palm mesh (index = 1)
       */
      geometry_msgs::PoseStamped palm_pose;
      palm_pose.pose.position.x = allmeshes[1]->pose.position[0];
      palm_pose.pose.position.y = allmeshes[1]->pose.position[1];
      palm_pose.pose.position.z = allmeshes[1]->pose.position[2];

      palm_pose.pose.orientation.x = allmeshes[1]->pose.orientation[0];
      palm_pose.pose.orientation.y = allmeshes[1]->pose.orientation[1];
      palm_pose.pose.orientation.z = allmeshes[1]->pose.orientation[2];
      palm_pose.pose.orientation.w = allmeshes[1]->pose.orientation[3];

      palm_pose.header.frame_id = "camera_depth_optical_frame";
      palm_pose.header.stamp = ros::Time::now();

      human_msgs::Hand hand_msg;
      hand_msg.palm_pose = palm_pose;

      // Fill the set message. For no good reason, both hands will receive the same message
      hands_msg.right_hand = hand_msg;
      hands_msg.left_hand = hand_msg;

      // Publish the message
      palm_pose_publisher.publish(hands_msg);


      allmeshes.push_back(&dxmesh);
      // render scene with camera near origin but rolled 180 on x to align CV convention with GL convention
      render_scene({ { 0, 0, -0.05f }, normalize(float4(1, 0, 0, 0)) }, allmeshes);
      drawimage(htk->get_cnn_difference(), { 0.01f,0.63f }, { 0.15f,-0.2f });  // show segment sent to cnn
    }
    float segment_scale = htk->segment_scale;
    WidgetButton(int2{ 3, glwin->res.y - 24 }, int2{ 120,23 }, *glwin, "[ESC] quit", []() {exit(0);}).Draw();
    WidgetSwitch(int2{ 3, glwin->res.y - 48 }, int2{ 220,23 }, *glwin, htk->always_take_cnn, "[c] cnn priority").Draw();
    WidgetSwitch(int2{ 3, glwin->res.y - 74 }, int2{ 220,23 }, *glwin, htk->angles_only    , "[a] cnn angles  ").Draw();
    WidgetSlider(int2{ 3, glwin->res.y - 102}, int2{ 220,23 }, *glwin, segment_scale, float2{0.12f ,0.20f }, "[+/-] handsize ").Draw();
    if (segment_scale != htk->segment_scale)
      htk->scale(segment_scale / htk->segment_scale);  // only call the scale routine if its been changed
    glwin->PrintString({2+ 120 / glwin->font_char_dims.x,0 }, "press ESC to quit,  place right hand only in depth camera view, cnn trained for egocentric ");
    glwin->PrintString({2+ 220 / glwin->font_char_dims.x,2 }, "apply cnn %s   ('c' to toggle)", htk->always_take_cnn ? "every time" : "only when ensures closer fit");
    glwin->PrintString({2+ 220 / glwin->font_char_dims.x,4 }, "%s   ('a' to toggle)", htk->angles_only ? "not using depth, just cnn angles" : "using depth for final fit");
    glwin->PrintString({2+ 220 / glwin->font_char_dims.x,6 }, "hand size  %f cm   (use +/- to scale)", segment_scale);
    glPopAttrib();
    glwin->SwapBuffers();
  }
  catch (const char *c)
  {
    ROS_ERROR_STREAM("catch block 1: " << c);
  }
  catch (std::exception e)
  {
    ROS_ERROR_STREAM("catch block 2: " << e.what());
  }
}

/*
 * Main
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_tracker_test");
  ros::NodeHandle nh;

  // Get the intrinsic parameters of the camra from the camrea info message
  {
    // Subscribe to the camera info topic
    ros::Subscriber camera_info_subscriber = nh.subscribe("camera/depth/camera_info", 10, cameraInfoCallback);

    // Wait for the subscriber to come up
    while (((camera_info_subscriber.getNumPublishers() == 0) || !camera_info_initialized) && ros::ok())
    {
      ROS_INFO("Waiting for depth camera info");
      ros::Duration(0.2).sleep();
      ros::spinOnce();
    }
  }

  // Advertise the palm pose publisher
  palm_pose_publisher = nh.advertise<human_msgs::Hands>("hand_tracker_output", 10);

  // Subscriber for the depth data
  ros::Subscriber depth_image_subscriber = nh.subscribe("camera/depth/image", 10, depthImageCallback);

  // Allocate the glwindow and the handtracker
  glwin = new GLWin("htk - testing hand tracking system  using realsense depth camera input",1280,720);
  htk = new HandTracker(ros::package::getPath(ROS_PACKAGE_NAME) + "/hand_tracking_samples");

  // Initialize the glwindow and the handtracker
  htk->always_take_cnn = false;  // when false the system will just use frame-to-frame when cnn result is not more accurate

  // Set the maximum depth data cut-off distance (in meters)
  htk->drangey = 0.45;

  glwin->keyboardfunc = [&](int key, int, int)
  {
    switch (std::tolower(key))
    {
      // make model hand larger
      case '+': case '=':
        htk->scale(1.02f);
        std::cout << "segment_scale " << htk->segment_scale << std::endl;
        break;

      // make model hand smaller
      case '-': case '_':
        htk->scale(1.0f / 1.02f);
        std::cout << "segment_scale " << htk->segment_scale << std::endl;
        break;

      case 'c':
        htk->always_take_cnn = !htk->always_take_cnn;
        break;

      case 'a':
        htk->angles_only = !htk->angles_only;
        break;

      default: std::cerr << "unused key " << (char)key << std::endl; break;
    }
  };

  // Spin
  ros::spin();

  // Free the memory occupied by the glwindow and the handtracker
  delete glwin;
  delete htk;
}
