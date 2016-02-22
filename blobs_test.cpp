/************************************************************
 * Name: blobs_test.cpp
 * Authors: David Mattia, Gina Gilmartin, Bridge Harrington,
			Daniel Huang
 * Date: 02/14/2015
 *
 * Description: This program will subscribe to the /blobs topic,
 *         and will drive to different colored blobs in an order
 *				based on the order in the ros parameter seek_visit_order
 ***********************************************************/
#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>

// Global Data Structures
struct blob_color {
  int red, green, blue;
  bool seen;
};

// Global Parameters
const int NUM_COLORS = 3;
struct blob_color colors[NUM_COLORS] = {
  {0, 0, 0, false},
  {1, 1, 1, false},
  {2, 2, 2, false}
};

const int MIN_BLOB_SIZE = 100; // Will only register blobs over this number of pixels
const double AREA_TO_STOP = 20000.0; // Area of the blob for the robot to stop

int blobSize = 0; // The largest blob area seen while going to the current blob
int blob_x_center = 0; // The most recent center of a blob

const int X_MIN = 0; // The left side of the kinect sensor's vision
const int X_MAX = 600; // The right side of the kinect sensor's vision
const int X_CENTER = (X_MAX + X_MIN) / 2; // The center of the kinect's horizontal vision

const double ANGULAR_SCALE = 1.0; // Relative turning speed
const double LINEAR_SCALE = 0.3;  // Relative moving speed
const double MIN_X = 0.1; // The minimum linear speed while moving forward

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 ***********************************************************/
void blobsCallBack (const cmvision::Blobs& blobsIn)
{
  /************************************************************
  * These blob.red, blob.green, and blob.blue values depend on the 
  * values those are provided in the colors.txt file.
  * For example, the color file is like:
  * 
  * [Colors]
  * (0, 0, 0) 0.000000 10 red 
  * (255, 255, 0) 0.000000 10 Green 
  * [Thresholds]
  * ( 75:110, 164:178, 90:113 )
  * ( 47:99, 96:118, 162:175 )
  * 
  * Now, if a red blob is found, then the blobs.red will be 0, and the others will be 0.
  * Similarly, for green blob, blob.red and blob.green will be 255, and blob.blue will be 0.
  ************************************************************/
  for (int i = 0; i < blobsIn.blob_count; ++i)
  {  
    cmvision::Blob blob = blobsIn.blobs[i];
    if(blob.area > MIN_BLOB_SIZE) {
      // A good sized blob has been found
      // Compare this blob to every color from or colors.txt file to see if there is a match
      for(int index = 0; index < NUM_COLORS; ++index) {
        struct blob_color color = colors[index];
        if(blob.red == color.red && blob.green == color.green && blob.blue == color.blue) {
          // Color match was found. Update global variables
          if(blob.area > blobSize) {
            // This blob is larger than the previously largest seen blob. Update @blobSize
            blobSize = blob.area;
          }
          blob_x_center = blob.x;
          colors[index].seen = true;
        }
      } 
    } else {
      // Blob was too small for us to care about it
      //ROS_INFO("Saw a small blob of size: %d", blob.area);
    }
  }
}
 
/************************************************************
 * Function Name: stopMoving
 * Parameters: geometry_msgs::Twist &t
 * Returns: void
 *
 * Description: Stops the robots movements both linearly
 *        and angularly
 ***********************************************************/ 
void stopMoving(geometry_msgs::Twist &t) {
  t.linear.x = t.angular.z = 0.0;
}

/************************************************************
 * Function Name: main
 * Parameters: int argc, char **argv
 * Returns: int
 *
 * Description: This is the main function. It will subscribe
 *        to the /blobs topic and will move to blobs in order
 *        based upon the values in the ros parameter
 *        seek_visit_order
 ***********************************************************/ 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "blobs_test");
  
  // Create handle that will be used for both subscribing and publishing. 
  ros::NodeHandle n;
  
  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);  

  // publish the geometry message twist message
  ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  ROS_INFO("Blobs Program started!");
  
  // Set the loop frequency in Hz.
  ros::Rate loop_rate(10);

  geometry_msgs::Twist t;
  t.linear.x = 0.0;
  
  std::vector<int> order;
  std::string string_order;
  bool got_seek_order = ros::param::get("seek_visit_order", string_order);
  if(!got_seek_order) {
    ROS_INFO("Could not find parameter seek_visit_order");
    return 1;
  } else {
    ROS_INFO("Successfully found order!");
    ROS_INFO("Order is: %s", string_order.c_str());
    // parse values from @string_order into @order
    std::stringstream order_stream(string_order);
    int blob_index;
    while(order_stream >> blob_index) {
      order.push_back(blob_index);
    }
  }
  
  //runtime loop
  // For every blob from the ros parameter seek_visit_order...
  for(std::vector<int>::const_iterator it = order.begin(); it != order.end(); ++it) {
    // Look for next blob to goto and set that it hasn't been seen
    int goto_blob = *it;
    colors[goto_blob-1].seen = false;
    ROS_INFO("Now going to blob: %d", goto_blob);

    // Reset global params
    blobSize = 0;
    blob_x_center = 0;

    while(ros::ok())
    {
      if (colors[goto_blob-1].seen) {
        ROS_INFO("blobSize: %d", blobSize);
        if(blobSize < AREA_TO_STOP) {
          // blob size is small. keep moving
          // Set speed and turn values based on PID movement principles
          t.linear.x = ((AREA_TO_STOP - blobSize) / AREA_TO_STOP) * LINEAR_SCALE + MIN_X;
          t.angular.z = ((X_CENTER - blob_x_center) / double(X_CENTER)) * ANGULAR_SCALE;
        } else {
          // blob is very large. stop
          stopMoving(t);
          ROS_INFO("Stopping!");
          break; // start going to next blob if there is one. Else end program
        }
      } else {
        // haven't seen the next blob yet. Keep turning
	blobSize = 0; // ignore largest blob seen so far
	t.linear.x = 0;
	t.angular.z = 0.5;
      }
  
      // Publish twist message and call callbacks  
      velocityPublisher.publish(t);
      ros::spinOnce();
      loop_rate.sleep();
    } 
  }
  return 0;
}
