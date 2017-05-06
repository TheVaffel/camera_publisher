#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <turbojpeg.h>
#include <cmath>

#include <cstdio>

#define USE_TURBOJPEG
#include <webcam/webcam.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_publisher_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::NodeHandle private_handle("~");

  if(!private_handle.hasParam("device_name")){
    ROS_FATAL("device_name needs to be specified (e.g. /dev/video0)");
    exit(1);
  }

  if(!private_handle.hasParam("topic_name")){
    ROS_FATAL("topic_tame needs to be specified");
    exit(1);
  }

  std::string device_name, topic_name;
  private_handle.param<std::string>("device_name", device_name, "/dev/video0");
  private_handle.param<std::string>("topic_name", topic_name, "data/video1");

  int w, h;
  private_handle.param<int>("camera_width", w, 1280);
  private_handle.param<int>("camera_height", h, 720);

  int out_w, out_h;
  private_handle.param<int>("output_width", out_w, w);
  private_handle.param<int>("output_height", out_h, h);

  std::string input_format;
  private_handle.param<std::string>("input_format", input_format, "jpeg");
  
  image_transport::Publisher pub = it.advertise(topic_name, 1);

  if(input_format == "jpeg"){
    webcam_init_full_name(w, h, device_name.c_str());
  }else if(input_format == "yuyv"){
    webcam_init_full_name(w, h, device_name.c_str(), WEBCAM_MODE_YUYV);
  }else if(input_format == "bayer"){
    webcam_init_full_name(w, h, device_name.c_str(), WEBCAM_MODE_BAYER);
  }else{
    ROS_FATAL("input_format not recognized");
    exit(1);
  }

  unsigned char* input_buffer = new unsigned char[w*h*4];
  unsigned char* output_buffer = new unsigned char[out_w*out_h*3];
  int scale = w/out_w;
  int scale2 = h/out_h;
  bool reliable_scale = scale*out_w == w && scale*out_h == h && scale == scale2; // In case it's not an integer multiple

  int scale_power = 1<<30;
  while(!((1<<scale_power)&scale)){
    scale_power--;
  }
  while(ros::ok()){
    webcam_capture_image(input_buffer);

    if(w == out_w && h == out_h){
      unsigned char* p_i = input_buffer, *p_o;
      for(int i = 0; i < w*h; i++){
	p_o = output_buffer + (i + 1)*3;
	for(int j = 0; j < 3; j++){
	  *(--p_o) = *(p_i++);
	}
	p_i++;
      }
    }else if(reliable_scale && ((scale & (scale -1)) == 0)){ // Is a power of two or zero
      unsigned char* p_o = output_buffer;
      for(int i = 0; i < out_h; i++){
	for(int j = 0; j < out_w; j++){
	  int r = 0, g = 0, b = 0;
	  int offx = j<<scale_power;
	  int offy = i<<scale_power;
	  for(int u = 0; u < scale ;u++){
	    for(int v = 0; v < scale; v++){
	      r += input_buffer[4*((offy + u)*w + offx + v)];
	      g += input_buffer[4*((offy + u)*w + offx + v) + 1];
	      b += input_buffer[4*((offy + u)*w + offx + v) + 2];
	    }
	  }
	  
	  //printf("R = %d, G = %d, B = %d\n", r, b, g);
	  *(p_o++) = (unsigned char)(b>>(scale_power*2));
	  *(p_o++) = (unsigned char)(g>>(scale_power*2));
	  *(p_o++) = (unsigned char)(r>>(scale_power*2));
	}
      }
    }else{
      for(int i= 0; i < out_h; i++){
	for(int j = 0; j < out_w; j++){
	  float ex_x = ((float)j)/out_w*w;
	  float ex_y = ((float)i)/out_h*h;

	  int x1 = floor(ex_x);
	  int x2 = ceil(ex_x);
	  int y1 = floor(ex_y);
	  int y2 = ceil(ex_y);

	  float dx = ex_x - x1;
	  float dy = ex_y - y1;

	  int index_o = 3*(i*out_w + j);
	  int index_i = 4*(y1*w + x1);

	  float xy = dx*dy; float ixy = (1 - dx)*dy; float xiy = dx*(1 - dy); float ixiy = (1 - dx)*(1 - dy);
	  output_buffer[index_o + 2] = (unsigned char)(ixiy*input_buffer[index_i]
						   + ixy*input_buffer[index_i + 4]
						   + xiy*input_buffer[index_i + 4*w]
						   + xy*input_buffer[index_i + 4*(w + 1)]);
	  index_i++;
	  output_buffer[index_o + 1] = (unsigned char)(ixiy*input_buffer[index_i]
						       + ixy*input_buffer[index_i + 4]
						       + xiy*input_buffer[index_i + 4*w]
						       + xy*input_buffer[index_i + 4*(w + 1)]);
	  index_i++;
	  output_buffer[index_o] = (unsigned char)(ixiy*input_buffer[index_i]
						       + ixy*input_buffer[index_i + 4]
						       + xiy*input_buffer[index_i + 4*w]
						       + xy*input_buffer[index_i + 4*(w + 1)]);
	}
      }
    }

    sensor_msgs::Image output_image;
    output_image.header.stamp = ros::Time::now();
    output_image.height = out_h;
    output_image.width = out_w;
    output_image.encoding = "rgb8";
    output_image.is_bigendian = false;
    output_image.step = 3*out_w;

    output_image.data = std::vector<unsigned char> (output_buffer, output_buffer + (out_w*out_h*3));

    pub.publish(output_image);
  }

  webcam_close();
  
}

