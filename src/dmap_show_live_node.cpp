#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>

using namespace std;


DMap dmap(0,0);
GridMapping grid_mapping;

// parameters
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;


void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2i> endpoints;
  float angle=scan.angle_min;
  
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    
    float alpha=scan.angle_min+i*scan.angle_increment;
    Eigen::Vector2i endpoint=grid_mapping.world2grid(Vector2f(r*cos(alpha), r*sin(alpha))).cast<int>();
    endpoints.push_back(endpoint);
  }
  dmap.clear();
  int dmax2=pow(expansion_range/resolution, 2);

  int ops=dmap.compute(endpoints, dmax2);
  cerr  << ops;
  Grid_<float> distances;
  dmap.copyTo(distances);
  distances.draw(canvas, true);
  showCanvas(canvas,1);
}

int main(int argc, char** argv) {
  ros:: init(argc, argv, "boh");
  ros::NodeHandle n;
  string topic_name=argv[1];
  
  // variables for generating dmap

  // compute the grid size
  int grid_size = 2*(max_range+expansion_range)/resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
  cerr << "grid_size" << grid_size << endl;
  cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;

  
  ros::Subscriber sub = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
