#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

using namespace std;


DMap dmap(0,0);
GridMapping grid_mapping;
DMapLocalizer localizer;

// parameters
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;
Isometry2f lmap_pose;
bool first_scan=true;

void computeScanEndpoints(std::vector<Vector2f>& dest, const sensor_msgs::LaserScan& scan) {
  dest.clear();
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float alpha=scan.angle_min+i*scan.angle_increment;
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    dest.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
  }
}

void computeGridEndpoints(std::vector<Vector2i>& dest, const std::vector<Vector2f>& src) {
  dest.clear();
  for (const auto &ep: src) {
    dest.push_back(grid_mapping.world2grid(ep).cast<int>());
  }
}

void initLocalizer(std::vector<Vector2f>& scan_endpoints) {
    std::vector<Vector2i> grid_endpoints;
    computeGridEndpoints(grid_endpoints, scan_endpoints);
    dmap.clear();
    int dmax2=pow(expansion_range/resolution, 2);
    int ops=dmap.compute(grid_endpoints, dmax2);
    cerr << "new kf" << endl;
    Grid_<float> distances;
    dmap.copyTo(distances);
    for (auto& d: distances.cells) {
      d*=resolution;
    }
    localizer.setMap(grid_mapping, distances);
}

void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2f> scan_endpoints;
  computeScanEndpoints(scan_endpoints, scan);
  
  if (first_scan) {
    initLocalizer(scan_endpoints);
    first_scan=false;
    localizer.X.setIdentity();
  } else {
    float angle=scan.angle_min;
    localizer.localize(scan_endpoints,10);
    float translation_norm=localizer.X.translation().norm();
    float orientation_norm=fabs(Eigen::Rotation2Df(localizer.X.linear()).angle());
    if (translation_norm>0.5 || orientation_norm>0.5) {
      lmap_pose = lmap_pose*localizer.X;
      initLocalizer(scan_endpoints);
    }
  }
  localizer.distances.draw(canvas, true);
  for (const auto& ep: scan_endpoints) {
    drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 3, 255);
  }
  showCanvas(canvas,1);
}

int main(int argc, char** argv) {
  ros:: init(argc, argv, "boh");
  ros::NodeHandle n;
  string topic_name=argv[1];

  // compute the grid size
  int grid_size = 2*(max_range+expansion_range)/resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
  cerr << "grid_size" << grid_size << endl;
  cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;

  lmap_pose.setIdentity();
  
  ros::Subscriber sub = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
