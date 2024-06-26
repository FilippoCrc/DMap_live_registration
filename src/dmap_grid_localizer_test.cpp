#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include "dmap_localizer.h"
#include "draw_helpers.h"
#include "grid_map.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

int main(int argc, char** argv) {
  if (argc<4) {
    std::cout << "usage " << argv[0] << "<map_image> <resolution> <dmax>" << std::endl;
    return -1;
  }

  std::string filename=argv[1];
  float resolution = atof(argv[2]);
  float dmax = atof(argv[3]);
  cerr << "parameters: " << endl;
  cerr << " resolution: " << resolution << endl;
  cerr << " dmax_influence: " << dmax << endl;

  // load the map
  GridMap grid_map(0,0,resolution);
  grid_map.loadFromImage(filename.c_str(), resolution);

  DMapLocalizer localizer;
  localizer.setMap(grid_map, dmax);
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;

  // extract the obstacles from the gmap to use as measurements
  std::vector<Vector2f> obstacles;
  for (int r=0; r<grid_map.rows; ++r)
    for (int c=0; c<grid_map.cols; ++c)
      if (grid_map(r,c)<127) {
        Vector2f endpoint=grid_map.grid2world(Vector2f(c,r));
        if (endpoint.norm()<50)
          obstacles.push_back(endpoint);
      }
  
  // 3. sugar: add an obstacle image as alternative background
  Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
  obstacle_image.fill(0);
  obstacle_image.draw(canvas, false);
  for (const auto& m: obstacles) {
    Vector2f m_hat_grid=grid_map.world2grid(m);
    drawCircle(canvas, m_hat_grid, 3, 255);
  }
  // we draw with cv and get back the result, dirty...
  memcpy(&obstacle_image.cells[0], canvas.data, distances.rows*distances.cols);

  // now run the localizer
  Isometry2f X=Eigen::Isometry2f::Identity();
  X.linear()=Rotation2Df(0.1).matrix();
  X.translation()<< 0.5, 0.5;
  localizer.X=X;
  bool show_obstacles=false;
  while (1) {
    if (show_obstacles)
      obstacle_image.draw(canvas, true);
    else
      distances.draw(canvas, true);
    for (const auto& m: obstacles) {
      Vector2f m_hat=localizer.X*m;
      Vector2f m_hat_grid=grid_map.world2grid(m_hat);
      drawCircle(canvas, m_hat_grid, 3, 255);
    }
    int key = showCanvas(canvas,0);
    if (key == 32) {
      show_obstacles = !show_obstacles;
      continue;
    }

    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    localizer.localize(obstacles, 1);
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;
  }
  
}
