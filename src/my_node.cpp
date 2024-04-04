#include <ros/ros.h>
#include <iostream>
#include <grid_map.h>
#include <dmap.h>
#include <draw_helpers.h>
#include <dmap_localizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

DMap dmap(0,0);
GridMapping grid_mapping;
DMapLocalizer localizer;
ros::Publisher odom_pub;

float resolution=0.05;
float max_range=10;
float expansion_range=1;
bool mappa = false;
bool init = false;
float influence_range=2;
uint8_t occ_threshold=100;


//questa funzione costruisce una dmap analizzando gli ostacoli
void mapCallback(const nav_msgs::OccupancyGrid& map_msg) {
  std::vector<Eigen::Vector2f> obstacles;
  obstacles.clear();
  for (size_t i=0; i<map_msg.info.height; ++i) {
    for (size_t j=0; j<map_msg.info.width; ++j){
        size_t k=j+i * map_msg.info.width;
        if (map_msg.data[k]==100){
            //è un ostacolo
            obstacles.push_back(Eigen::Vector2f(j,i));
        }
    }
  }
  //map_msg.data() gli devo passare un vettore nuovo contenente tutti i map_msgs.dat(i)?
  GridMap grid_map(0.1, 0, 0);
  grid_map.loadFromVector(map_msg.data, map_msg.info.resolution, map_msg.info.width,
                          map_msg.info.height); 
  //localizer.setMap(obstacles, resolution, influence_range);
  localizer.setMap(grid_map, 2, 100);
  mappa = true;
}

/*
void computeGridEndpoints(std::vector<Vector2i>& g_map, const std::vector<Vector2f>& src) {
  g_map.clear();
  for (const auto &ep: src) {
    g_map.push_back(grid_mapping.world2grid(ep).cast<int>());
  }
}
*/
/*
//in questa funzione devo inizializzare la dmap, per fare cio devo usare uno dei metodi dentro 
//dmap_localizer. io voglio usare la 3 che vuole come input solo la grid_map non piena (la calcola da sola)
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {

 /* questo non lo posso fare non ho le scan è inutile se le avessi OK ma non le ho
 //creo il vettore scan_endpoints che contiene le scansioni dentro un range
 std::vector<Vector2f> scan_endpoints;
 scan_endpoints.clear();
  //ciclo per populare il vettore, analizza tutte le scan e prende solo quelle con un certo range
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float alpha=scan.angle_min+i*scan.angle_increment;
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
  }

  std::vector<Vector2i> grid_endpoints;
  computeGridEndpoints(grid_endpoints, scan_endpoints);

  pure questo è tutto inutile
  dmap.clear();
  int dmax2=pow(expansion_range/resolution, 2);
  int ops=dmap.compute(grid_endpoints, dmax2);
  cerr << "new kf" << endl;
  Grid_<float> distances;
  dmap.copyTo(distances);

  for (auto& d: distances.cells) {
    d*=resolution;
  }
  
  localizer.setMap(g_map, distances);
  
  //oppure (dovrebbe andare bene uguale (ma non devo calcolare la d map?))
  //localizer.setMap(g_map, 2, 127);
  //questa è la funzione dentro la linea sopra

  //se mi dovesse servire la grid_map come la calcolo?

  grid_mapping = grid_map;
  std::vector<Vector2i> grid_obstacles;
  for (int r=0; r<grid_map.rows; ++r)
    for (int c=0; c<grid_map.cols; ++c)
      if (grid_map(r,c)<occ_threshold) {
        grid_obstacles.push_back(Vector2i(c,r));
      }

  int dmax_2=pow(influence_range/grid_mapping.resolution(),2);
  DMap dmap(grid_map.rows, grid_map.cols);
  dmap.clear();
  int ops = dmap.compute(grid_obstacles, dmax_2);
  
  //compute from the dmap the distances
  dmap.copyTo(distances, dmax_2);

  // convert it from pixels (squared) to meters
  for (auto& f: distances.cells)
    f=sqrt(f)*grid_mapping.resolution();

  // compute the derivatives
  distances.colDerivative(distances_dc);
  distances.rowDerivative(distances_dr);
  mappa = true;
}
*/


void initCallback(const geometry_msgs::PoseWithCovarianceStamped& posa){

  float x = posa.pose.pose.position.x;
  float y = posa.pose.pose.position.y;

  Eigen::Quaternionf q(posa.pose.pose.orientation.w, posa.pose.pose.orientation.x, 
                       posa.pose.pose.orientation.y, posa.pose.pose.orientation.z);
  
  Eigen::Matrix2f matrix = q.toRotationMatrix().block<2, 2>(0, 0);
  std::cout<<"matrice rotazione\n"<< matrix <<std::endl;
  Isometry2f isometry = Isometry2f::Identity();
  isometry.translation() << x, y;
  isometry.linear()=matrix.cast<float>();

  localizer.X = isometry;
  init = true;
}


void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2f> scan_endpoints;

  if(mappa && init){
    scan_endpoints.clear();

    for (size_t i=0; i<scan.ranges.size(); ++i) {
      float alpha=scan.angle_min+i*scan.angle_increment;
      float r=scan.ranges[i];
      if (r< scan.range_min || r> scan.range_max)
        continue;
      scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
    }

    localizer.localize(scan_endpoints,10);
    std::cout<<localizer.X.translation().transpose()<<std::endl;

    nav_msgs::Odometry check_odom;
    check_odom.header.stamp = ros::Time::now();
    check_odom.header.frame_id = "map"; 
    check_odom.child_frame_id = "robot"; 

    Eigen::Matrix3f matrix_3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix2f matrix_2 = localizer.X.linear();
    matrix_3.block<2, 2> (0, 0) = matrix_2;
    Eigen::Quaternionf q_2(matrix_3);

    check_odom.pose.pose.position.x = localizer.X.translation().x();
    check_odom.pose.pose.position.y = localizer.X.translation().y();
    check_odom.pose.pose.position.z = 0.0;

    check_odom.pose.pose.orientation.w = q_2.w();
    check_odom.pose.pose.orientation.x = q_2.x();
    check_odom.pose.pose.orientation.y = q_2.y();
    check_odom.pose.pose.orientation.z = q_2.z();

    //da cancellare
    //isometry_2.linear = matrix_2.cast<float>();
    //Isometry_2.translation() << localizer.X.translation().x(), localizer.X.translation().y();


    /* check_odom.pose.pose.position.x = localizer.X.translation().x();
    check_odom.pose.pose.position.y = localizer.X.translation().y();
    check_odom.pose.pose.position.z = 0.0;

    check_odom.pose.pose.orientation.w = localizer.X.linear().w();
    check_odom.pose.pose.orientation.x = localizer.X.linear().x();
    check_odom.pose.pose.orientation.y = localizer.X.linear().y();
    check_odom.pose.pose.orientation.z = localizer.X.linear().z(); */
     
    odom_pub.publish(check_odom);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_node");
  ros::NodeHandle n;

  // compute the grid size
  int grid_size = 2*(max_range+expansion_range)/resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
  cerr << "grid_size" << grid_size << endl;
  cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;

  //subscriber al topic della mappa
  ros::Subscriber map_sub = n.subscribe("/map", 10, mapCallback);

  //subscriber al topic del'initial pose
  ros::Subscriber init_sub = n.subscribe("/initialpose", 10, initCallback);
  
  //subscriber al topic delle scansioni
  ros::Subscriber scan_sub = n.subscribe("/base_scan", 10, laserCallback);

  //publisher control
  odom_pub = n.advertise<nav_msgs::Odometry>("/estimated_odometry", 10);
  
  ros::spin();
  return 0;
}

