#include <string>
#include <gpd/candidate/hand.h>
#include <gpd/grasp_detector.h>
#include <fstream>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>
namespace gpd {
namespace apps {
namespace detect_grasps {

bool checkFileExists(const std::string &file_name) {
  std::ifstream file;
  file.open(file_name.c_str());
  if (!file) {
    std::cout << "File " + file_name + " could not be found!\n";
    return false;
  }
  file.close();
  return true;
}





int DoMain(int argc, char *argv[]) {
  // Read arguments from command line.
  if (argc < 3) {
    std::cout << "Error: Not enough input arguments!\n\n";
    std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
    std::cout << "Detect grasp poses for a point cloud, PCD_FILE (*.pcd), "
                 "using parameters from CONFIG_FILE (*.cfg).\n\n";
    std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each "
                 "point in the cloud (*.csv).\n";
    return (-1);
  }

  std::string config_filename = argv[1];
  std::string pcd_dir = argv[2];
  std::string cat = argv[3];
  std::string idx = argv[4];
  std::string dir = argv[5];
  


  size_t n = 3;
 
  int precision = n - std::min(n, idx.size());
  idx.insert(0, precision, '0');


  std::string pcd_filename = pcd_dir + "/" + cat + "/" + cat + idx + ".pcd";
  std::string save_dir = dir + "/" + cat;
  std::string save_path = save_dir  + "/" + cat + idx + "_gpd_grasps.csv";
  // std::string save_path = save_dir + "/" + cat + "/" + cat + idx + "_gpd_grasps.csv";
  
  
  // std::string normals_filename = argv[3];
  std::cout << pcd_filename;
  // return 0

  if (!checkFileExists(config_filename)) {
    printf("Error: config file not found!\n");
    return (-1);
  }
  if (!checkFileExists(pcd_filename)) {
    printf("Error: PCD file not found!\n");
    return (-1);
  }

  // Read parameters from configuration file.
  util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();

  // Set the camera position. Assumes a single camera view.
  std::vector<double> camera_position =
      config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                 "0.0 0.0 0.0");
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << camera_position[0], camera_position[1], camera_position[2];

  // Load point cloud from file.
  util::Cloud cloud(pcd_filename, view_points);
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  // Load surface normals from file.
  // if (argc > 3) {
  //   std::string normals_filename = argv[3];
  //   cloud.setNormalsFromFile(normals_filename);
  //   std::cout << "Loaded surface normals from file: " << normals_filename
  //             << "\n";
  // }

  GraspDetector detector(config_filename);

  // Preprocess the point cloud.
  detector.preprocessPointCloud(cloud);

  // If the object is centered at the origin, reverse all surface normals.
  bool centered_at_origin =
      config_file.getValueOfKey<bool>("centered_at_origin", false);
  if (centered_at_origin) {
    printf("Reversing normal directions ...\n");
    cloud.setNormals(cloud.getNormals() * (-1.0));
  }
  std::vector<std::unique_ptr<candidate::Hand>> clusters;


  // clusters = detector.detectGrasps(cloud);
  clusters = detector.detectGrasps(cloud);

  // std::cout<< clusters.front().get() << std::endl;

  // auto& grasp = clusters[0];

  // // auto& hand = *grasp;
  // // cout << grasp->getScore() << endl;
  // // cout << grasp->getCenter() << endl;

  // // cout << grasp->&getFrame() << endl;
  // // Eigen::Matrix3d orientation = grasp->getOrientation();
  // Eigen::Matrix3d orientation = grasp->getFrame();
  // Eigen::Vector3d position = grasp->getPosition();
  // std::cout << grasp->getBottom() << std::endl;
  // // Eigen::Vector3d position = grasp->getBottom();
  // cout << position << endl;
  // cout << orientation << endl;
  
  std::ofstream save_file;
  // save_file.open("grasp_from_gpd.csv");


  // const char* dirname = save_dir.c_str();
  // // Creating a directory
  // if (mkdir(dirname, 0777) == -1)
  //     cerr << "Error :  " << strerror(errno) << endl;
  // else
  //     mkdir(dirname, 0777);
  //     cout << "Directory created";
  // cout << save_dir << endl;
  boost::filesystem::create_directories(save_dir);
  save_file.open(save_path);
  std::cout<< "saving clusters" << clusters.size() << std::endl;

  int kk = 0;
  for(kk=0; kk<clusters.size(); kk++) {
    auto& grasp = clusters[kk];
    Eigen::Matrix3d orientation = grasp->getFrame();
    Eigen::Vector3d position = grasp->getPosition();
    // cout << "new grasp: " << endl;
    // cout << grasp->getScore() << endl;
    // cout << position.x() << position.y()  << endl;
    // cout << orientation(0,0) << endl;

    save_file << grasp->getScore() << "," << position.x() << "," << position.y() << "," << position.z()\
              << orientation(0,0) << "," << orientation(0,1) << "," << orientation(0,2) << ","\
              << orientation(1,0) << "," << orientation(1,1) << "," << orientation(1,2) << ","\
              << orientation(2,0) << "," << orientation(2,1) << "," << orientation(2,2) << endl;
  }
  save_file.close();



  // std::ofstream file("grasp_from_gpd.txt");
  // if (file.is_open()) {
  //   file << position.transpose() << '\n';
  //   file << orientation;
  // }

  // grasp->writeHandsToFile("hands.txt", *grasp);

// hand_list[i]

  // std::cout<< clusters.front().get() << std::endl;
  // clusters.front().writeHandsToFile("clusters.txt", clusters.front())
  // std::cout<< clusters.front().get() << std::endl;


  // cout << clusters.at(0) << endl;

  // candidate::Hand * ptr = clusters.front().get();
  // std::cout << *ptr << endl;

  // clusters.front()->writeHandsToFile("clusters.txt", *clusters.at(0).get());



  // cout << clusters.at(0)->writeHandsToFile("clusters.txt", clusters.at(0)) << endl;
  // for (std::vector<int>::iterator it = clusters.begin() ; it != clusters.end(); ++it)
    // std::cout << ' ' << *it;
  // std::cout << '\n';
  // std::cout << "Value pointed ptr   " << clusters.size() << std::endl;
  // std::cout << *clusters << std::endl;

  return 0;
}




}  // namespace detect_grasps
}  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  return gpd::apps::detect_grasps::DoMain(argc, argv);
}
