#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " <input.pcd> <alpha>\n";
    return 1;
  }

  float alpha = std::stof (argv[2]);

  // Load file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (pcl::io::loadPCDFile (argv[1], *source_cloud) == -1)
  {
    std::cerr << "Error: could not load " << argv[1] << '\n';
    return 1;
  }

  for (std::size_t i = 0; i < source_cloud->size (); ++i)
  {
    source_cloud->points[i].z = 0.0;
  }

  std::vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_tmp (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::ConcaveHull<pcl::PointXYZRGB> concave_hull;
  concave_hull.setInputCloud (source_cloud);
  concave_hull.setAlpha (alpha);
  concave_hull.setDimension (2);
  concave_hull.setKeepInformation (true);
  concave_hull.reconstruct (*hull_tmp, polygons);

  std::cout << "Number of generated hulls: " << polygons.size () << '\n';
  return 0;
}
