// Standalone diagnostic: compile with the mapper's -march=native flag and the
// installed binary PCL. argv[1] enables caller-side allocation of the output.
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
int main(int argc, char **)
{
  using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
  Cloud::Ptr input(new Cloud()), output(new Cloud());
  for (int i=0;i<200;++i)
  {
    pcl::PointXYZINormal p{};
    p.x=i*.1f;p.y=.1f;p.z=.1f;
    input->push_back(p);
  }
  pcl::VoxelGrid<pcl::PointXYZINormal> filter;
  filter.setLeafSize(.5f,.5f,.5f);
  filter.setInputCloud(input);
  if(argc>1) output->reserve(input->size());
  filter.filter(*output);
  return output->empty() ? 2 : 0;
}
