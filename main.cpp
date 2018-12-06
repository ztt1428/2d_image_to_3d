#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <cstring>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <fstream>

bool get_setImagePixel(IplImage* img, int x, int y, std::vector<int> &rgb);
void PcloudToImage(double x, double y, double z,std::vector<double> &pixelXY);
 
namespace po = boost::program_options;

void split(std::string &s,std::vector<std::string> &result,const char flag = ' ')
{
  result.clear();
    std::istringstream iss(s);
    std::string temp;
    while(getline(iss,temp,flag))
    {
        result.push_back(temp);
    }
    return;
}

int count = 0;
 
int main(int argc, char **argv){
	std::string infile = "";
	std::string inpath;
	std::string outpath;
	std::string outfile;
	std::string labelPath;
	std::string labelFile;
	char buffer[100];
	inpath = "/media/ttzhou/documents/dataset/Kitti/object/training/velodyne/";
	outpath = "/media/ttzhou/documents/dataset/Kitti/point_seg_result/";
	labelPath = "/media/ttzhou/documents/dataset/Kitti/label_seg/";
	std::string name;
	for(int filecount = 0;filecount<7481;filecount++)
	{
	    infile = "";
	    int n = std::sprintf(buffer,"%s%06d.bin",inpath.c_str(),filecount);
	    for(int i = 0;i<n;i++)
	            infile+=buffer[i];
	    int name_pos = infile.rfind("/");
	    name = infile.substr(name_pos+1,6);
	    std::cout<<name<<std::endl;
	    outfile = outpath + name + "seg.pcd";
	    std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
	    if(!input.good()){
		    std::cerr << "Could not read file: " << infile << std::endl;
		    exit(EXIT_FAILURE);
	    }
	    input.seekg(0, std::ios::beg);
    
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
	    int i;
	    double intensity;
	    for (i=0; input.good() && !input.eof(); i++) {
		    pcl::PointXYZRGBA point;
		    input.read((char *) &point.x, 3*sizeof(float));
		    input.read((char *) &intensity, sizeof(float));
		    point.r = intensity * 255;
		    point.g = point.r;
		    point.b = point.r;
		    points->push_back(point);
	    }
	    input.close();
    
	    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << std::endl;
	    
	    std::vector<double> pixelXY;
	    std::vector<int> rgb;
	    
	    std::vector<std::string> result;
	    
	    split(infile,result,'/');
	    
	    std::string pathForCloud = "/media/ttzhou/documents/dataset/Kitti/image_seg_result/mark";
	    //std::string absolutePath = pathForCloud + result[result.size()-1];
	    //int pos = absolutePath.rfind("bin");
	    //absolutePath.replace(pos,3,"png");
	    std::string absolutePath = pathForCloud + name + ".png";
	    labelFile = labelPath + name + ".txt";
	    std::ofstream fileout;
	    std::string ss;
	    ss = "(1, 2)";
	    fileout.open(labelFile);
	    if(!fileout.is_open())
	         return 0;
	    cv::Mat image = cv::imread(absolutePath,1);
	    IplImage* img = cvLoadImage(absolutePath.c_str());
	    std::cout<<image.cols<<" "<<image.rows<<std::endl;
	    for(int index = 0; index < i;index++)
	    {
		PcloudToImage(points->points[index].x,points->points[index].y,points->points[index].z,pixelXY);
		bool ExistInImage = get_setImagePixel(img,pixelXY[0],pixelXY[1],rgb);
		points->points[index].r = rgb[0];
		points->points[index].g = rgb[1];
		points->points[index].b = rgb[2];
		if(ExistInImage)
		{
		     //fileout<<ss<<std::endl;
		     fileout<<index<<" "<<points->points[index].x<<" "<<points->points[index].y<<" "<<points->points[index].z<<" "<<rgb[0]<<" "<<rgb[1]<<" "<<rgb[2]<<"\n";
		}
	    }
	    fileout.close();
	    std::cout<<count<<std::endl;

	    //pcl::io::savePCDFileBinary(outfile, *points);
	}
	
}


bool get_setImagePixel(IplImage* img, int x, int y, std::vector<int> &rgb)
{
      CvScalar scalar; 
      rgb.clear();
      bool ExistInImage = false;
      int width = img->width;
      int height = img->height;
      int channels = img->nChannels;
      if(x < width && x>0 && y>0 && y < height)
      {
	    if(channels == 1)
	    {
		int pixel = cvGet2D(img,y,x).val[0];
		std::cout<<"the gray value is: "<<pixel<<std::endl;
		rgb.push_back(pixel);
	    }
	    else
	    {
	        scalar = cvGet2D(img,y,x);
		int b = scalar.val[0];
		int g = scalar.val[1];
		int r = scalar.val[2];
		rgb.push_back(r);
		rgb.push_back(g);
		rgb.push_back(b);
		count++;
		ExistInImage = true;
	    }
      }
      else 
      {
	    ExistInImage = false;
	    rgb.push_back(255);
	    rgb.push_back(255);
	    rgb.push_back(255);
      }
      return ExistInImage;
}

void PcloudToImage(double x, double y, double z,std::vector<double> &pixelXY)
{
      pixelXY.clear();
      double img_x,img_y,img_z;
      double cloud_x,cloud_y,cloud_z;
      double Tr_velo_to_cam[4][4] = {{0.007533745,-0.9999714,-0.000616602,-0.004069766},{0.01480249,0.0007280733,-0.9998902,-0.07631618},{0.9998621,-0.00752379,0.01480755,-0.2717806},{0,0,0,1}};
      double r0_rect[4][4] = {{0.999924,0.00984,-0.00745,0.0},{-0.00987,0.9999421,-0.00425,0.0},{0.0074025,0.0043516,0.9999631,0.0},{0.0,0.0,0.0,1.0}};
      double P2[3][4] = {{721.5377,0.0,609.5593,44.85728},{0.0,721.5377,172.854,0.2163791},{0.0,0.0,1.0,0.002729905}};
      double img_cordinate[4][1];
      double img_tmp[4][1];
      double img_uv[3][1];
      img_x = -y;
      img_y = -z - 0.08;
      img_z = x - 0.27;

      img_cordinate[0][0] = img_x;
      img_cordinate[1][0] = img_y;
      img_cordinate[2][0] = img_z;
      img_cordinate[3][0] = 1.0;

      double sum;
      for (int im = 0; im < 4; im++)
      {
	  sum = 0;
	  for (int j = 0; j < 1; j++)
	  {
	      for (int k = 0; k < 4; k++)
	      {
		  sum += r0_rect[im][k] * img_cordinate[k][j];
	      }
	      img_tmp[im][j] = sum;
	      sum = 0;
	  }
      }

      for (int im = 0; im < 3; im++)
      {
	  sum = 0;
	  for (int j = 0; j < 1; j++)
	  {
	      for (int k = 0; k < 4; k++)
	      {
		  sum += P2[im][k] * img_tmp[k][j];
	      }
	      img_uv[im][j] = sum;
	      sum = 0;
	  }
      }
      img_uv[0][0] = img_uv[0][0] / img_uv[2][0];
      img_uv[1][0] = img_uv[1][0] / img_uv[2][0];
      pixelXY.push_back((int)(img_uv[0][0]));
      pixelXY.push_back((int)(img_uv[1][0]));
}
	 

	  
	  
  
