#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class Get_map_sensor_data
{
	protected:
		ros::NodeHandle nh_;
		int grid_size = 10;
		nav_msgs::OccupancyGrid World_Map;
		bool map_generated = false;
		bool cloud_generated = false;
		sensor_msgs::PointCloud2 pointCloud;
		int robot_pose[3] = {0,2,0}; // x-coordinate, y-coordinate and rotation about +ve z-axis (CCW is +ve)
		ros::Publisher cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 10);
		

		
	public:
		Get_map_sensor_data(void)
		{			
			Get_map_sensor_data::create_Map();
			Get_map_sensor_data::generate_pointCloud();
			Get_map_sensor_data::publish_pointCloud();
		}

		~Get_map_sensor_data(void)
		{
		}

		void create_Map()
		{
  			World_Map.info.width = grid_size;
			World_Map.info.height = grid_size;
			for (unsigned int i=0; i<grid_size; i++)
			{
				for(unsigned int j=0; j<grid_size; j++)
				{
					if((i==0 && j!=6) || (i!=4 && j==9) || (i>0 && i!=4 && i!=5 && j==0) || (i==9 && j!=7 && j!=8)) 
					{
						World_Map.data.push_back(1);
					}
					else if((i==3 && j==3) || (i==3 && j==6) || (i==6 && j==3) || (i==6 && j==6))
					{
						World_Map.data.push_back(1);
					}
					else
					{
						World_Map.data.push_back(0);
					}
				}
			}
			//World_Map.data[]
			map_generated = true;
		}
		
		void generate_pointCloud()
		{
			ros::Rate rate(5);			
			while(!map_generated)
			{
				rate.sleep();
			}

			unsigned int count = 0;
			int idx_map = (9-robot_pose[0])*grid_size + 9-robot_pose[1];
			int laser_view[3][5];
			int num_visible_obst = 0;
			Get_map_sensor_data::get_laser_view(idx_map, robot_pose, &laser_view, &num_visible_obst);

			pointCloud.header.frame_id = "laser";
			pointCloud.height = 1;
			pointCloud.width = num_visible_obst;
			pointCloud.is_bigendian = false;
			pointCloud.is_dense = false; // there may be invalid points
			sensor_msgs::PointCloud2Modifier modifier(pointCloud);
			modifier.setPointCloud2FieldsByString(1,"xyz");
			modifier.resize(num_visible_obst);
			pointCloud.header.stamp = ros::Time::now();			
			sensor_msgs::PointCloud2Iterator<float> out_x(pointCloud, "x");
			sensor_msgs::PointCloud2Iterator<float> out_y(pointCloud, "y");
			sensor_msgs::PointCloud2Iterator<float> out_z(pointCloud, "z");
			
			for (int i=0; i<3; i++)
			{
				for(int j=0; j<5; j++)
					{
						if(laser_view[i][j] == 1)
						{
							*out_x = float(3 - (i + 1));
							*out_y = float(5 - (j + 1) -2);
							*out_z = float(0);
							++out_x; ++out_y; ++out_z;
						}
					}
			}
			cloud_generated = true;
		}

		void publish_pointCloud()
		{
			ros::Rate pub_rate(1);
			ros::Rate rate(2);
			while(!cloud_generated && ros::ok())
			{
			rate.sleep();
			}
			while(cloud_generated && ros::ok())
			{
				cloud_pub_.publish(pointCloud);
				pub_rate.sleep();
			}

		}

		void update_robotPose()
		{
			robot_pose[0] = 0; // x-coordinate
			robot_pose[1] = 2; // y-coordinate
			robot_pose[2] = 0; // rotation about z- axis (CCW is positive)

		}

		void get_laser_view(int idx,int robot_pose[3], int (*laser_view)[3][5], int *num_visible_obst)
		{
			int start_idx;
			switch (robot_pose[2])
			{
				case 0:
				{
					start_idx = idx - (2*grid_size + 2);
					for(int i=0;i<3;i++)
					{
						for(int j=0;j<5;j++)
						{
							(*laser_view)[i][j] = int(World_Map.data[start_idx + i*grid_size + j]);
						}
					}
					break;
				}

				case 90:
				{
					start_idx = idx + 2*grid_size - 2;
					for(int i=0;i<3;i++)
					{
						for(int j=0;j<5;j++)
						{
							(*laser_view)[i][j] = int(World_Map.data[start_idx - j*grid_size + i]);
						}
					}
					break;
				}
				case 180:
				{
					start_idx = idx + 2*grid_size + 2;
					for(int i=0;i<3;i++)
					{
						for(int j=0;j<5;j++)
						{
							(*laser_view)[i][j] = int(World_Map.data[start_idx - i*grid_size - j]);
						}
					}
					break;
				}

				case 270:
				{
					start_idx = idx - 2*grid_size + 2;
					for(int i=0;i<3;i++)
					{
						for(int j=0;j<5;j++)
						{
							(*laser_view)[i][j] = int(World_Map.data[start_idx - j*grid_size - i]);
						}
					}
					break;
				}
				
				default:
				{break;}
			}

			/* In this ocluded cells have assigned -1, by assuming that the ray's originating from laser
			should directly go onto to the cell in-order to be visible if they are being completely or 
			partially obstructed by the occupied cells than they are not visible by the laser scanner*/
			if ((*laser_view)[2][3] == 1) {(*laser_view)[1][4]=-1; (*laser_view)[2][4]=-1;}
			if ((*laser_view)[1][3] == 1) {(*laser_view)[0][3]=-1; (*laser_view)[0][4]=-1; (*laser_view)[1][4]=-1;}
			if ((*laser_view)[1][2] == 1) {(*laser_view)[0][3]=-1; (*laser_view)[0][2]=-1; (*laser_view)[0][1]=-1;}
			if ((*laser_view)[1][1] == 1) {(*laser_view)[0][0]=-1; (*laser_view)[0][1]=-1; (*laser_view)[1][0]=-1;}
			if ((*laser_view)[2][1] == 1) {(*laser_view)[1][0]=-1; (*laser_view)[2][0]=-1;}

			for (int i=0; i<3; i++)
			{
				for(int j=0; j<5; j++)
					{
						if((*laser_view)[i][j] == 1){*num_visible_obst = *num_visible_obst + 1;}
					}
			}

		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_data");

  Get_map_sensor_data get_map;
  ros::NodeHandle nh_;

  ros::spin();

  return 0;
}
