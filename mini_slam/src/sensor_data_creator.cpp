#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"

class Get_map_sensor_data
{
	protected:
		ros::NodeHandle nh_;
		int size = 10;
		nav_msgs::OccupancyGrid World_Map;
		bool map_generated = false;

		
	public:
		Get_map_sensor_data(void)
		{
			Get_map_sensor_data::create_Map();
		}

		~Get_map_sensor_data(void)
		{
		}

		void create_Map()
		{
  			World_Map.info.width = size;
			World_Map.info.height = size;
			ROS_INFO("Map Generated is as follows");
			for (unsigned int i=0; i<size; i++)
			{
				for(unsigned int j=0; j<size; j++)
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
			map_generated = true;
		}
		
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_data");

  Get_map_sensor_data get_map;
  ros::spin();

  return 0;
}
