///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, e-Con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**********************************************************************
PointCloudApp: Defines the methods to view the pointcloud image of the
			   scene.
**********************************************************************/

#include "PointCloudApp.h"
#include "../cpp_server_client/server.h"


//	GLOBAL VARIABLES

/**********************************************************************/

pcl::visualization::CloudViewer *g_PclViewer;//("Stereo Point Cloud Viewer");
Server server;

/**********************************************************************/

//Constructor
PointCloudApp::PointCloudApp(void)
{
	//init the variables
	Trans_x = 0.0;
	Trans_y = 0.0;
	Trans_z = 15.0;
	Rot_x = 0.0;
	Rot_y = 0.0;
	Rot_z = 0.0;
	m_MinDepth = 10000.0;
}


//Initialises all the necessary files
int PointCloudApp::Init()
{
	//Initialise the camera
	if(!_Disparity.InitCamera(true, true))
	{
		if(DEBUG_ENABLED)
			cout << "Camera Initialisation Failed\n";
		return FALSE;
	}

	//Camera Streaming
	CameraStreaming();

	return TRUE;
}


//Streams the input from the camera
int PointCloudApp::CameraStreaming()
{
	bool statusShowPointCloud = TRUE;


	//streaming
	while(1)
	{
		if(!_Disparity.GrabFrame(&LeftFrame, &RightFrame)) //Reads the frame and returns the rectified image
		{
			//delete g_PclViewer;
			cv::destroyAllWindows();
			break;
		}

		//Get disparity
		_Disparity.GetDisparity(LeftFrame, RightFrame, &gDisparityMap, &gDisparityMap_viz);

		//Pointcloud display
		statusShowPointCloud = ShowPointCloud();

	}

	return TRUE;
}



//The code below is gathering the points through the nested loops
int PointCloudApp::ShowPointCloud()
{
	#pragma region For Point Cloud Rendering

	cv::Mat xyz;
	reprojectImageTo3D(gDisparityMap, xyz, _Disparity.DepthMap, true);

	PointCloud<PointXYZRGB>::Ptr point_cloud_ptr (new PointCloud<PointXYZRGB>);
	const double max_z = 1.0e4;

	int count = 0;

	for (int Row = 0; Row < xyz.rows; Row++)
	{
		for (int Col = 0; Col < xyz.cols-100; Col++)
		{
			PointXYZRGB point;
			uchar Pix=(uchar)255;
			//Just taking the Z Axis alone
			cv::Vec3f Depth = xyz.at<cv::Vec3f>(Row,Col);
			point.x = Depth[0];
			point.y = -Depth[1];
			point.z = -Depth[2];

			if(fabs(Depth[2] - max_z) < FLT_EPSILON || fabs(Depth[2]) > max_z|| Depth[2] < 0 || Depth[2] > m_MinDepth)
				continue;

			Pix = LeftFrame.at<uchar>(Row, Col);
			uint32_t rgb = (static_cast<uint32_t>(Pix) << 16 |static_cast<uint32_t>(Pix) << 8 | static_cast<uint32_t>(Pix));

			point.rgb= *reinterpret_cast<float*>(&rgb);

			point_cloud_ptr->points.push_back (point);

		}

	}

	/*
	  Code below is rotating the point cloud data around its orgin
 	  http://www.pcl-users.org/Rotate-point-cloud-around-it-s-origin-td4040578.html
	*/

	point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;

	Eigen::Affine3f Transform_Matrix = Eigen::Affine3f::Identity();
	// Define a translation of 2.5 meters on the x axis.
	Transform_Matrix.translation() << Trans_x, Trans_y, Trans_z;

	// The same rotation matrix as before; tetha radians arround Z axis
	Transform_Matrix.rotate (Eigen::AngleAxisf (Rot_x, Eigen::Vector3f::UnitX()));
	Transform_Matrix.rotate (Eigen::AngleAxisf (Rot_y, Eigen::Vector3f::UnitY()));
	Transform_Matrix.rotate (Eigen::AngleAxisf (Rot_z, Eigen::Vector3f::UnitZ()));

	PointCloud<PointXYZRGB>::Ptr point_cloud_Transformed_ptr (new PointCloud<PointXYZRGB>);
	transformPointCloud (*point_cloud_ptr, *point_cloud_Transformed_ptr, Transform_Matrix);

	/*
          VoxelGridFilter applys a filter to reduce the total number of points
	  http://pointclouds.org/documentation/tutorials/voxel_grid.php


	VoxelGrid<pcl::PointXYZRGB> vg;
	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
	vg.setInputCloud(point_cloud_Transformed_ptr);
	vg.setLeafSize( 7.5f, 7.5f, 7.5f );
	vg.filter(*cloud_filtered);
*/
	for(int i; i < point_cloud_Transformed_ptr->points.size(); i++)
	{
		server.serverTask(point_cloud_Transformed_ptr->points[i]);
	}

	#pragma endregion For Point Cloud Rendering

	return TRUE;
}


//Main Function
int main()
{
	if(DEBUG_ENABLED)
	{
		cout << "Point Cloud - Application\n";
		cout << "-------------------------\n\n";
	}


	// start server first and wait for client to connect
	server.startServer();

	//Object creation
	PointCloudApp _PointCloud;
	_PointCloud.Init();


	if(DEBUG_ENABLED)
		cout << "Exit - Point Cloud Application\n";
	return TRUE;
}
