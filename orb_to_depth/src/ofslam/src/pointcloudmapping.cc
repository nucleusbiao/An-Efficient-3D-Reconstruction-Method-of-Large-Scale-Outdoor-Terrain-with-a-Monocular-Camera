/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "/home/zjd/orb_to_depth/devel/include/zoetodepth/depthimage.h"

int currentloopcount = 0;
PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    cout<<"receive a keyframe, id = "<<idk<<" 丄1�71ￄ1�77"<<kf->mnId<<"丄1�71ￄ1�77"<<endl;
    // std::string filename = "/home/gold/shujuji/depth/" + std::to_string(idk) + ".png";
    // cv::imwrite(filename, color);
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pointcloude.pcE = generatePointCloud(kf,color,depth);
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();
}

void PointCloudMapping::insertKeyFramemono(KeyFrame* kf, cv::Mat& color,int idk,vector<KeyFrame*> vpKFs)
{
    ros::NodeHandle nh;

    // 创建图像请求服务客户竄1�71ￄ1�771ￄ1�71ￄ1�777
    ros::ServiceClient client = nh.serviceClient<zoetodepth::depthimage>("depthimage");

    // cout<<"receive a keyframe, id = "<<idk<<" 丄1�71ￄ1�77"<<kf->mnId<<"丄1�71ￄ1�77"<<endl;
    std::string filename = "/home/zjd/shujuji/ip24orb+mvg+mvs/images/" + std::to_string(idk) + ".png";
    cv::imwrite(filename, color);
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
    sensor_msgs::Image msg1 = *msg;
    // cout << "height = " << msg1.height << endl;
    zoetodepth::depthimage srv;
    srv.request.rgbimage = *msg;
    // cout << "width = " << srv.request.rgbimage.width << endl;
    client.call(srv);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.depthimage, sensor_msgs::image_encodings::TYPE_32FC1);
    // cout << "width = " << cv_ptr->image.cols << endl;
    // cout << "height = " << cv_ptr->image.rows << endl;
    // std::string filename = "/home/zjd/shujuji/depth/" + std::to_string(idk) + ".png";
    // cv::imwrite(filename, cv_ptr->image);

    pointcloude.pcE = generatePointCloud(kf,color,cv_ptr->image);
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();
}


pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    PointCloud::Ptr tmp( new PointCloud() );
    // float d = depth.ptr<float>(0)[0];
    // cout << "d = " << d << endl;
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=1 )
    {
        for ( int n=0; n<depth.cols; n+=1 )
        {
            float d = depth.ptr<float>(m)[n];
            // if ((d / 256) < 2 || (d / 256)>5)
            //     continue;
            
            if(m < 30 || n < 30 || m > 450 || n > 610 || d > 1.75) continue;
            PointT p;
            // cout << d << endl;
            p.z = d;
            // p.z = 1.56971;
            // cout << "d = " << p.z << endl;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //PointCloud::Ptr cloud(new PointCloud);
    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    //cloud->is_dense = false;
    
    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    // cout << "tmp.size = " << tmp->size() << endl;
    return tmp;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
          //cout<<"loopbusy || bStop"<<endl;
            continue;
        }
        //cout<<lastKeyframeSize<<"    "<<N<<endl;
        if(lastKeyframeSize == N)
            cloudbusy = false;
        //cout<<"待处理点云个敄1�71ￄ1�771ￄ1�71ￄ1�777 = "<<N<<endl;
          cloudbusy = true;
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {

          
            PointCloud::Ptr p (new PointCloud);
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            //cout<<"处理好第i个点亄1�71ￄ1�771ￄ1�71ￄ1�777"<<i<<endl;
            *globalMap += *p;
            //PointCloud::Ptr tmp(new PointCloud());
            //voxel.setInputCloud( globalMap );
           // voxel.filter( *tmp );
            //globalMap->swap( *tmp );
           
 
        }
      
        // depth filter and statistical removal 
        PointCloud::Ptr tmp1 ( new PointCloud );
        // cout << "statistical_filter before globalMap = " << globalMap->size() << endl;
        statistical_filter.setInputCloud(globalMap);
        statistical_filter.filter( *tmp1 );
        // cout << "statistical_filter after globalMap = " << tmp1->size() << endl;
        
        // PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *globalMap );

        //globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout<<"After volxel globalMap = " << globalMap->points.size() << endl;
        lastKeyframeSize = N;
        cloudbusy = false;
        //*globalMap = *tmp1;
        
        //if()
        //{
	    
	//}
    }
}
void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}
void PointCloudMapping::updatecloud()
{
	if(!cloudbusy)
	{
		loopbusy = true;
		cout<<"startloopmappoint"<<endl;
        PointCloud::Ptr tmp1(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {   
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId) 
				{   
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;

					//cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
					continue;
				}
			}
		}
        cout<<"finishloopmap"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );
        //viewer.showCloud( globalMap );
        loopbusy = false;
        //cloudbusy = true;
        loopcount++;

        //*globalMap = *tmp1;
	}
}
