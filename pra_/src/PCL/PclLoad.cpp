
#include <stdio.h>
#include "PclOps.h"
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//设置背景颜色
	viewer.setBackgroundColor(1.0, 0, 1.0);
}

void print(){
    printf("this is a pclOpas---\n");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new  pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new  pcl::PointCloud<pcl::PointXYZRGB>);
    int8_t res=pcl::io::loadPCDFile("/home/zj/share_sdb5/tempData/DataSave/testLoam_pcd/1422133392.537796096.pcd", *cloud1); //加载点云文件  
    // printf("load pcd res:%d \n",res);
    if(res==-1){
        printf("load pcd error\n");
    }
	pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建viewer对象  
	// //blocks until the cloud is actually rendered  
	viewer.showCloud(cloud1);//将pcb与viewer对象联系起来
	// viewer.runOnVisualizationThreadOnce(viewerOneOff);
	while (!viewer.wasStopped())//要想让自己所创窗口一直显示，则加上 while (!viewer.wasStopped()){ };即可.
	{
	}
}