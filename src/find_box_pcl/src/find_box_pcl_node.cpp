#include <iostream>
#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <random>
#include <map>
#include <algorithm>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <ros/ros.h>

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

#include <string>
#include <vector>
using namespace std;

double norm2(PointNT data)
{
    return (sqrt(pow(data.x,2)+pow(data.y,2)+pow(data.z,2)));
}
double norm2_between_points(PointNT center, PointNT point)
{
    return (sqrt(pow(point.x-center.x,2)+pow(point.y-center.y,2)+pow(point.z-center.z,2)));
}
double norm2_coefficients(pcl::ModelCoefficients::Ptr data1,pcl::ModelCoefficients::Ptr data2)
{
    return (sqrt(pow(data1->values[0]-data2->values[0],2)+pow(data1->values[1]-data2->values[1],2)+pow(data1->values[2]-data2->values[2],2))+pow(data1->values[3]-data2->values[3],2));
}
double vector_angle_3d(double v1[3],double v2[3])
{
//    https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
//    dot = x1*x2 + y1*y2 + z1*z2    #between [x1, y1, z1] and [x2, y2, z2]
//    lenSq1 = x1*x1 + y1*y1 + z1*z1
//    lenSq2 = x2*x2 + y2*y2 + z2*z2
//    angle = acos(dot/sqrt(lenSq1 * lenSq2))
    double dot=v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
    double lenSq1 = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
    double lenSq2 = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    return (acos(dot/sqrt(lenSq1 * lenSq2))*180/M_PI);
}

bool same_plane_coefficients(pcl::ModelCoefficients::Ptr data1,pcl::ModelCoefficients::Ptr data2)
{
    double n1[3]={data1->values[0],data1->values[1],data1->values[2]};
    double n2[3]={data2->values[0],data2->values[1],data2->values[2]};
    if(abs(vector_angle_3d(n1,n2))<5 && abs(data1->values[3]-data2->values[3])<0.015)
    {
        return true;
    }
    return false;
}
struct cloud_plane
{
    pcl::ModelCoefficients::Ptr coefficients;//=new pcl::ModelCoefficients();
    pcl::PointCloud<PointNT>::Ptr cloud;
    PointNT centroid;
};

struct box
{
    int data_indexes[3];
};

void calculate_centroid(cloud_plane &element)
{
    double x=0,y=0,z=0;
    for(int i=0; i<element.cloud->points.size() ; i++)
    {
        x+=(double)element.cloud->points[i].x;
        y+=(double)element.cloud->points[i].y;
        z+=(double)element.cloud->points[i].z;
    }
    x=x/(double)element.cloud->points.size();
    y=y/(double)element.cloud->points.size();
    z=z/(double)element.cloud->points.size();
    element.centroid.x=x;
    element.centroid.y=y;
    element.centroid.z=z;
}

PointNT calculate_centroid_cloud(pcl::PointCloud<PointNT>::Ptr cloud)
{
    double x=0,y=0,z=0;
    for(int i=0; i<cloud->points.size() ; i++)
    {
        x+=(double)cloud->points[i].x;
        y+=(double)cloud->points[i].y;
        z+=(double)cloud->points[i].z;
    }
//    cout << x << " " << y << " " << z << endl;
    PointNT centroid;
    x=x/(double)cloud->points.size();
    y=y/(double)cloud->points.size();
    z=z/(double)cloud->points.size();
//    cout << x << " " << y << " " << z << endl;
    centroid.x=x;
    centroid.y=y;
    centroid.z=z;
    return centroid;
}


pcl::PointCloud<PointNT>::Ptr check_for_clusters_on_plane(vector<cloud_plane> &current_planes, cloud_plane plane_in_cloud, int accepted_count=50)
{
//    cout << "HVAAAAAE" << endl;
    pcl::PointCloud<PointNT>::Ptr rejected_points(new pcl::PointCloud<PointNT>);
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointNT>::Ptr tree (new pcl::search::KdTree<PointNT>);
    tree->setInputCloud (plane_in_cloud.cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointNT> ec;
    ec.setClusterTolerance (0.04); // 2cm
    ec.setMinClusterSize (25);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (plane_in_cloud.cloud);
    ec.extract (cluster_indices);
//    cout << cluster_indices.data()->indices.size() << endl;
    cout << "We have the following amout of point clouds: " << endl;
    for(int i=0; i<cluster_indices.size(); i++)
        cout << cluster_indices[i].indices.size() << endl;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
//        if(it->indices.size()>accepted_count) {
        pcl::PointCloud<PointNT>::Ptr cloud_cluster(new pcl::PointCloud<PointNT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(plane_in_cloud.cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_plane devided_plane;
        devided_plane.coefficients = plane_in_cloud.coefficients;
        devided_plane.cloud = cloud_cluster;
        calculate_centroid(devided_plane);
        current_planes.push_back(devided_plane);
        cout << "WE ADDED PLANE!!!!" << endl;
//        } else
//        {
//            cout << "Plane rejected with size: " << it->indices.size() << endl;
//            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
//                rejected_points->points.push_back(plane_in_cloud.cloud->points[*pit]);
//        }
    }
    return rejected_points;

}

// SO ugly
vector<int> combination, indexes,result;
void go(int offset, int k) {
    if (k == 0) {
        for(int i=0; i<combination.size() ; i++)
        {
            result.push_back(combination[i]);
        }
        return;
    }
    for (int i = offset; i <= indexes.size() - k; ++i) {
        combination.push_back(indexes[i]);
        go(i+1, k-1);
        combination.pop_back();
    }
}
vector<int> all_combinations( int k, int n) {

    result.clear();
    indexes.clear();
    combination.clear();

    if (k == 0 || n==0) {
        cerr << "You are stupid at making combinations" << endl;

    }
    for(int i=0; i<n ;i++)
    {
        indexes.push_back(i);
    }
    go(0, k);

    return result;
}

// end so ugly



#include <sensor_msgs/PointCloud2.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

pcl::PCLPointCloud2::Ptr temp;
pcl::PointCloud<PointNT>::Ptr temp2;
bool new_data=false;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//    pcl_conversions::toPCL(*input,*temp);

    pcl::fromROSMsg(*input,*temp2);
    pcl::toPCLPointCloud2(*temp2, *temp);
//    pcl::toPCLPointCloud2(input,temp);
    new_data=true;
//    pcl_conversions::toPCL()
}

int
main (int argc, char** argv) {
    pcl::PCLPointCloud2::Ptr wow(new pcl::PCLPointCloud2);
    pcl::PointCloud<PointNT>::Ptr wow2(new pcl::PointCloud<PointNT>);
    temp2=wow2;
    temp=wow;


    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<PointNT>::Ptr cloud_filtered(new pcl::PointCloud<PointNT>), cloud_p(
            new pcl::PointCloud<PointNT>), cloud_f(new pcl::PointCloud<PointNT>);

    PointCloudT::Ptr object (new PointCloudT);
    pcl::console::print_highlight ("Loading point clouds...\n");
    if (pcl::io::loadPCDFile<PointNT> ("/home/jepod13/bagfiles/new_model.pcd", *object) < 0)
    {
        pcl::console::print_error ("Error loading object/scene file!\n");
        return (1);
    }
    PointCloudT::Ptr object_aligned (new PointCloudT),object_aligned_5_cm (new PointCloudT) ;


    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);



    ros::init(argc,argv,"PCL_Handler");

    ros::NodeHandle _nh;
//    pcl::visualization::PCLVisualizer visu2("wow");

    ros::Subscriber sub = _nh.subscribe ("/pico_flexx/points", 1, cloud_cb);

    ros::Publisher pub= _nh.advertise<sensor_msgs::PointCloud2> ("/pico_flexx/box", 1);

    while(ros::ok())
    {
        if (new_data == true) {
            new_data = false;
            cloud_blob = temp;

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_blob);
            sor.setLeafSize(0.005f, 0.005f, 0.005f);
            sor.filter(*cloud_filtered_blob);

            // Convert to the templated PointCloud
            pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);


            PointNT center_of_interest;
            center_of_interest.x = 0;
            center_of_interest.y = 0;
            center_of_interest.z = 0.35;

            int i = 0;
            while (i != cloud_filtered->size()) {
                if (norm2_between_points(cloud_filtered->points[i], center_of_interest) > 0.20) {
                    cloud_filtered->erase(cloud_filtered->begin() + i);
                } else {
                    i++;
                }
            }


            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            // Create the segmentation object
            pcl::SACSegmentation<PointNT> seg;
            // Optional
            seg.setOptimizeCoefficients(true);

            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.002);

            // Create the filtering object
            pcl::ExtractIndices<PointNT> extract;

            vector<cloud_plane> data;

            bool b_break = false;
            i = 0;
            while (b_break == false)
//    while (cloud_filtered->points.size () > 0.3 * nr_points)
            {

                pcl::PointCloud<PointNT>::Ptr temp(new pcl::PointCloud<PointNT>);
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);


                if (inliers->indices.size() != 0) {
                    // Extract the inliers
                    extract.setInputCloud(cloud_filtered);
                    extract.setIndices(inliers);
                    extract.setNegative(false);
                    extract.filter(*temp);
//                    cout << "Plane " << i << " is " << temp->points.size() << " Big " << endl;
//                    cout << "We have following number of planes: " << data.size() << endl;
                    //        extract.filter (*cloud_p);
                    //        std::cerr << "PointCloud representing the planar component: " << temp->width * temp->height << " data points." << std::endl;
                    cloud_plane temp_cloud_plane;
                    temp_cloud_plane.cloud = temp;
                    temp_cloud_plane.coefficients = coefficients;


                    if (data.size() > 0) {
                        if (same_plane_coefficients(temp_cloud_plane.coefficients, data[0].coefficients)) {
                            *data[0].cloud += *temp_cloud_plane.cloud;
                        } else {
                            check_for_clusters_on_plane(data, temp_cloud_plane);
//                    data.push_back(temp_cloud_plane);
                        }
                    } else {
                        data.push_back(temp_cloud_plane);
                    }


                    // Create the filtering object
                    extract.setNegative(true);
                    extract.filter(*cloud_filtered);
//            if(rejected_points!= nullptr)
//                *cloud_filtered+=*rejected_points;
//            cout << "Our new big cloud is of size: " << cloud_filtered->points.size() << endl;
                    i++;
                } else {
                    b_break = true;
                }

            }

            // Look for planes with at least one orthogonal plane, which is relatively close
            vector<vector<int> > possible_shit(data.size(), vector<int>(data.size()));

            double acceptance_criteria = 30.0;
            for (i = 1; i < data.size(); i++) {
                for (int j = 1; j < data.size(); j++) {
                    if (i != j && data[j].cloud->points.size() > 0) {
                        double a[3] = {data[i].coefficients->values[0], data[i].coefficients->values[1],
                                       data[i].coefficients->values[2]};
                        double b[3] = {data[j].coefficients->values[0], data[j].coefficients->values[1],
                                       data[j].coefficients->values[2]};
//                double dot_product=std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0); // shit
//                cout << "Plane " << i << " and plane " << j << " has a angle of " << abs(vector_angle_3d(a,b)) << endl;
                        if (abs(vector_angle_3d(a, b)) < 90 + acceptance_criteria &&
                            abs(vector_angle_3d(a, b)) > 90 - acceptance_criteria &&
                            norm2_between_points(data[i].centroid, data[j].centroid) < 0.08) {
                            possible_shit[i][j] = 1;
//                            cout << "####################################" << endl;
//                            cout << "Following planes matched: " << i << " " << j << endl;
                        } else {
                            possible_shit[i][j] = 0;
//                            cout << "Following DID NOT MATCH : " << i << " " << j << endl;
//                            cout << "Centroid of i: " << data[i].centroid.x <<  " " << data[i].centroid.y <<  " " << data[i].centroid.z <<  endl;
//                            cout << "Centroid of j: " << data[j].centroid.x <<  " " << data[j].centroid.y <<  " " << data[j].centroid.z <<  endl;
//                            cout << "Distance is: " <<  norm2_between_points(data[i].centroid,data[j].centroid) << endl;
//                            cout << "Angle is: " << abs(vector_angle_3d(a,b)) << endl;
                        }
                    }
                }
            }






//            cout << "We have " << data.size() << " planes" << endl;
//            cout << "Our table is " << data[0].cloud->points.size() << " points big" << endl;
//    pcl::visualization::PCLVisualizer visu("Alignment");
            PointCloudT::Ptr scene(new PointCloudT);
            for (int j = 1; j < data.size(); j++) {
                bool use_it = false;
                for (int h = 1; h < data.size(); h++) {
                    if (j != h) {
                        if (possible_shit[j][h] == 1) {
                            use_it = true;
                        }
                    }
                }
                if (use_it == true) {
                    *scene += *data[j].cloud;
                }
            }
            if (scene->points.size() > 0) {
                //            object_aligned_5_cm;
                // First translated fully
                double object_alligned_fit = 0;
                PointNT scene_centroid = calculate_centroid_cloud(scene);
                //    cout << "scene_centroid c: " << scene_centroid.x << " " << scene_centroid.y << " " << scene_centroid.z << endl;
                PointNT object_centroid = calculate_centroid_cloud(object);
                //    cout << "Object c: " << object_centroid.x << " " << object_centroid.y << " " << object_centroid.z << endl;
                double x = scene_centroid.x - object_centroid.x;
                double y = scene_centroid.y - object_centroid.y;
                double z = scene_centroid.z - object_centroid.z;
                //            cout << x << " " << y << " " << z  << endl;
                Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                transform(0, 3) = x;
                transform(1, 3) = y;
                transform(2, 3) = z;
                pcl::transformPointCloud(*object, *object, transform);
                //    cout << "Object c2: " << object_centroid.x << " " << object_centroid.y << " " << object_centroid.z << endl;

                pcl::IterativeClosestPoint<PointNT, PointNT> icp;
                icp.setInputSource(object);
                icp.setInputTarget(scene);
                icp.setMaxCorrespondenceDistance(0.12);
                icp.setMaximumIterations(100);
                icp.align(*object_aligned);

                icp.setInputSource(object_aligned);
                icp.setMaxCorrespondenceDistance(0.02);
                icp.setMaximumIterations(1000);
                icp.align(*object_aligned);

                icp.setInputSource(object_aligned);
                icp.setMaxCorrespondenceDistance(0.005);
                icp.setMaximumIterations(1000);
                icp.align(*object_aligned);
                object_alligned_fit = icp.getFitnessScore(0.01);

                double object_alligned_fit_5cm = 0;
                scene_centroid = calculate_centroid_cloud(scene);
                object_centroid = calculate_centroid_cloud(object);
                x = scene_centroid.x - object_centroid.x;
                y = scene_centroid.y - object_centroid.y;
                z = scene_centroid.z - object_centroid.z;
                //            cout << x << " " << y << " " << z  << endl;
                transform = Eigen::Matrix4f::Identity();
                transform(0, 3) = x;
                transform(1, 3) = y;
                transform(2, 3) = z - 0.05;
                pcl::transformPointCloud(*object, *object, transform);

                icp.setInputSource(object);
                icp.setInputTarget(scene);
                icp.setMaxCorrespondenceDistance(0.12);
                icp.setMaximumIterations(100);
                icp.align(*object_aligned_5_cm);

                icp.setInputSource(object_aligned_5_cm);
                icp.setMaxCorrespondenceDistance(0.02);
                icp.setMaximumIterations(1000);
                icp.align(*object_aligned_5_cm);

                icp.setInputSource(object_aligned_5_cm);
                icp.setMaxCorrespondenceDistance(0.005);
                icp.setMaximumIterations(1000);
                icp.align(*object_aligned_5_cm);
                object_alligned_fit_5cm = icp.getFitnessScore(0.01);

                //            cout << object_alligned_fit << " " << object_alligned_fit_5cm <<endl;

                sensor_msgs::PointCloud2 msg;

                if (object_alligned_fit_5cm<object_alligned_fit && object_alligned_fit_5cm<7.5e-05) {
                    //                cout << "5 cm did a HUGE difference " << endl;
                    pcl::toROSMsg(*object_aligned_5_cm.get(), msg);
                    msg.header.frame_id = "pico_flexx_optical_frame";
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);
                }
                else if(object_alligned_fit_5cm>object_alligned_fit && object_alligned_fit<7.5e-05)
                {
                    //                cout << "Length did not matter" << endl;
                    pcl::toROSMsg(*object_aligned.get(), msg);
                    msg.header.frame_id = "pico_flexx_optical_frame";
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);
                } else{
                    cout << "No Match for ICP" << endl;
                }

            } else {
                cout << "no Matches" << endl;
            }
        }

        ros::spinOnce();
    }
    return (0);
}