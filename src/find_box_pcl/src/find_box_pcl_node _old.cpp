#include <iostream>
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
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandlerT;

#include <string>
#include <vector>
#include <ros/ros.h>
using namespace std;

double norm2(pcl::PointXYZRGB data)
{
    return (sqrt(pow(data.x,2)+pow(data.y,2)+pow(data.z,2)));
}
double norm2_between_points(pcl::PointXYZRGB center, pcl::PointXYZRGB point)
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointXYZRGB centroid;

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



void check_for_clusters_on_plane(vector<cloud_plane> &current_planes, cloud_plane plane_in_cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (plane_in_cloud.cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.04); // 2cm
    ec.setMinClusterSize (40);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (plane_in_cloud.cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (plane_in_cloud.cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_plane devided_plane;
        devided_plane.coefficients=plane_in_cloud.coefficients;
        devided_plane.cloud=cloud_cluster;
        calculate_centroid(devided_plane);
        current_planes.push_back(devided_plane);
    }


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
pcl::PointCloud<pcl::PointXYZ>::Ptr temp2;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr wow2(new pcl::PointCloud<pcl::PointXYZ>);
    temp2=wow2;
    temp=wow;


    pcl::PCLPointCloud2::Ptr cloud_blob;
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(
            new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

    ros::init(argc,argv,"PCL_Handler");

    ros::NodeHandle _nh;
//    pcl::visualization::PCLVisualizer visu2("wow");

    ros::Subscriber sub = _nh.subscribe ("/pico_flexx/points", 1, cloud_cb);

    ros::Publisher pub= _nh.advertise<sensor_msgs::PointCloud2> ("/pico_flexx/box", 1);

    while(ros::ok())
    {
        if (new_data == true) {
            new_data = false;
            cloud_blob=temp;

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_blob);
            sor.setLeafSize(0.005f, 0.005f, 0.005f);
            sor.filter(*cloud_filtered_blob);

            // Convert to the templated PointCloud
            pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);


            int i = 0;
            while (i != cloud_filtered->size()) {
                if (norm2(cloud_filtered->points[i]) > 0.5) {
                    cloud_filtered->erase(cloud_filtered->begin() + i);
                } else {
                    i++;
                }
            }
            //    // Write the downsampled version to disk
            //    pcl::PCDWriter writer;
            //    writer.write<pcl::PointXYZRGB> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);


            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            // Optional
            seg.setOptimizeCoefficients(true);
            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.002);

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;

            i = 0;
            int nr_points = (int) cloud_filtered->points.size();
    //        pcl::visualization::PCLVisualizer visu("Alignment");
            // While 30% of the original cloud is still there



            //    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> data;
            vector<cloud_plane> data;


            while (cloud_filtered->points.size() > 3)
                //    while (cloud_filtered->points.size () > 0.3 * nr_points)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);

                // Extract the inliers
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*temp);
//                cout << "Plane " << i << " is " << temp->points.size() << " Big " << endl;
                //        extract.filter (*cloud_p);
                //        std::cerr << "PointCloud representing the planar component: " << temp->width * temp->height << " data points." << std::endl;
                cloud_plane temp_cloud_plane;
                temp_cloud_plane.cloud = temp;
                temp_cloud_plane.coefficients = coefficients;
                if (data.size() > 0) {
                    //            if(norm2_coefficients(data[0].coefficients,temp_cloud_plane.coefficients)<0.08)
                    if (same_plane_coefficients(data[0].coefficients, temp_cloud_plane.coefficients)) {
                        *data[0].cloud += *temp_cloud_plane.cloud;
                    } else {
                        check_for_clusters_on_plane(data, temp_cloud_plane);
                        //                data.push_back(temp_cloud_plane);
                    }
                } else {
                    //            data.push_back(temp_cloud_plane);
                    check_for_clusters_on_plane(data, temp_cloud_plane);
                }


                // Create the filtering object
                extract.setNegative(true);
                extract.filter(*cloud_filtered);

                i++;

            }

    //        std::uniform_int_distribution<int> dist(0, 255);
    //        std::random_device rd;
    //        int red = dist(rd), green = dist(rd), blue = dist(rd);
    //        for (int j = 0; j < data.size(); j++) {
    //            cout << data[j].cloud->points.size() << endl;
    //            red = dist(rd), green = dist(rd), blue = dist(rd);
    //            visu.addPointCloud(data[j].cloud, ColorHandlerT(data[j].cloud, red, green, blue),
    //                               "scene" + std::to_string(j));
    //        }
            //    cout << "Size of temp: " << temp->points.size() << endl;
            //    cout << "Size of full cloud: " << cloud_filtered->points.size() << endl;
            //    visu.addPointCloud (cloud_filtered, ColorHandlerT (cloud_filtered, 255, 0, 0), "full");
            //    visu.addPointCloud (temp, ColorHandlerT (temp, 255, 255, 255), "temp");
    //        visu.spin();


            //    // The biggest plane is not going to be our box - most likely the table
            //    vector< vector<int> > possible_shit;
            vector<vector<int> > possible_shit(data.size(), vector<int>(data.size()));


            int index_j = 0, index_i = 0;
            double closest_to_zero = 99;
            double acceptance_criteria = 15.0;
            for (i = 1; i < data.size(); i++) {
                for (int j = 1; j < data.size(); j++) {
                    if (i != j && data[j].cloud->points.size() > 25) {
                        double a[3] = {data[i].coefficients->values[0], data[i].coefficients->values[1],
                                       data[i].coefficients->values[2]};
                        double b[3] = {data[j].coefficients->values[0], data[j].coefficients->values[1],
                                       data[j].coefficients->values[2]};
                        //                double dot_product=std::inner_product(std::begin(a), std::end(a), std::begin(b), 0.0); // shit
                        //                cout << "Plane " << i << " and plane " << j << " has a angle of " << abs(vector_angle_3d(a,b)) << endl;
                        if (abs(vector_angle_3d(a, b)) < 90 + acceptance_criteria &&
                            abs(vector_angle_3d(a, b)) > 90 - acceptance_criteria &&
                            norm2_between_points(data[i].centroid, data[j].centroid) < 0.06) {
                            possible_shit[i][j] = 1;
                            //                    cout << i << " " << j << endl;
                        } else {
                            possible_shit[i][j] = 0;
                        }
                        //                cout << "Plane " << i << " dot Plane " << j << " = " << dot_product << endl;
                        //                if(abs(dot_product)<abs(closest_to_zero))
                        //                {
                        //                    closest_to_zero=dot_product;
                        //                    index_i=i;
                        //                    index_j=j;
                        //                    cout << closest_to_zero << " " << index_i << " " << index_j << endl;
                        //                }
                    }
                }
            }
            vector<box> boxes;
            for (i = 1; i < data.size(); i++) {
                //        cout << "Plane " << i << " has ";
                vector<int> possible_matches;
                for (int j = 1; j < data.size(); j++) {
                    if (possible_shit[i][j] == 1) {
                        //                cout << j << " , ";
                        possible_matches.push_back(j);
                    }
                }
                //        cout << "as possible candidates" << endl;
                if (possible_matches.size() > 1) // at least two orthorgonal planes
                {
                    //            cout << "Revised candidates for " << i << " are: ";
                    //            cout << "WTF " << possible_matches.size()<< endl;
                    all_combinations(2, possible_matches.size());
                    vector<int> check_combinations = result;
                    //            cout << "Got combinations" << check_combinations.size() << endl;
                    for (int h = 0; h < check_combinations.size() / 2; h++) {
                        int index1 = possible_matches[check_combinations[h * 2 + 1]];
                        int index0 = possible_matches[check_combinations[h * 2]];
                        //                cout << h << endl;
                        //                cout << "Wow" <<  index0 << index1 << endl;
                        //                cout << check_combinations[h*2] <<  " " <<check_combinations[h*2+1] << endl;
                        double a[3] = {data[i].coefficients->values[0], data[i].coefficients->values[1],
                                       data[i].coefficients->values[2]};
                        double b[3] = {data[index0].coefficients->values[0], data[index0].coefficients->values[1],
                                       data[index0].coefficients->values[2]};
                        double c[3] = {data[index1].coefficients->values[0], data[index1].coefficients->values[1],
                                       data[index1].coefficients->values[2]};
                        double ab = abs(vector_angle_3d(a, b));// shit
                        double ac = abs(vector_angle_3d(a, c)); // shit
                        double bc = abs(vector_angle_3d(b, c)); // shit
                        bool ab_within = ab < 90 + acceptance_criteria && ab > 90 - acceptance_criteria;
                        bool ac_within = ac < 90 + acceptance_criteria && ac > 90 - acceptance_criteria;
                        bool bc_within = bc < 90 + acceptance_criteria && bc > 90 - acceptance_criteria;

                        if (ab_within && ac_within && bc_within) {
    //                        cout << "INDEX should work: " << i << " " << index0 << " " << index1 << endl;
                            vector<int> temp;
                            temp.push_back(i);
                            temp.push_back(index0);
                            temp.push_back(index1);
                            sort(temp.begin(), temp.end());
                            bool duplicate = false;
                            for (int l = 0; l < boxes.size(); l++) {
                                if (boxes[l].data_indexes[0] == temp[0] && boxes[l].data_indexes[1] == temp[1] &&
                                    boxes[l].data_indexes[2] == temp[2]) {
                                    duplicate = true;
                                }
                            }
                            if (duplicate == false) {
                                box new_box;
                                new_box.data_indexes[0] = temp[0];
                                new_box.data_indexes[1] = temp[1];
                                new_box.data_indexes[2] = temp[2];
                                boxes.push_back(new_box);
                            }
                        }
                    }
                }

            }
            if(boxes.size()>0) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_for_sending(new pcl::PointCloud<pcl::PointXYZRGB>);
                //            visu2.removeAllPointClouds();
                for (int l = 0; l < boxes.size(); l++) {
                    *temp_for_sending += *data[boxes[l].data_indexes[0]].cloud;
                    *temp_for_sending += *data[boxes[l].data_indexes[1]].cloud;
                    *temp_for_sending += *data[boxes[l].data_indexes[2]].cloud;
                    //            cout << boxes[l].data_indexes[0] << " " << boxes[l].data_indexes[1] << " " << boxes[l].data_indexes[2]
                    //                 << endl;
                    //                visu2.addPointCloud(data[boxes[l].data_indexes[0]].cloud,
                    //                                    ColorHandlerT(data[boxes[l].data_indexes[0]].cloud, 255, 0, 0),
                    //                                    "scene" + to_string(boxes[l].data_indexes[0]) + to_string(l));
                    //                visu2.addPointCloud(data[boxes[l].data_indexes[1]].cloud,
                    //                                    ColorHandlerT(data[boxes[l].data_indexes[1]].cloud, 0, 255, 0),
                    //                                    "scene" + to_string(boxes[l].data_indexes[1]) + to_string(l));
                    //                visu2.addPointCloud(data[boxes[l].data_indexes[2]].cloud,
                    //                                    ColorHandlerT(data[boxes[l].data_indexes[2]].cloud, 0, 0, 255),
                    //                                    "scene" + to_string(boxes[l].data_indexes[2]) + to_string(l));

                }
                //            pcl_conversions::to
                sensor_msgs::PointCloud2 msg;
                pcl::toROSMsg(*temp_for_sending.get(), msg);
                msg.header.frame_id="pico_flexx_optical_frame";
                msg.header.stamp=ros::Time::now();
                pub.publish(msg);
                cout << "FOUND SOMETHING" << endl;
            } else
            {
                cout <<"Did not find anything" << endl;
            }

    //
        }
//        visu2.spinOnce();
        ros::spinOnce();
    }
    return (0);
}