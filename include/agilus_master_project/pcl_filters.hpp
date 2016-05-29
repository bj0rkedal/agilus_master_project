//
// Original author Kristoffer Larsen. Latest change date 01.05.2016
// This .hpp file defines the content of pcl_filters.cpp which is a toolbox that implements the most commonly used features from PCL.
// In this application, pcl_filters.cpp is used for 3D point cloud processing.
//
// Created as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//


#ifndef qt_filter_tester_PCLFILTERS_H
#define qt_filter_tester_PCLFILTERS_H

#include <QObject>
#include <iostream>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral_omp.h>

//PCL Feature estimation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include "pcl/keypoints/sift_keypoint.h"

//PCL Registration and object detection
#include <pcl/registration/ia_ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>

/*!
 * \brief The RayTraceCloud struct contain all the data related to a training set model.
 */
struct RayTraceCloud {
    /*! The point cloud of the ray trace */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    /*! The pose transformation from the camera to the mesh when the ray trace was generated */
    Eigen::Matrix4f pose;

    /*! The amount of the whole mesh seen in the camera */
    float enthropy;

    /*! The clouds keypoints */
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;

    /*! The clouds surface normal */
    pcl::PointCloud<pcl::Normal>::Ptr normals;

    /*! The clouds local descriptors */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptors;

    /*! The clouds global descriptor */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptors;
};

/*!
 * \brief The icpResult struct contain all the important data derived from the 3D object detection procedure.
 */
struct icpResult {
    /*! A 3D point cloud that illustrates the 3D object detection result */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    /*! The estimated pose of part A */
    Eigen::Matrix4f partAFinal;

    /*! The estimated pose of part B */
    Eigen::Matrix4f partBFinal;
};


namespace agilus_master_project {

class PclFilters : public QObject{
Q_OBJECT

public:

    /*!
     * \brief Constructor for the PclFilters class.
     * \param parent Default 0.
     */
    PclFilters(QObject *parent = 0);

    ~PclFilters();

    /*!
     * \brief Return the cluster that is most similar to the provided training set.
     * \param clusters Clusters that is to be searched.
     * \param model The training set of the model that is searched for in the scene.
     * \return The index that corresponds to the best matching cluster in the input cluster vector.
     */
    int search_for_model(std::vector<RayTraceCloud> clusters, std::vector<RayTraceCloud> model);

    /*!
     * \brief Method for testing an experimental 3D object detection implementation. This implementation is based on RANSAC.
     * \param models Training set that is to be searched for.
     * \param object Object cluster.
     */
    void ransac_recognition(std::vector<RayTraceCloud> models, RayTraceCloud object);

    /*!
     * \brief Estimates an initial alignment bewteen the source and target model based on the Sigular Value Decomposition approach.
     * \param source The source point cloud.
     * \param target The target point cloud.
     * \param min_sample_distance Mathcing parameter used to limit correspondences.
     * \param max_correspondence_distance Matching parameter used to limit correspondences.
     * \param nr_iterations Maximum number of iterations run before returning a pose.
     * \return The estimated initial alignment.
     */
    Eigen::Matrix4f calculateInitialAlignment(RayTraceCloud source, RayTraceCloud target, float min_sample_distance, float max_correspondence_distance, int nr_iterations);

    /*!
     * \brief Estimates a final alignment between the source and target model based on the Iterative Closest Point approach.
     * \param source The source point cloud.
     * \param target The target point cloud.
     * \param initial_alignment The initial alignment between the models used as a starting point.
     * \param max_correspondence_distance Matching parameter used to limit correspondences.
     * \param outlier_rejection_threshold Parameter used to set the outlier rejection threshold.
     * \param transformation_epsilon Parameter that defines an acceptable transformation epsilon.
     * \param eucledian_fitness_epsilon Parameter that defines an acceptable eucledian fitness epsilon.
     * \param max_iterations Maximum number of iterations run before returning a pose.
     * \return The estimated final alignment.
     */
    Eigen::Matrix4f calculateRefinedAlignment (RayTraceCloud source, RayTraceCloud target, Eigen::Matrix4f initial_alignment, float max_correspondence_distance, float outlier_rejection_threshold, float transformation_epsilon, float eucledian_fitness_epsilon, int max_iterations);

    /*!
     * \brief Generates a Kd-tree used for nearest neighbour search from a set of global descriptors.
     * \param Training set containing global descriptors.
     * \return The generated Kd-tree.
     */
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr generate_search_tree(std::vector<RayTraceCloud> models);

    /*!
     * \brief Descriptor matching between the input model and a generated Kd-tree based on the VFH global descriptor.
     * \param object_model The input model.
     * \param search_tree The input Kd-tree
     * \return A vector containing <index of best matching model, confidence level of the match>.
     */
    std::vector<float> match_cloud(RayTraceCloud object_model, pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree);

    /*!
     * \brief Descriptor matching between the input model and a generated Kd-tree based on the CVFH global descriptor.
     * \param object_model The input model.
     * \param search_tree The input Kd-tree.
     * \return A vector containing <index of best matching model, confidence level of the match>.
     */
    std::vector<float> temp_matching_cvfh(RayTraceCloud object_model, pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree);

    /*!
     * \brief Calculated the keypoints of an input point cloud based on the SIFT3D keypoint selector method.
     * \param cloud The input point cloud.
     * \param min_scale SIFT3D minimum scale parameter.
     * \param nr_octaves SIFT3D number of octaves calcualted parameter.
     * \param nr_scales_per_octave SIFT3D number of scales per octave calculated parameter.
     * \param min_contrast SIFT3D minimum constrast parameter.
     * \return A point cloud containing the resulting keypoints.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr calculate_keypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast);

    /*!
     * \brief Estimates the FPFH local descriptor for a 3D point cloud.
     * \param cloud The input point cloud.
     * \param normal The surface normals of the input point cloud.
     * \param keypoints The keypoints of the input point cloud.
     * \param feature_radius The feature radius FPFH parameter.
     * \return A point cloud containing a local descriptor for each keypoint of the original input point cloud.
     */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculate_local_descritor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, float feature_radius);

    /*!
     * \brief Estimates the VFH global descriptor for a 3D point cloud.
     * \param points The input point cloud.
     * \param normals The surface normals of the input point cloud.
     * \return The VFH global descriptor for the input point cloud.
     */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculate_vfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals);


    /*!
     * \brief Estimates surface normals, keypoints, local descriptors and global descriptor for the input point cloud.
     * \param inputcloud The input point cloud.
     * \return A struct containing all the calculated features.
     */
    RayTraceCloud calculate_features(RayTraceCloud inputcloud);

    /*!
     * \brief Creates a PCL Visualizer containing the input cloud.
     * \param cloud Input point cloud.
     * \return A pcl visualizer containing the input cloud.
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Creates a PCL Visualizer to visualize a PointXYZRGB point cloud.
     * \param cloud Input point cloud.
     * \return A pcl visualizer containing the input cloud.
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /*!
     * \brief Creates a PCL Visualizer used to visualize a point clouds normals.
     * \param cloud Input point cloud.
     * \param radius Double value for the search radius for the normal estimation of a p.oint cloud.
     * \param numOfNormals Integer value for the number of normals to display in the visualizer.
     * \return A pcl visualizer containing the input cloud and normals as defined by the input parameters.
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int numOfNormals);

    /*!
     * \brief Segments out the biggest plane in the input point cloud.
     * \param cloud Input point cloud.
     * \param distance Double value for the maximum distance between points in a plane.
     * \return A pcl point cloud containing the points of the segmented plane.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance);

    /*!
     * \brief Creates a PCL Visualizer containing a point cloud of the clusters extracted from the input cloud using Eucledian cluster extraction.
     * \param cloud Input point cloud.
     * \param distance Double value for the maximum distance between points in a plane.
     * \return A pcl visualizer containing the extracted clusters from the input cloud.
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_extraction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance);

    /*!
     * \brief Returns the most recent point cloud handled by the class.
     * \return The most recent point cloud handled by the class.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_cloud();

    /*!
     * \brief Colors all the points in a point cloud.
     * \param cloud Input point cloud.
     * \param r Integer value for the red component of the color.
     * \param g Integer value for the green component of the color.
     * \param b Integer value for the blue component of the color.
     * \return The input point cloud colored in the specified color.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

    /*!
     * \brief Returns the surface normals of a Point cloud.
     * \param cloud Input point cloud.
     * \param radius Double value for the search radius of the normal estimation.
     * \return The surface normals of the input point cloud.
     */
    pcl::PointCloud<pcl::Normal>::Ptr get_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);

    /*!
     * \brief Returns the a point cloud filtered using passthrough filtering.
     * \param cloud Input point cloud.
     * \param min Double value for the minimum value of the filter.
     * \param max Double value for the maximum value of the filter.
     * \param axis std::string value for the axis of the filter (lower case).
     * \return A point cloud filtered using passthrough filtering as specified.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);

    /*!
     * \brief Returns the a point cloud filtered using voxel grid filtering.
     * \param cloud Input point cloud.
     * \param lx Double value for the voxel size in the "x" axis of the filter.
     * \param ly Double value for the voxel size in the "y" axis of the filter.
     * \param lz Double value for the voxel size in the "z" axis of the filter.
     * \return A point cloud filtered using voxel grid filtering as specified.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);

    /*!
     * \brief Returns the a point cloud filtered using median filtering.
     * \param cloud Input point cloud.
     * \param window_size Integer value for the window size of the filter.
     * \param max_allowed_movement Double value for the maximum allowed movenet of the filter.
     * \return A point cloud filtered using median filtering as specified.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr median_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);

    /*!
     * \brief Returns the a point cloud filtered using shadow point removal filtering.
     * \param cloud Input point cloud.
     * \param threshold Double value for the filter threshold.
     * \param radius Double value for the filter search radius.
     * \return A point cloud filtered using shadow point removal filtering as specified.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr shadowpoint_removal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius);

    /*!
     * \brief Returns the a point cloud filtered using statistical outlier removal filtering.
     * \param cloud Input point cloud.
     * \param meanK Integer value for the number of nearest neighbors to use for mean distance estimation.
     * \param std_deviation_threshold Double value for the standard deviation multiplier for the distance threshold calculation.
     * \return A point cloud filtered using statistical outlier removal filtering as specified.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);

    /*!
     * \brief Combines all clouds in an vector to one cloud.
     * \param input std::vector containing all clouds to be combined.
     * \return A point cloud containing all clouds in the input vector.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr combine_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input);

    /*!
     * \brief Generates the CVFH descriptors for an object (cluster).
     * \param object Input point cloud, cluster of the object.
     * \param normals Input normal cloud of the object.
     * \return The corresponding CVFH global descriptor.
     */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculate_cvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr object);

    /*!
     * \brief Returns a pcl point cloud filtered using a bilateral filter.
     * \param cloud Input point cloud.
     * \param sigmaS Double value for the half size of the gaussian bilateral filter window.
     * \param sigmaR Double value for the standard deviation parameter.
     * \return A point cloud filtered using a bilateral filter.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr bilateral_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sigmaS, double sigmaR);

    /*!
     * \brief Returns the esf descriptor for a pcl point cloud.
     * \param cloud Input point cloud.
     * \return ESDSignature640 descriptor for the input point cloud.
     */
    pcl::PointCloud<pcl::ESFSignature640>::Ptr calculate_esf_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Returns the ourcvfh descriptor for a pcl point cloud.
     * \param cloud Input point cloud.
     * \return OurCVFH descriptor for the input point cloud.
     */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr calculate_ourcvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                             pcl::PointCloud<pcl::Normal>::Ptr normal);
    /*!
     * \brief Returns the gfpfh descriptor for a pcl point cloud.
     * \param cloud Input point cloud.
     * \return GFPFH descriptor for the input point cloud.
     */
    pcl::PointCloud<pcl::GFPFHSignature16>::Ptr calculate_gfpfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Detects two objects in an area of the point cloud and returns a pointcloud showing the detected parts.
     * \param cloud Input point cloud.
     * \param model_a A list of RayTraceCloud objects generated by the ModelLoader class.
     * \param model_b A list of RayTraceCloud objects generated by the ModelLoader class.
     * \return Pointcloud showing the result.
     */
    icpResult object_detection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<RayTraceCloud> model_a, std::vector<RayTraceCloud> model_b);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; //!< A pcl viewer used to visualize pcl point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud; //!< The product of a filter operation.
    pcl::PassThrough<pcl::PointXYZ> passfilter; //!< A pcl passthrough filter object.
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter; //!< A pcl voxelgrid filter object.
    pcl::MedianFilter<pcl::PointXYZ> medianfilter; //!< A pcl median filter object.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier; //!< A pcl statistical outlier removal filter object.
    pcl::ShadowPoints<pcl::PointXYZ, pcl::Normal> shadowpoint_filter; //!< A pcl shadowpoints removal filter object.
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree_;

public Q_SLOTS:
    //Slots used to recieve events from one another class. All slots and signals are connected in main_window.cpp

Q_SIGNALS:
    //Signals used to emit event from one class to another. All signals are connected in main_window.cpp
};
}
#endif
