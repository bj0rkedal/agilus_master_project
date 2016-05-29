//
// Original code by Adam Leon Kleppe on 01.02.16, modifiead by Kristoffer Larsen.
// Latest change date 01.05.2016
// modelloader.cpp is a tool used to create training set used for 3D object detection.
//
// Modified and used as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//

#ifndef qt_filter_tester_MODELLOADER_H
#define qt_filter_tester_MODELLOADER_H

#include <QObject>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <string.h>
#include <sstream>
#include <iomanip>

#include "pcl_filters.hpp"

namespace agilus_master_project {

class ModelLoader : public QObject
{
    Q_OBJECT

public:
    /*!
     * \brief Constructor for the ModelLoader class
     * \param mesh Polygon Mesh that will be used to create a trining set.
     * \param mesh_name The name of the training set.
     */
    ModelLoader(pcl::PolygonMesh mesh, std::string mesh_name);

    /*!
     * \brief Constructor for the ModelLoader class
     * \param mesh_name The name of the training set to load.
     */
    ModelLoader(std::string mesh_name);

    ~ModelLoader();

    /*!
     * \brief Returns the models of the selected training set.
     * \param load Set true if the models are not loaded.
     * \return The models of the selected training set.
     */
    std::vector<RayTraceCloud> getModels(bool load = false);

    /*!
     * \brief Creates a complete training set for the input polygon mesh.
     */
    void populateLoader();

    /*!
     * \brief Sets the polygon mesh used for training set creation.
     * \param mesh The polygon mesh that is to be used.
     */
    void setMesh(pcl::PolygonMesh mesh){
        ModelLoader::mesh = mesh;
    }

    /*!
     * \brief Sets the name of the training set.
     * \param mesh_name The name of the training set.
     */
    void setMeshName(std::string mesh_name){
        ModelLoader::mesh_name = mesh_name;
    }

    /*!
     * \brief Sets the wanted tesseltaion level for the viewpoint rendering.
     * \param tesselation_level Tesselation level, 1=42, 2=162 ...
     */
    void setTesselation_level(int tesselation_level){
        ModelLoader::tesselation_level = tesselation_level;
    }

    /*!
     * \brief Sets the viewpoint rendering resolution.
     * \param cloud_resolution The wanted resolution.
     */
    void setCloudResolution(int cloud_resolution){
        ModelLoader::cloud_resolution = cloud_resolution;
    }

    /*!
     * \brief Sets the output path of the training set creation process.
     * \param path The wanted output path.
     */
    void setPath(const std::string &path){
        ModelLoader::path = path;
    }

Q_SIGNALS:
    //Signals used to emit events from one class to another. All signals are connected in main_window.cpp

public Q_SLOTS:
    //Slots used to recieve events from one another class. All slots and signals are connected in main_window.cpp

private:
    /*!
     * \brief This function will generate the traces from a mesh and populate the ray_trace_clouds variable.
     */
    void generatePointClouds();

    /*!
     * \brief This function will load and populate the ray_trace_clouds variable from the given path.
     * \return False if the loading failed.
     */
    bool loadPointClouds();

    /*!
     * \brief This function will save all the infromation from the ray_trace_clouds variable to the given path.
     * \return False if the saving action failed.
     */
    bool savePointClouds();

    std::string path; //!< The path for saving and loading files.
    std::string mesh_name; //!< The name of the mesh. Used for saving and loading file names.
    std::vector<RayTraceCloud> ray_trace_clouds; //!< List of ray trace clouds.
    pcl::PolygonMesh mesh; //!< The mesh which is used for generation.
    int cloud_resolution; //!< The resolution camera when generating clouds.
    int tesselation_level; //!< The tesselation level of the sphere for the camera.
    PclFilters *filters;  //!< Object for calculating features of the raytraced models.
};

}

#endif // MODELLOADER_HPP
