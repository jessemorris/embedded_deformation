
#include "utils/IO_libIGL/readOBJ.h"
#include "utils/IO/readPLY.h"
#include "utils/IO/writePLY.h"
#include "utils/IO/readCSV.h"



#include "libGraphCpp/readGraphOBJ.hpp"
#include "libGraphCpp/polyscopeWrapper.hpp"

#include "embedded_deformation/embedDeform.hpp"
#include "utils/visualization/plotMesh.h"
#include "utils/visualization/plotCloud.h"
#include "embedded_deformation/options.hpp"

#include <yaml-cpp/yaml.h>
#include "polyscope/polyscope.h"

#include <open3d/Open3D.h>
#include <memory>


void constructGraphMatrices(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const open3d::geometry::TriangleMesh& mesh) {
    V.resize(3, mesh.vertices_.size());
    for(size_t i = 0; i < mesh.vertices_.size(); i++) {
        const Eigen::Vector3d& vertices = mesh.vertices_[i];
        V(0, i) = vertices[0];
        V(1, i) = vertices[1];
        V(2, i) = vertices[2];
    }
    // for(const Eigen::Vector3d& vertices : mesh.vertices_) {
    //     V << vertices;
    // }

    F.resize(3, mesh.triangles_.size());
    for(size_t i = 0; i < mesh.triangles_.size(); i++) {
        const Eigen::Vector3i& triangle = mesh.triangles_[i];
        F(0, i) = triangle[0];
        F(1, i) = triangle[1];
        F(2, i) = triangle[2];
    }
    

    std::cout << "Vertices shape " << "ROWS " << V.rows() << " COLS " << V.cols() << std::endl;
    std::cout << "Faces shape " << "ROWS " << F.rows() << " COLS " << F.cols() << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Starting Opend3D sample" << std::endl;

    options opts;
    opts.loadYAML("../open3d_config.yaml");
    std::cout << "Progress: yaml loaded\n";

    polyscope::init();

    open3d::geometry::PointCloud pointcloud;
    open3d::io::ReadPointCloudOption cloud_options;
    open3d::io::ReadPointCloudFromPLY("../data/fragment.ply", pointcloud,cloud_options);

    //these will be 3 x N
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    EmbeddedDeformation* non_rigid_deformation;

    std::shared_ptr<open3d::geometry::PointCloud> downsampled_cloud =
        pointcloud.UniformDownSample(40);

    std::shared_ptr<open3d::geometry::TriangleMesh> mesh =
        open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*downsampled_cloud, 0.1);

    std::cout << "Mesh triangles " << mesh->triangles_.size() << std::endl;
    std::cout << "Mesh adjacency list " << mesh->adjacency_list_.size() << std::endl;
    std::cout << "Mesh vertices " << mesh->vertices_.size() << std::endl;
    constructGraphMatrices(V, F, *mesh);


    std::cout << "Vertices shape " << "ROWS " << V.rows() << " COLS " << V.cols() << std::endl;
    std::cout << "Faces shape " << "ROWS " << F.rows() << " COLS " << F.cols() << std::endl;

    if (opts.visualization)
        if (F.rows() != 0)
            plot_mesh(V,F);
        else
            plot_cloud(V);

    non_rigid_deformation = new EmbeddedDeformation(V, opts);
    // open3d::visualization::Visualizer visualizer;
    // std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(
    //         new open3d::geometry::PointCloud);
    // *pointcloud_ptr = pointcloud;
    // pointcloud_ptr->NormalizeNormals();

    // visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
    // visualizer.AddGeometry(mesh);
    // visualizer.Run();
    // visualizer.DestroyVisualizerWindow();






}