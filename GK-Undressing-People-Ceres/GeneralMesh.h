#pragma once

// Fix compillation errors
#ifdef _MSC_VER
#pragma warning(disable:4996)
#endif

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readPLY.h>
#include <igl/per_vertex_normals.h>

#include <map>
#include <vector>

// for opengl-friendly conversion
#include <glm/glm.hpp>

using CoordsDictionary = std::map<std::string, Eigen::RowVectorXd>;
using CoordsDictEntry = std::pair<std::string, Eigen::RowVectorXd>;

class GeneralMesh
{
public:
    struct GLMVertex
    {
        glm::vec3 position;
        glm::vec3 normal;
    };

    GeneralMesh(const char* input_filename_c, const char* key_vertices_filename = nullptr);
    ~GeneralMesh();

    const std::string& getName() const           { return name_; };

    const Eigen::MatrixXi& getFaces() const      { return faces_; };
    // with vertex ids staring from zero:
    const std::vector<unsigned int>& getGLMFaces() const      { return gl_faces_; };

    const Eigen::MatrixXd& getVertices() const   { return verts_; };
    const Eigen::MatrixXd& getNormalizedVertices() const   { return verts_normalized_; };
    const std::vector<GLMVertex>& getGLNormalizedVertices() const   { return gl_vertices_normalized_; };

    const Eigen::VectorXd& getMeanPoint() const  { return mean_point_; };
    // the user need to check the dictionary for emptiness
    const CoordsDictionary& getKeyPoints() const  { return key_points_; }

private:
    void readFile_(const std::string& filename);
    void normalizeVertices_();
    void glFriendlyMesh_();
    void cutName_(const std::string& filename);
    void readKeyVertices_(const char * filename);
    bool checkFileExist_(const char * filename);

    std::string name_;

    Eigen::MatrixXd verts_;
    Eigen::MatrixXd verts_normalized_;
    Eigen::MatrixXd verts_normals_;
    std::vector<GLMVertex> gl_vertices_normalized_;

    Eigen::MatrixXi faces_;
    std::vector<unsigned int> gl_faces_;

    Eigen::VectorXd mean_point_;

    CoordsDictionary key_points_;
};

