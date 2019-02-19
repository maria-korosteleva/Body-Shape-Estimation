#pragma once

#include <map>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readPLY.h>

using Dictionary = std::map<std::string, int>;
using DictEntry = std::pair<std::string, int>;

class GeneralMesh
{
public:
    GeneralMesh(const char* input_filename_c, const char* key_vertices_filename = nullptr);
    ~GeneralMesh();

    const Eigen::MatrixXi& getFaces() const      { return this->faces_; };
    const Eigen::MatrixXd& getVertices() const   { return this->verts_; };
    const Eigen::VectorXd& getMeanPoint() const  { return this->mean_point_; };
    // the user need to check the dictionary for emptiness
    const Dictionary& getKeyVertices() const     { return this->key_vertices_; }

private:
    Eigen::MatrixXd verts_;
    Eigen::MatrixXi faces_;
    Eigen::VectorXd mean_point_;

    Dictionary key_vertices_;

    void readKeyVertices_(const char * filename);
};

