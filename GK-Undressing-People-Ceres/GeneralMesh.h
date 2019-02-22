#pragma once

#include <map>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readPLY.h>

using CoordsDictionary = std::map<std::string, Eigen::RowVectorXd>;
using CoordsDictEntry = std::pair<std::string, Eigen::RowVectorXd>;

class GeneralMesh
{
public:
    GeneralMesh(const char* input_filename_c, const char* key_vertices_filename = nullptr);
    ~GeneralMesh();

    const Eigen::MatrixXi& getFaces() const      { return this->faces_; };
    const Eigen::MatrixXd& getVertices() const   { return this->verts_; };
    const Eigen::VectorXd& getMeanPoint() const  { return this->mean_point_; };
    // the user need to check the dictionary for emptiness
    const CoordsDictionary& getKeyPoints() const     { return this->key_points_; }

private:
    Eigen::MatrixXd verts_;
    Eigen::MatrixXi faces_;
    Eigen::VectorXd mean_point_;

    CoordsDictionary key_points_;

    void readKeyVertices_(const char * filename);
    bool checkFileExist_(const char * filename);
};

