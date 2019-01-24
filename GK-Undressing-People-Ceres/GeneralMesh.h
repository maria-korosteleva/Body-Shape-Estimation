#pragma once

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readPLY.h>

class GeneralMesh
{
public:
    GeneralMesh(const char*);
    ~GeneralMesh();

    Eigen::MatrixXi getFaces()      { return this->faces_; };
    Eigen::MatrixXd getVertices()   { return this->verts_; };
    Eigen::VectorXd getMeanPoint()  { return this->mean_point_; };

private:
    Eigen::MatrixXd verts_;
    Eigen::MatrixXi faces_;
    Eigen::VectorXd mean_point_;
};

