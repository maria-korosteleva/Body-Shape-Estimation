#pragma once

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readPLY.h>

class GeneralMesh
{
public:
    GeneralMesh(const char*);
    ~GeneralMesh();

    const Eigen::MatrixXi& getFaces() const      { return this->faces_; };
    const Eigen::MatrixXd& getVertices() const   { return this->verts_; };
    const Eigen::VectorXd& getMeanPoint() const  { return this->mean_point_; };

private:
    Eigen::MatrixXd verts_;
    Eigen::MatrixXi faces_;
    Eigen::VectorXd mean_point_;
};

