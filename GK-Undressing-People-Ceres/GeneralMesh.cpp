#include "pch.h"
#include "GeneralMesh.h"


GeneralMesh::GeneralMesh(const char* input_filename_c)
{   
    // load according to the extention
    std::string input_filename(input_filename_c);
    std::size_t pos = input_filename.find_last_of('.');
    std::string extention = input_filename.substr(pos + 1);
    if (extention == "obj")
    {
        igl::readOBJ(input_filename, this->verts_, this->faces_);
    }
    else if (extention == "ply")
    {
        igl::readPLY(input_filename, this->verts_, this->faces_);
    }
    else
    {
        throw std::exception("Unsupported type of input mesh. Supported types: .obj, .ply");
    }

    // calculate mean point
    this->mean_point_ = this->verts_.colwise().mean();
}


GeneralMesh::~GeneralMesh()
{
}
