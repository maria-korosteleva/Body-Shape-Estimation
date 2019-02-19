#include "pch.h"
#include "GeneralMesh.h"


GeneralMesh::GeneralMesh(const char* input_filename_c, const char* key_vertices_filename)
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

    this->mean_point_ = this->verts_.colwise().mean();
    
    if (key_vertices_filename != nullptr)
    {
        this->readKeyVertices_(key_vertices_filename);
    }
}


GeneralMesh::~GeneralMesh()
{
}

void GeneralMesh::readKeyVertices_(const char * filename)
{
    std::fstream inFile;
    inFile.open(filename, std::ios_base::in);
    int keys_n;
    inFile >> keys_n;
    // Sanity check
    if (keys_n <= 0)
    {
        throw std::exception("Number of key vertices should be a positive number!");
    }

    std::string key_name;
    int vertexId;
    for (int i = 0; i < keys_n; i++)
    {
        inFile >> key_name;
        inFile >> vertexId;
        this->key_vertices_.insert(DictEntry(key_name, vertexId));
    }

    inFile.close();
}
