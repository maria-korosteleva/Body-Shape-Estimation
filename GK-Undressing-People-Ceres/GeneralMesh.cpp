#include "pch.h"
#include "GeneralMesh.h"


GeneralMesh::GeneralMesh(const char* input_filename_c, const char* key_vertices_filename)
{   
    // check for existance
    if (!this->checkFileExist_(input_filename_c))
    {
        throw std::exception("General Mesh: input file doesn't exist");
    }
    if (key_vertices_filename != nullptr && !this->checkFileExist_(key_vertices_filename))
    {
        throw std::exception("General Mesh: key vertices file doesn't exist");
    }

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

        this->key_points_.insert(CoordsDictEntry(key_name, this->verts_.row(vertexId)));
    }

    inFile.close();
}

bool GeneralMesh::checkFileExist_(const char * filename)
{
    std::ifstream infile(filename);
    return infile.good();
}
