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

    std::string input_filename(input_filename_c);
    this->readFile_(input_filename);
    this->cutName_(input_filename);
    if (key_vertices_filename != nullptr)
    {
        this->readKeyVertices_(key_vertices_filename);
    }    

    this->mean_point_ = this->verts_.colwise().mean();
}


GeneralMesh::~GeneralMesh()
{
}


void GeneralMesh::readFile_(const std::string & filename)
{
    std::size_t point_pos = filename.find_last_of('.');
    std::string extention = filename.substr(point_pos + 1);
    if (extention == "obj")
    {
        igl::readOBJ(filename, this->verts_, this->faces_);
    }
    else if (extention == "ply")
    {
        igl::readPLY(filename, this->verts_, this->faces_);
    }
    else
    {
        throw std::exception("Unsupported type of input mesh. Supported types: .obj, .ply");
    }
}


void GeneralMesh::cutName_(const std::string & filename)
{
    std::size_t point_pos = filename.find_last_of('.');
    std::size_t slash_pos = filename.find_last_of('/');
    std::string path = filename.substr(0, slash_pos);
    std::string data_group = path.substr(path.find_last_of('/') + 1);
    std::string object_name = filename.substr(slash_pos + 1, (point_pos - slash_pos - 1));

    this->name_ = data_group + "-" + object_name;
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
