#include "GeneralMesh.h"


GeneralMesh::GeneralMesh(const char* input_filename_c, const char* key_vertices_filename)
{   
    // check for existance
    if (!checkFileExist_(input_filename_c))
    {
        throw std::exception("General Mesh: input file doesn't exist");
    }
    if (key_vertices_filename != nullptr && !checkFileExist_(key_vertices_filename))
    {
        throw std::exception("General Mesh: key vertices file doesn't exist");
    }

    std::string input_filename(input_filename_c);
    readFile_(input_filename);
    normalizeVertices_();
    glFriendlyMesh_();

    cutName_(input_filename);

    if (key_vertices_filename != nullptr)
    {
        readKeyVertices_(key_vertices_filename);
    }    

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
        igl::readOBJ(filename, verts_, faces_);
    }
    else if (extention == "ply")
    {
        igl::readPLY(filename, verts_, faces_);
    }
    else
    {
        throw std::exception("Unsupported type of input mesh. Supported types: .obj, .ply");
    }

    igl::per_vertex_normals(verts_, faces_, verts_normals_);
}

void GeneralMesh::normalizeVertices_()
{
    mean_point_ = verts_.colwise().mean();

    verts_normalized_ = verts_.rowwise() - mean_point_.transpose();

    // convert to meters heuristically
    // check for sm througth height (Y axis)
    if (verts_normalized_.col(1).maxCoeff() - verts_normalized_.col(1).minCoeff() > 100)
    {
        verts_normalized_ *= 0.01;
        std::cout << "Warning: Mesh is found to use sm/mm units. Scaled down by 0.01" << std::endl;
    }
    // check for mm/dm
    if (verts_normalized_.col(1).maxCoeff() - verts_normalized_.col(1).minCoeff() > 10)
    {
        verts_normalized_ *= 0.1;
        std::cout << "Warning: Mesh is found to use mm/dm units. Scaled down by 0.1" << std::endl;
    }
}

void GeneralMesh::glFriendlyMesh_()
{
    // vertices
    for (int i = 0; i < verts_normalized_.rows(); ++i)
    {
        GLMVertex gl_vert;
        gl_vert.position = glm::vec3(verts_normalized_(i, 0), verts_normalized_(i, 1), verts_normalized_(i, 2));
        gl_vert.normal = glm::vec3(verts_normals_(i, 0), verts_normals_(i, 1), verts_normals_(i, 2));

        gl_vertices_normalized_.push_back(gl_vert);
    }

    // faces
    for (int i = 0; i < faces_.rows(); ++i)
    {
        for (int j = 0; j < faces_.cols(); ++j)
        {
            // shift indices to start from zero
            gl_faces_.push_back(faces_(i, j));
        }
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

        key_points_.insert(CoordsDictEntry(key_name, verts_.row(vertexId)));
    }

    inFile.close();
}

bool GeneralMesh::checkFileExist_(const char * filename)
{
    std::ifstream infile(filename);
    return infile.good();
}
