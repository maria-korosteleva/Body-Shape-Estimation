#pragma once


#include <iostream>
#include <string>
#include <memory>
#include <ctime>
#include <fstream>
#include <assert.h>
#include <map>

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <openpose/headers.hpp>

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/signed_distance.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>

// my external modules
#include <GeneralMesh/GeneralMesh.h>
#include <Photographer/Photographer.h>
