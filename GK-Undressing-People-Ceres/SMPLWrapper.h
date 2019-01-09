#pragma once
/*
The class is a wrapper around SMPL model.
It calculates the SMPL model output based on pose and shape parameters

*/

class SMPLWrapper
{
public:
    SMPLWrapper(const char*);
    int getPoseSize() { return POSE_SIZE_; };
    int getShapeSize() { return SHAPE_SIZE_; };
    ~SMPLWrapper();

private:
    const int POSE_SIZE_ = 72;  // root rotation is included
    const int TRANSLATION_SIZE_ = 3;
    const int SHAPE_SIZE_ = 10;
};

