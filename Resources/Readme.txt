# SMPL statistical body model

All the data in this folders are part of the SMPL statistical body model. Refer to http://smpl.is.tuebingen.mpg.de/

## Prior information
The information on mean pose (_mean_pose.txt_) and covariance matrix (_covars.txt_) is obtained from the SMPL gmm pose prior published here:
http://smplify.is.tue.mpg.de/downloads

Only the first from 8 avalible mean poses and covariance matrices is used here. 
The _stiffness.txt_ contains the stifness matrix to be used as part of ceres::NormalPrior. It is computed as square root of inverse matrix of covars. 