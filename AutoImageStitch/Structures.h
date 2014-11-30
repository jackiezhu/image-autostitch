//
//  Structures.h
//  Sift
//
//  Created by JackieZhu on 14-4-8.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef Sift_Structures_h
#define Sift_Structures_h

#include "opencv2/core/core.hpp"

struct ImageNode {
    cv::Mat img;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct EdgeNode {
    cv::Mat H;             //the transform matrix
    struct EdgeNode *next; //ptr to the next node of the graph
    int nodeid;            //ptr to the image
    EdgeNode(cv::Mat H, int nodeid): H(H), nodeid(nodeid),next(NULL) {};
};

struct BFSNode {
    int nodeid;
    cv::Mat H;
    BFSNode(int nodeid, cv::Mat H):nodeid(nodeid), H(H) {}
};

#endif
