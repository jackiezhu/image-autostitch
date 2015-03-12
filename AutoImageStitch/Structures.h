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
    int to;             //ptr to the image
    int from;
    double err;
    EdgeNode(cv::Mat H, int from, int to, double err): H(H), from(from), to(to),next(NULL), err(err){};
};

struct BFSNode {
    int nodeid;
    cv::Mat H;
    BFSNode(int nodeid, cv::Mat H):nodeid(nodeid), H(H) {}
};

struct Parent {
    int pre;
    cv::Mat H;
    Parent(int pre, cv::Mat H) : pre(pre), H(H) {}
    Parent():pre(-1),H(cv::Mat::eye(3, 3, CV_64F)) {}
};

#endif
