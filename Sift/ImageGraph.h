//
//  ImageGraph.h
//  Sift
//
//  Created by JackieZhu on 14-4-8.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef __Sift__ImageGraph__
#define __Sift__ImageGraph__

#include <stdio.h>
#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"     //included for SurfFeatureDector
#include "opencv2/legacy/legacy.hpp"       //included for BruteForceMatcher
#include "opencv2/video/tracking.hpp"      //estimate affine matrix with ransac
#include <queue>

#include "Structures.h"


class ImageGraph {
public:
    ImageGraph(std::vector<std::string> imgs);
    ~ImageGraph();
    void displayGraph();
    cv::Mat findTranformMat(int dest, int src);    //find the transform matrix from src to dest
    cv::Size singleImageSize();                    //get the size of a single image.
    cv::Mat getTransFormedImg(int imgID, cv::Mat H, cv::Size size); //warp the src image to the given size of image.
    
    //Affine transform
    cv::Mat getAffineTransFormedImg(int imgID, cv::Mat H, cv::Size size);    
    
private:
    std::vector<ImageNode> vecImg; // used to store the node
    std::vector<EdgeNode *> head;  //used to store the edge
    void addEdges();
    void calHomographyMatrix(ImageNode img1, ImageNode img2, cv::Mat &one2two, cv::Mat &two2one);
    void deleteNode(EdgeNode *head); //only used in destructor to delete the node.
    void getSIFTKeyPointsDescriptor(ImageNode &img);  //compute keypoints and sift descriptor for one image
    
    //Affine transform
    void addAffineEdges();      //the relation between two images is Affine transform.
    void calAffineMatrix(ImageNode img1, ImageNode img2, cv::Mat &one2two, cv::Mat &two2one);

};


#endif /* defined(__Sift__ImageGraph__) */
