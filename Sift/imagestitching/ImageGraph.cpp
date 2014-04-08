//
//  ImageGraph.cpp
//  Sift
//
//  Created by JackieZhu on 14-4-8.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#include "ImageGraph.h"

//constructor of ImageGraph
//1.read the images from imgs
//2.judge whether every two images are adjecent
//3.add an edge between two images which are adjecent
ImageGraph::ImageGraph(std::vector<std::string> imgs) {
    int imgcnt = (int)imgs.size();
    this->vecImg.resize(imgcnt);
    for (int i=0; i<imgcnt; i++) {
        vecImg[i].img = cv::imread(imgs[i]);
    }
    
    head.resize(imgcnt);
    addEdges();
}


/*
 *destructure the Image Graph
 */
ImageGraph::~ImageGraph() {
    for(int i=0; i<head.size(); i++) {
        std::cout << i << "===============" << std::endl;
        deleteNode(head[i]);
    }
}

/*
 *return the size of a single image
 */
cv::Size ImageGraph::singleImageSize() {
    assert(vecImg.size() != 0);
    return cv::Size(vecImg[0].img.cols, vecImg[0].img.rows);
}

/*
 *this function is only used in destructor
 *it is used to delete the node in adjecent table
 */
void ImageGraph::deleteNode(EdgeNode *head) {
    if (head!=NULL) {
        deleteNode(head->next);
        delete head;
        head = NULL;
    }
}

/*
 *warp the src image to the given size of image.
 */
cv::Mat ImageGraph::getTransFormedImg(int imgID, cv::Mat H, cv::Size size) {
    cv::Mat ret;
    warpPerspective(vecImg[imgID].img, ret, H, size);
    return ret;
}


void ImageGraph::calTrasformMatrix(ImageNode img1, ImageNode img2, cv::Mat &one2two, cv::Mat &two2one) {
    cv::SiftFeatureDetector detector;
    cv::Mat img_1 = img1.img;
    cv::Mat img_2 = img2.img;
    
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    
    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );
    
    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SiftDescriptorExtractor extractor;
    
    cv::Mat descriptors_1, descriptors_2;
    
    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    cv::BruteForceMatcher< cv::L2<float> > matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
    
    double max_dist = 0;
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance > max_dist) {
            max_dist = matches[i].distance;
        }
    }
    
    std::vector<cv::DMatch> goodmatches;
    std::vector<cv::Point2f> goodpoints1;
    std::vector<cv::Point2f> goodpoints2;
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance <= 0.2 * max_dist) {
            goodmatches.push_back(matches[i]);
            goodpoints1.push_back(keypoints_1[matches[i].queryIdx].pt);
            goodpoints2.push_back(keypoints_2[matches[i].trainIdx].pt);
        }
    }
    two2one = findHomography(cv::Mat(goodpoints2), cv::Mat(goodpoints1), CV_RANSAC);
    one2two = findHomography(cv::Mat(goodpoints1), cv::Mat(goodpoints2), CV_RANSAC);
}


//TODO:how to judge two images are adjecent
void ImageGraph::addEdges() {
    int imgcnt = (int) vecImg.size();
    for (int i=1; i<imgcnt; i++) {
        cv::Mat one2two, two2one;
        calTrasformMatrix(vecImg[i-1], vecImg[i], one2two, two2one);
        EdgeNode *i2im1 = new EdgeNode(two2one, i-1);
        EdgeNode *im12i = new EdgeNode(one2two, i);
        i2im1->next = head[i];
        head[i] = i2im1;         //add an edge from i to i-1
        im12i->next = head[i-1];
        head[i-1] = im12i;       //add an edge from i-1 to i
    }
}

/* 
 *use broad first search to find the closest path from src to dest
 *and calculate the transform matrix.
 */
cv::Mat ImageGraph::findTranformMat(int dest, int src) {
    if (dest == src) {
        return cv::Mat::eye(3, 3, CV_64F);
    }
    BFSNode h(src, cv::Mat::eye(3, 3, CV_64F));
    std::queue<BFSNode> Q;
    std::vector<int> visit(head.size(), 0);
    Q.push(h);
    visit[src] = 1;
    while (!Q.empty()) {
        BFSNode f = Q.front();
        Q.pop();
        if (f.nodeid == dest) return f.H;         // if find, return
        for (EdgeNode *i=head[f.nodeid]; i!=NULL; i=i->next) {
            if (visit[i->nodeid]) continue;
            BFSNode p(i->nodeid, i->H * f.H);
            Q.push(p);
            visit[p.nodeid] = 1;
        }
    }
    return cv::Mat::zeros(3, 3, CV_64F);
}

/*
 *this function is used to display the image graph
 */
void ImageGraph::displayGraph() {
    for (int i=0; i<head.size(); i++) {
        std::cout << "node:" << i << ":  " << std::endl;
        EdgeNode *iter = head[i];
        while (iter != NULL) {
            std::cout << "      node: " <<iter->nodeid << ". transform matrix: " << iter->H << std::endl;
            iter = iter->next;
        }
    }
}