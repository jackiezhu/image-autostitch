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
        std::cout << imgs[i] << std::endl;
        vecImg[i].img = cv::imread(imgs[i]);
        getSIFTKeyPointsDescriptor(vecImg[i]);
        // cv::namedWindow("reeeee");
       // cv::imshow("reeeee", vecImg[i].img);
        //cv::waitKey(0);
        //cv::destroyWindow("reeeee");
    }
    head.resize(imgcnt);
    for (int i=0; i<head.size(); i++) {
        head[i] = NULL;
    }
    //addAffineEdges();
    addEdges();
}
void ImageGraph::getSIFTKeyPointsDescriptor(ImageNode &img) {
    cv::SiftFeatureDetector detector;
    cv::Mat imgM = img.img;
    detector.detect( imgM, img.keypoints );
    cv::SiftDescriptorExtractor extractor;
    extractor.compute(imgM, img.keypoints, img.descriptors);
}

/*
 *destructure the Image Graph
 */
ImageGraph::~ImageGraph() {
    for(int i=0; i<head.size(); i++) {
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


void ImageGraph::calHomographyMatrix(ImageNode img1, ImageNode img2, cv::Mat &one2two, cv::Mat &two2one) {

    //Matching descriptor vectors with a brute force matcher
    cv::BruteForceMatcher< cv::L2<float> > matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( img1.descriptors, img2.descriptors, matches );
    
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
        if (matches[i].distance <=  0.8*max_dist) {
            goodmatches.push_back(matches[i]);
            goodpoints1.push_back(img1.keypoints[matches[i].queryIdx].pt);
            goodpoints2.push_back(img2.keypoints[matches[i].trainIdx].pt);
        }
    }
    std::cout << goodpoints1.size() << "   aaaaa   " << goodpoints2.size() << std::endl;
    two2one = findHomography(cv::Mat(goodpoints2), cv::Mat(goodpoints1), CV_RANSAC);
    one2two = findHomography(cv::Mat(goodpoints1), cv::Mat(goodpoints2), CV_RANSAC);
}


//TODO:how to judge two images are adjecent
void ImageGraph::addEdges() {
    int imgcnt = (int) vecImg.size();
    for(int p=0; p<8; p++) {
        std::cout << "row " << p << std::endl;
        for (int i=1; i<7; i++) {

            std::cout << "  img " << i << std::endl;
            cv::Mat one2two, two2one;
            calHomographyMatrix(vecImg[p*7+i-1], vecImg[p*7+i], one2two, two2one);
            
            EdgeNode *i2im1 = new EdgeNode(two2one, p*7+i-1);
            EdgeNode *im12i = new EdgeNode(one2two, p*7+i);
            i2im1->next = head[p*7+i];
            head[p*7+i] = i2im1;         //add an edge from i to i-1
            im12i->next = head[p*7+i-1];
            head[p*7+i-1] = im12i;       //add an edge from i-1 to i
        }
    }
    for (int i=10; i<imgcnt; i+=7) {
        //if(i == 28) continue;
        std::cout << "  col " << i << std::endl;
        cv::Mat one2two, two2one;
        calHomographyMatrix(vecImg[i-7], vecImg[i], one2two, two2one);
        
        EdgeNode *i2im1 = new EdgeNode(two2one, i-7);
        EdgeNode *im12i = new EdgeNode(one2two, i);
        i2im1->next = head[i];
        head[i] = i2im1;         //add an edge from i to i-1
        im12i->next = head[i-7];
        head[i-7] = im12i;       //add an edge from i-1 to i
    }
 //   cv::Mat one2two, two2one;
  //  calHomographyMatrix(vecImg[22], vecImg[29], one2two, two2one);
    
  ///  EdgeNode *i2im1 = new EdgeNode(two2one, 22);
  //  EdgeNode *im12i = new EdgeNode(one2two, 29);
  //  i2im1->next = head[29];
  //  head[29] = i2im1;         //add an edge from i to i-1
  //  im12i->next = head[22];
  //  head[22] = im12i;       //add an edge from i-1 to i
    
}

/*
 *use broad first search to find the closest path from src to dest
 *and calculate the transform matrix.
 */
cv::Mat ImageGraph::findTranformMat(int dest, int src) {
    cv::Mat virMat = (cv::Mat_<double>(3,3) << 1, 0, vecImg[dest].img.cols*7/2, 0, 1, vecImg[dest].img.rows*7/2, 0, 0, 1);
    if (dest == src) {
        return virMat;
    }
    BFSNode h(src, cv::Mat::eye(3, 3, CV_64F));
    std::queue<BFSNode> Q;
    std::vector<int> visit(head.size(), 0);
    Q.push(h);
    visit[src] = 1;
    while (!Q.empty()) {
        BFSNode f = Q.front();
        Q.pop();
        std::cout << f.nodeid << std::endl;
        if (f.nodeid == dest) return virMat * f.H;         // if find, return
        for (EdgeNode *i=head[f.nodeid]; i!=NULL; i=i->next) {
            if (visit[i->nodeid]) continue;
            BFSNode p(i->nodeid, i->H * f.H);
            Q.push(p);
            visit[p.nodeid] = 1;
        }
    }
    return virMat;
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



/****************************************************
 *Affine transform functions are blow
 *The transform matrix between two images is 3*3
 *in order to adapt to continuious transforms
 ***************************************************/

/*
 *calculate Affine Matrixes that transform image1 to 
 *image2 and image2 to image1.
 */
void ImageGraph::calAffineMatrix(ImageNode img1, ImageNode img2, cv::Mat &one2two, cv::Mat &two2one) {
    cv::BruteForceMatcher< cv::L2<float> > matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( img1.descriptors, img2.descriptors, matches );
    
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
        if (matches[i].distance <= max_dist) {
            goodmatches.push_back(matches[i]);
            goodpoints1.push_back(img1.keypoints[matches[i].queryIdx].pt);
            goodpoints2.push_back(img2.keypoints[matches[i].trainIdx].pt);
        }
    }
  //  cv::Mat outp;
  //  cv::drawMatches(img1.img, img1.keypoints, img2.img, img2.keypoints, goodmatches, outp);
  ////  cv::namedWindow("aaa");
//cv::imshow("aaa", outp);
  //  cvWaitKey(0);
    cv::Mat t2o = cv::estimateRigidTransform(goodpoints1, goodpoints2, true);
    std::cout << t2o.rows << std::endl;
    assert(t2o.rows == 3);
    one2two = cv::Mat::zeros(3, 3, CV_64F);
    std::cout << goodpoints1.size() << " " << goodpoints2.size() << std::endl;
    std::cout << "t2o:  " << std::endl;
    std::cout << t2o << std::endl;
    cv::Mat l = (cv::Mat_<double>(1,3)<< 0,0,1);
    t2o.row(0).copyTo(one2two.row(0));
    t2o.row(1).copyTo(one2two.row(1));
    l.copyTo(one2two.row(2));
    
    cv::Mat o2t = cv::estimateRigidTransform(goodpoints2, goodpoints1, true);
    assert(o2t.rows == 3);
    two2one = cv::Mat::zeros(3, 3, CV_64F);
    o2t.row(0).copyTo(two2one.row(0));
    o2t.row(1).copyTo(two2one.row(1));
    l.copyTo(two2one.row(2));
    std::cout << "two2one:" << std::endl;
    std::cout << two2one << std::endl;
    std::cout << "one2two" << std::endl;
    std::cout << one2two << std::endl;
    //std::cout << "asdf  " << two2one << std::endl;
 //   std::cout << "row 0: " << t2o.row(0) << std::endl;
 //   std::cout << "row 1: " << t2o.row(1) << std::endl;
  //  std::cout << "row 2: " << l << std::endl;
  //  std::cout << "xxx " << t2o << std::endl;
  //  std::cout << "yyy " << one2two << std::endl;

}




/*
 *add an edge between two image nodes.
 *the transform matrix is Affine Matrix.
 */
void ImageGraph::addAffineEdges() {
    int imgcnt = (int) vecImg.size();
    for (int i=1; i<imgcnt; i++) {
        cv::Mat one2two = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat two2one = cv::Mat::zeros(3, 3, CV_64F);;
        calAffineMatrix(vecImg[i-1], vecImg[i], one2two, two2one);
        EdgeNode *i2im1 = new EdgeNode(two2one, i-1);
        EdgeNode *im12i = new EdgeNode(one2two, i);
        i2im1->next = head[i];
        head[i] = i2im1;         //add an edge from i to i-1
        im12i->next = head[i-1];
        head[i-1] = im12i;       //add an edge from i-1 to i
    }
}


/*
 *warp the src image using AffineTransForm to the given size of image.
 */
cv::Mat ImageGraph::getAffineTransFormedImg(int imgID, cv::Mat H, cv::Size size) {
    cv::Mat affineM = cv::Mat::zeros(2, 3, CV_64F);
    H.row(0).copyTo(affineM.row(0));
    H.row(1).copyTo(affineM.row(1));
    std::cout << affineM << std::endl;
    cv::Mat warp_dist;
    warp_dist = cv::Mat::zeros( size, vecImg[imgID].img.type() );
    cv::warpAffine( vecImg[imgID].img, warp_dist, affineM, warp_dist.size() );
    return warp_dist;
}