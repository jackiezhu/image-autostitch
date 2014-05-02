//
//  main.cpp
//  Sift
//
//  Created by JackieZhu on 14-3-25.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"     //included for SurfFeatureDector
#include "opencv2/legacy/legacy.hpp"       //included for BruteForceMatcher
#include "opencv2/video/tracking.hpp"      //estimate affine matrix with ransac

#include "ImageGraph.h"

using namespace cv;
using namespace std;
void readme();
string itos(int i);    //convert int to string
void registerImage(Mat &dest, Mat &src);
uchar getColor(uchar dest, uchar src);

/** @function main */
int main( int argc, char** argv )
{
    string dir = "/Users/JackieZhu/Documents/work/research/test/";
    const int IMGCNT = 56;
    vector<string> imgs;
    for (int i=0; i<IMGCNT; i++) {
        string nu = itos(i);
        string img = dir + nu + ".tif";
        //cout << img << endl;
        imgs.push_back(img);
    }
    cout << imgs.size() << endl;
    ImageGraph IG(imgs);
    IG.displayGraph();
   // system("pause");
    //the final image's max size should be (images * rows, images * cols)
    Size sz = IG.singleImageSize();
    Size finalSize = Size(sz.width * 7, sz.height * 7);
    Mat final;
    final.create(finalSize.height, finalSize.width, CV_8UC3);
   // namedWindow("aa");
    //imshow("aa", final);
    for (int i=0; i<IMGCNT; i++) {
        cout << i << endl;
        //Mat H = IG.findTranformMat(0, i);
        Mat H = IG.findTranformMat(49, i);
        Mat ig = IG.getTransFormedImg(i, H, finalSize);
        //Mat ig = IG.getAffineTransFormedImg(i, H, finalSize);
        string name = "res"+itos(i);
        //imwrite(dir + name +".jpg", ig);
        registerImage(final, ig);
    }
    
   // namedWindow("stitched");
    imwrite(dir+"parama2.jpg", final);
    //imshow("stitched", final);
    cv::waitKey(0);
    /*
    //cv::initModule_nonfree();
    Mat img_1 = imread( "/Users/JackieZhu/Documents/work/research/256主室南壁原始数据150DPI/G9PQ0282.tif");
    Mat img_2 = imread( "/Users/JackieZhu/Documents/work/research/256主室南壁原始数据150DPI/G9PQ0283.tif");
    cout << img_1.channels() << endl;
    if( !img_1.data || !img_2.data )
    { return -1; }

    SiftFeatureDetector detector;
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );
    
    //-- Step 2: Calculate descriptors (feature vectors)
    SiftDescriptorExtractor extractor;
    
    Mat descriptors_1, descriptors_2;
    
    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );
    cout << descriptors_1.cols << endl;
    //-- Step 3: Matching descriptor vectors with a brute force matcher
    BruteForceMatcher< L2<float> > matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
    
    double max_dist = 0;
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance > max_dist) {
            max_dist = matches[i].distance;
        }
    }
    
    vector<DMatch> goodmatches;
    vector<Point2f> goodpoints1;
    vector<Point2f> goodpoints2;
    for (int i=0; i<matches.size(); i++) {
        if (matches[i].distance <= 0.6 * max_dist) {
            goodmatches.push_back(matches[i]);
            goodpoints1.push_back(keypoints_1[matches[i].queryIdx].pt);
            goodpoints2.push_back(keypoints_2[matches[i].trainIdx].pt);
        }
    }
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_matches );
    
    cv::Mat warp_mat = cv::estimateRigidTransform(goodpoints2, goodpoints1, true);
    Mat warp_dist;
    warp_dist = Mat::zeros( img_1.rows, img_1.cols*2, img_1.type() );
    warpAffine( img_2, warp_dist, warp_mat, warp_dist.size() );

    namedWindow("result1");
    imshow("result1", warp_dist);
    Mat ROI(warp_dist, Rect(0, 0, img_1.cols, img_1.rows));
    img_1.copyTo(ROI);
    namedWindow("result");
    imshow("result", warp_dist);
    imwrite("/Users/JackieZhu/Documents/work/research/Affine.jpg", warp_dist);
    waitKey(0);
    */
    return 0;
}

/** @function readme */
void readme()
{ std::cout << " Usage: ./SIFT <dir> <img1> <img2>... dir must be ended with /" << std::endl; }

string itos(int i)
{ string ret; if(!i) return "0"; for (; i; i/=10) { ret = (char)(i%10 + '0')+ret; } return ret;}

uchar getColor(uchar dest, uchar src) {
    if(dest == 0) return src;
    if(src == 0) return dest;
    return (src>>1)+ (dest>>1);
}

void registerImage(Mat &dest, Mat &src) {
    Mat_<Vec3b>::iterator destbegin = dest.begin<Vec3b>();
    Mat_<Vec3b>::iterator srcbegin = src.begin<Vec3b>();
    Mat_<Vec3b>::iterator destend = dest.end<Vec3b>();
    Mat_<Vec3b>::iterator srcend = dest.end<Vec3b>();
    
    while (destbegin != destend && srcbegin != srcend) {
        for (int i=0; i<3; i++) {
            (*destbegin)[i] = getColor((*destbegin)[i], (*srcbegin)[i]);
        }
        destbegin ++;
        srcbegin ++;
    }
    
}