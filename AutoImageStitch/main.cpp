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
#include "ExpUtil.h"


using namespace cv;
using namespace std;

void getError(cv::Mat imga, cv::Mat imgb, int& totPixl, double &sumerror);

/** @function main */
int main( int argc, char** argv )
{

    string dir = "/Users/JackieZhu/Documents/work/research/citytest/";
    const int IMGCNT = 56;
    vector<string> imgs;
    for (int i=0; i<IMGCNT; i++) {
        string nu = itos(i);
        string img = dir + nu + ".JPG";
        //cout << img << endl;
        imgs.push_back(img);
    }
    double starttime = static_cast<double>(cv::getTickCount());
    cout << imgs.size() << endl;
    ImageGraph IG(imgs);
    IG.displayGraph();
    //return 0;
    double tt = static_cast<double>(cv::getTickCount());
    cout << "construct the graph: " << (static_cast<double>(cv::getTickCount()) - starttime)/cv::getTickFrequency() << "s" << endl;

    //the final image's max size should be (images * rows, images * cols)
    Size sz = IG.singleImageSize();
    Size finalSize = Size(sz.width * 7, sz.height * 9);
    Mat final;
    final.create(finalSize.height, finalSize.width, CV_8UC3);
    double ttt = static_cast<double>(cv::getTickCount());
    cout << endl << endl;
    //vector<Mat> trans = IG.findDijkTransformMat();
   // namedWindow("aa");
    //imshow("aa", final);
    int totPixel = 0;
    double sumerr = 0;
    for (int i=0; i<IMGCNT; i++) {
        cout << "processing image " << i << "..." << endl;
        
        Mat H = IG.findTranformMat(3, i);
        Mat ig = IG.getTransFormedImg(i, H, finalSize);

        string name = "res"+itos(i);
        imwrite(dir + name +".jpg", ig);
        getError(final, ig, totPixel, sumerr);
        registerImage(final, ig);
        imwrite(dir + itos(i) +"zigzag.jpg", final);
    }
    cout << "total pixel : " << totPixel << endl;
    cout << "sum error : " << fixed << sumerr << endl;
    cout << "average error : " << fixed << sumerr / totPixel << endl;
    //cout << "register the graph: " << (static_cast<double>(cv::getTickCount()) - tt)/cv::getTickFrequency() << "s" << endl;
    tt = static_cast<double>(cv::getTickCount());
    imwrite(dir+"bgparamazigzag.jpg", final);
    
    medianBlur(final, final, 5);
   // cout << "blending the output image: " << (static_cast<double>(cv::getTickCount()) - tt)/cv::getTickFrequency() << "s" << endl;
   // cout << "total time: " << (static_cast<double>(cv::getTickCount()) - starttime)/cv::getTickFrequency() << "s" << endl;

    imwrite(dir+"paramazigzag.jpg", final);
   // cout  << "stitching finished!" << endl;
    return 0;
}

/** @function readme */
void readme()
{ std::cout << " Usage: ./SIFT <dir> <img1> <img2>... dir must be ended with /" << std::endl; }

void getError(cv::Mat imga, cv::Mat imgb, int& totPixl, double &sumerror) {
    Mat_<Vec3b>::iterator destbegin = imga.begin<Vec3b>();
    Mat_<Vec3b>::iterator srcbegin = imgb.begin<Vec3b>();
    Mat_<Vec3b>::iterator destend = imga.end<Vec3b>();
    Mat_<Vec3b>::iterator srcend = imgb.end<Vec3b>();
    double ret = 0;
    int tot = 0;
    while (destbegin != destend && srcbegin != srcend) {
        if (isBlack((*destbegin)) || isBlack(*srcbegin) ) {
            destbegin ++;
            srcbegin ++;
            continue;
        }
        for (int i=0; i<3; i++) {
            double err = (*destbegin)[i]*1.0 - (*srcbegin)[i]*1.0;
            ret += fabs(err * err);
            //cout <<err << endl;
        }
        tot += 3;
        destbegin ++;
        srcbegin ++;
    }
    
}



