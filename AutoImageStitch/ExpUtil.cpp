//
//  ExpUtil.cpp
//  Sift
//
//  Created by JackieZhu on 14-5-23.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#include "ExpUtil.h"

using namespace cv;
using namespace std;




//main函数中的工具函数

string itos(int i)
{ string ret; if(!i) return "0"; for (; i; i/=10) { ret = (char)(i%10 + '0')+ret; } return ret;}

uchar getColor(uchar dest, uchar src) {
    if(dest == 0) return src;
    if(src == 0) return dest;
    return src;
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

bool isBlack(Vec3b img) {
    if (img[0] == 0 && img[1] == 0 && img[2] == 0) {
        return true;
    }
    return false;
}

double getError(cv::Mat imga, cv::Mat imgb) {
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
        }
        tot ++;
        destbegin ++;
        srcbegin ++;
    }
    return ret / tot;
}