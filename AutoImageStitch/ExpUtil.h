//
//  ExpUtil.h
//  Sift
//
//  Created by JackieZhu on 14-5-23.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef __Sift__ExpUtil__
#define __Sift__ExpUtil__

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"     //included for SurfFeatureDector
#include "opencv2/legacy/legacy.hpp"       //included for BruteForceMatcher
#include "opencv2/video/tracking.hpp"      //estimate affine matrix with ransac

#include "ImageGraph.h"
#define MAXN 200
#define inf 1000000000
typedef double elem_t;
//单源最短路径,dijkstra算法+二分堆,邻接表形式,复杂度O(mlogm)
//求出源s到所有点的最短路经,传入图的大小n和邻接表list
//返回到各点最短距离min[]和路径pre[],pre[i]记录s到i路径上i的父结点,pre[s]=-1
//可更改路权类型,但必须非负!
#define _cp(a,b) ((a).d<(b).d)
struct heap_t{elem_t d;int v;};
struct heap{
    heap_t h[MAXN*MAXN];
    int n,p,c;
    void init(){n=0;}
    void ins(heap_t e){
        for (p=++n;p>1&&_cp(e,h[p>>1]);h[p]=h[p>>1],p>>=1);
        h[p]=e;
    }
    int del(heap_t& e){
        if (!n) return 0;
        for (e=h[p=1],c=2;c<n&&_cp(h[c+=(c<n-1&&_cp(h[c+1],h[c]))],h[n]);h[p]=h[c],p=c,c<<=1);
        h[p]=h[n--];return 1;
    }
};

void readme();
std::string itos(int i);    //convert int to string
void registerImage(cv::Mat &dest, cv::Mat &src); //merge image dest to src
uchar getColor(uchar dest, uchar src);
bool isBlack(cv::Vec3b img);
double getError(cv::Mat imga, cv::Mat imgb);
#endif /* defined(__Sift__ExpUtil__) */
