//
//  datastructure.h
//  Sift
//
//  Created by JackieZhu on 14-3-25.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef Sift_datastructure_h
#define Sift_datastructure_h
#include "definition.h"

//Data structure for a float image.
typedef struct ImageSt {        /*金字塔每一层*/
    
    float levelsigma;
    int levelsigmalength;
    float absolute_sigma;
    CvMat *Level;       //CvMat是OPENCV的矩阵类，其元素可以是图像的象素值
} ImageLevels;

typedef struct ImageSt1 {      /*金字塔每一阶梯*/
    int row, col;          //Dimensions of image.
    float subsample;
    ImageLevels *Octave;
} ImageOctaves;

ImageOctaves *DOGoctaves;
//DOG pyr，DOG算子计算简单，是尺度归一化的LoG算子的近似。

ImageOctaves *mag_thresh ;
ImageOctaves *mag_pyr ;
ImageOctaves *grad_pyr ;

//keypoint数据结构，Lists of keypoints are linked by the "next" field.
typedef struct KeypointSt
{
    float row, col; /* 反馈回原图像大小，特征点的位置 */
    float sx,sy;    /* 金字塔中特征点的位置*/
    int octave,level;/*金字塔中，特征点所在的阶梯、层次*/
    
    float scale, ori,mag; /*所在层的尺度sigma,主方向orientation (range [-PI,PI])，以及幅值*/
    float *descrip;       /*特征描述字指针：128维或32维等*/
    struct KeypointSt *next;/* Pointer to next keypoint in list. */
} *Keypoint;

//定义特征点具体变量
Keypoint keypoints=NULL;      //用于临时存储特征点的位置等
Keypoint keyDescriptors=NULL; //用于最后的确定特征点以及特征描述字

#endif
