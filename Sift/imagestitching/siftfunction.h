//
//  siftfunction.h
//  Sift
//
//  Created by JackieZhu on 14-3-25.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef Sift_siftfunction_h
#define Sift_siftfunction_h
#include "util.h"
//SIFT算法第一步：图像预处理
CvMat *ScaleInitImage(CvMat * im) ;                  //金字塔初始化

//SIFT算法第二步：建立高斯金字塔函数
ImageOctaves* BuildGaussianOctaves(CvMat * image) ;  //建立高斯金字塔

//SIFT算法第三步：特征点位置检测，最后确定特征点的位置
int DetectKeypoint(int numoctaves, ImageOctaves *GaussianPyr);
void DisplayKeypointLocation(IplImage* image, ImageOctaves *GaussianPyr);

//SIFT算法第四步：计算高斯图像的梯度方向和幅值，计算各个特征点的主方向
void ComputeGrad_DirecandMag(int numoctaves, ImageOctaves *GaussianPyr);

int FindClosestRotationBin (int binCount, float angle);  //进行方向直方图统计
void AverageWeakBins (double* bins, int binCount);       //对方向直方图滤波
//确定真正的主方向
bool InterpolateOrientation (double left, double middle,double right, double *degreeCorrection, double *peakValue);
//确定各个特征点处的主方向函数
void AssignTheMainOrientation(int numoctaves, ImageOctaves *GaussianPyr,ImageOctaves *mag_pyr,ImageOctaves *grad_pyr);
//显示主方向
void DisplayOrientation (IplImage* image, ImageOctaves *GaussianPyr);

//SIFT算法第五步：抽取各个特征点处的特征描述字
void ExtractFeatureDescriptors(int numoctaves, ImageOctaves *GaussianPyr);

//为了显示图象金字塔，而作的图像水平、垂直拼接
CvMat* MosaicHorizen( CvMat* im1, CvMat* im2 );
CvMat* MosaicVertical( CvMat* im1, CvMat* im2 );

bool InterpolateOrientation (double left, double middle,double right, double *degreeCorrection, double *peakValue);
void DisplayOrientation (IplImage* image, ImageOctaves *GaussianPyr);

//特征描述点，网格
#define GridSpacing 4

#endif
