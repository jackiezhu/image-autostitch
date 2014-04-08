//
//  function.h
//  Sift
//
//  Created by JackieZhu on 14-3-25.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#ifndef Sift_function_h
#define Sift_function_h
#include "util.h"

CvMat * halfSizeImage(CvMat * im);     //缩小图像：下采样
CvMat * doubleSizeImage(CvMat * im);   //扩大图像：最近临方法
CvMat * doubleSizeImage2(CvMat * im);  //扩大图像：线性插值
float getPixelBI(CvMat * im, float col, float row);//双线性插值函数
void normalizeVec(float* vec, int dim);//向量归一化
CvMat* GaussianKernel2D(float sigma);  //得到2维高斯核
void normalizeMat(CvMat* mat) ;        //矩阵归一化
float* GaussianKernel1D(float sigma, int dim) ; //得到1维高斯核

//在具体像素处宽度方向进行高斯卷积
float ConvolveLocWidth(float* kernel, int dim, CvMat * src, int x, int y) ;
//在整个图像宽度方向进行1D高斯卷积
void Convolve1DWidth(float* kern, int dim, CvMat * src, CvMat * dst) ;
//在具体像素处高度方向进行高斯卷积
float ConvolveLocHeight(float* kernel, int dim, CvMat * src, int x, int y) ;
//在整个图像高度方向进行1D高斯卷积
void Convolve1DHeight(float* kern, int dim, CvMat * src, CvMat * dst);
//用高斯函数模糊图像
int BlurImage(CvMat * src, CvMat * dst, float sigma) ;
void normalizeMat(CvMat* mat);
float GetVecNorm( float* vec, int dim );

#endif
