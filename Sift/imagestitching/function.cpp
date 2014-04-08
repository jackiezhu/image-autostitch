//
//  function.cpp
//  Sift
//
//  Created by JackieZhu on 14-3-25.
//  Copyright (c) 2014年 JackieZhu. All rights reserved.
//

#include "function.h"
#include <cmath>

//下采样原来的图像，返回缩小2倍尺寸的图像
CvMat * halfSizeImage(CvMat * im)
{
    unsigned int i,j;
    int w = im->cols/2;
    int h = im->rows/2;
    CvMat *imnew = cvCreateMat(h, w, CV_32FC1);
    
#define Im(ROW,COL) ((float *)(im->data.fl + im->step/sizeof(float) *(ROW)))[(COL)]
#define Imnew(ROW,COL) ((float *)(imnew->data.fl + imnew->step/sizeof(float) *(ROW)))[(COL)]
    for ( j = 0; j < h; j++)
        for ( i = 0; i < w; i++)
            Imnew(j,i)=Im(j*2, i*2);
    return imnew;
}

//上采样原来的图像，返回放大2倍尺寸的图像
CvMat * doubleSizeImage(CvMat * im)
{
    unsigned int i,j;
    int w = im->cols*2;
    int h = im->rows*2;
    CvMat *imnew = cvCreateMat(h, w, CV_32FC1);
    
#define Im(ROW,COL) ((float *)(im->data.fl + im->step/sizeof(float) *(ROW)))[(COL)]
#define Imnew(ROW,COL) ((float *)(imnew->data.fl + imnew->step/sizeof(float) *(ROW)))[(COL)]
    
    for ( j = 0; j < h; j++)
        for ( i = 0; i < w; i++)
            Imnew(j,i)=Im(j/2, i/2);
    
    return imnew;
}

//上采样原来的图像，返回放大2倍尺寸的线性插值图像
CvMat * doubleSizeImage2(CvMat * im)
{
    unsigned int i,j;
    int w = im->cols*2;
    int h = im->rows*2;
    CvMat *imnew = cvCreateMat(h, w, CV_32FC1);
    
#define Im(ROW,COL) ((float *)(im->data.fl + im->step/sizeof(float) *(ROW)))[(COL)]
#define Imnew(ROW,COL) ((float *)(imnew->data.fl + imnew->step/sizeof(float) *(ROW)))[(COL)]
    
    // fill every pixel so we don't have to worry about skipping pixels later
    for ( j = 0; j < h; j++)
    {
        for ( i = 0; i < w; i++)
        {
            Imnew(j,i)=Im(j/2, i/2);
        }
    }
    /*
     A B C
     E F G
     H I J
     pixels A C H J are pixels from original image
     pixels B E G I F are interpolated pixels
     */
    // interpolate pixels B and I
    for ( j = 0; j < h; j += 2)
        for ( i = 1; i < w - 1; i += 2)
            Imnew(j,i)=0.5*(Im(j/2, i/2)+Im(j/2, i/2+1));
    // interpolate pixels E and G
    for ( j = 1; j < h - 1; j += 2)
        for ( i = 0; i < w; i += 2)
            Imnew(j,i)=0.5*(Im(j/2, i/2)+Im(j/2+1, i/2));
    // interpolate pixel F
    for ( j = 1; j < h - 1; j += 2)
        for ( i = 1; i < w - 1; i += 2)
            Imnew(j,i)=0.25*(Im(j/2, i/2)+Im(j/2+1, i/2)+Im(j/2, i/2+1)+Im(j/2+1, i/2+1));
    return imnew;
}

//双线性插值，返回像素间的灰度值
float getPixelBI(CvMat * im, float col, float row)
{
    int irow, icol;
    float rfrac, cfrac;
    float row1 = 0, row2 = 0;
    int width=im->cols;
    int height=im->rows;
#define ImMat(ROW,COL) ((float *)(im->data.fl + im->step/sizeof(float) *(ROW)))[(COL)]
    
    irow = (int) row;
    icol = (int) col;
    
    if (irow < 0 || irow >= height
        || icol < 0 || icol >= width)
        return 0;
    if (row > height - 1)
        row = height - 1;
    if (col > width - 1)
        col = width - 1;
    rfrac = 1.0 - (row - (float) irow);
    cfrac = 1.0 - (col - (float) icol);
    if (cfrac < 1)
    {
        row1 = cfrac * ImMat(irow,icol) + (1.0 - cfrac) * ImMat(irow,icol+1);
    }
    else
    {
        row1 = ImMat(irow,icol);
    }
    if (rfrac < 1)
    {
        if (cfrac < 1)
        {
            row2 = cfrac * ImMat(irow+1,icol) + (1.0 - cfrac) * ImMat(irow+1,icol+1);
        } else
        {
            row2 = ImMat(irow+1,icol);
        }
    }
    return rfrac * row1 + (1.0 - rfrac) * row2;
}

//矩阵归一化
void normalizeMat(CvMat* mat)
{
#define Mat(ROW,COL) ((float *)(mat->data.fl + mat->step/sizeof(float) *(ROW)))[(COL)]
    float sum = 0;
    
    for (unsigned int j = 0; j < mat->rows; j++)
        for (unsigned int i = 0; i < mat->cols; i++)
            sum += Mat(j,i);
    for (unsigned j = 0; j < mat->rows; j++)
        for (unsigned int i = 0; i < mat->rows; i++)
            Mat(j,i) /= sum;
}

//向量归一化
void normalizeVec(float* vec, int dim)
{
    unsigned int i;
    float sum = 0;
    for ( i = 0; i < dim; i++)
        sum += vec[i];
    for ( i = 0; i < dim; i++)
        vec[i] /= sum;
}

//得到向量的欧式长度，2-范数
float GetVecNorm( float* vec, int dim )
{
    float sum=0.0;
    for (unsigned int i=0;i<dim;i++)
        sum+=vec[i]*vec[i];
    return sqrt(sum);
}

//产生1D高斯核
float* GaussianKernel1D(float sigma, int dim)
{
    
    unsigned int i;
    //printf("GaussianKernel1D(): Creating 1x%d vector for sigma=%.3f gaussian kernel/n", dim, sigma);
    
    float *kern=(float*)malloc( dim*sizeof(float) );
    float s2 = sigma * sigma;
    int c = dim / 2;
    float m= 1.0/(sqrt(2.0 * CV_PI) * sigma);
    double v;
    for ( i = 0; i < (dim + 1) / 2; i++)
    {
        v = m * exp(-(1.0*i*i)/(2.0 * s2)) ;
        kern[c+i] = v;
        kern[c-i] = v;
    }
    //   normalizeVec(kern, dim);
    // for ( i = 0; i < dim; i++)
    //  printf("%f  ", kern[i]);
    //  printf("/n");
    return kern;
}

//产生2D高斯核矩阵
CvMat* GaussianKernel2D(float sigma)
{
    // int dim = (int) max(3.0f, GAUSSKERN * sigma);
    int dim = (int) fmax(3.0f, 2.0 * GAUSSKERN *sigma + 1.0f);
    // make dim odd
    if (dim % 2 == 0)
        dim++;
    //printf("GaussianKernel(): Creating %dx%d matrix for sigma=%.3f gaussian/n", dim, dim, sigma);
    CvMat* mat=cvCreateMat(dim, dim, CV_32FC1);
#define Mat(ROW,COL) ((float *)(mat->data.fl + mat->step/sizeof(float) *(ROW)))[(COL)]
    float s2 = sigma * sigma;
    int c = dim / 2;
    //printf("%d %d/n", mat.size(), mat[0].size());
    float m= 1.0/(sqrt(2.0 * CV_PI) * sigma);
    for (int i = 0; i < (dim + 1) / 2; i++)
    {
        for (int j = 0; j < (dim + 1) / 2; j++)
        {
            //printf("%d %d %d/n", c, i, j);
            float v = m * exp(-(1.0*i*i + 1.0*j*j) / (2.0 * s2));
            Mat(c+i,c+j) =v;
            Mat(c-i,c+j) =v;
            Mat(c+i,c-j) =v;
            Mat(c-i,c-j) =v;
        }
    }
    // normalizeMat(mat);
    return mat;
}

//x方向像素处作卷积
float ConvolveLocWidth(float* kernel, int dim, CvMat * src, int x, int y)
{
#define Src(ROW,COL) ((float *)(src->data.fl + src->step/sizeof(float) *(ROW)))[(COL)]
    unsigned int i;
    float pixel = 0;
    int col;
    int cen = dim / 2;
    //printf("ConvolveLoc(): Applying convoluation at location (%d, %d)/n", x, y);
    for ( i = 0; i < dim; i++)
    {
        col = x + (i - cen);
        if (col < 0)
            col = 0;
        if (col >= src->cols)
            col = src->cols - 1;
        pixel += kernel[i] * Src(y,col);
    }
    if (pixel > 1)
        pixel = 1;
    return pixel;
}

//x方向作卷积
void Convolve1DWidth(float* kern, int dim, CvMat * src, CvMat * dst)
{
#define DST(ROW,COL) ((float *)(dst->data.fl + dst->step/sizeof(float) *(ROW)))[(COL)]
    unsigned int i,j;
    
    for ( j = 0; j < src->rows; j++)
    {
        for ( i = 0; i < src->cols; i++)
        {
            //printf("%d, %d/n", i, j);
            DST(j,i) = ConvolveLocWidth(kern, dim, src, i, j);
        }
    }
}

//y方向像素处作卷积
float ConvolveLocHeight(float* kernel, int dim, CvMat * src, int x, int y)
{
#define Src(ROW,COL) ((float *)(src->data.fl + src->step/sizeof(float) *(ROW)))[(COL)]
    unsigned int j;
    float pixel = 0;
    int cen = dim / 2;
    //printf("ConvolveLoc(): Applying convoluation at location (%d, %d)/n", x, y);
    for ( j = 0; j < dim; j++)
    {
        int row = y + (j - cen);
        if (row < 0)
            row = 0;
        if (row >= src->rows)
            row = src->rows - 1;
        pixel += kernel[j] * Src(row,x);
    }
    if (pixel > 1)
        pixel = 1;
    return pixel;
}

//y方向作卷积
void Convolve1DHeight(float* kern, int dim, CvMat * src, CvMat * dst)
{
#define Dst(ROW,COL) ((float *)(dst->data.fl + dst->step/sizeof(float) *(ROW)))[(COL)]
    unsigned int i,j;  
    for ( j = 0; j < src->rows; j++)   
    {  
        for ( i = 0; i < src->cols; i++)   
        {  
            //printf("%d, %d/n", i, j);  
            Dst(j,i) = ConvolveLocHeight(kern, dim, src, i, j);  
        }  
    }  
}  

//卷积模糊图像  
int BlurImage(CvMat * src, CvMat * dst, float sigma)   
{  
    float* convkernel;  
    int dim = (int) fmax(3.0f, 2.0 * GAUSSKERN * sigma + 1.0f);
    CvMat *tempMat;  
    // make dim odd  
    if (dim % 2 == 0)  
        dim++;  
    tempMat = cvCreateMat(src->rows, src->cols, CV_32FC1);  
    convkernel = GaussianKernel1D(sigma, dim);  
    
    Convolve1DWidth(convkernel, dim, src, tempMat);
    Convolve1DHeight(convkernel, dim, tempMat, dst);  
    cvReleaseMat(&tempMat);  
    return dim;  
}