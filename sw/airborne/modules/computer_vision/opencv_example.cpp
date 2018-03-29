/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

#include <cstdio>

int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;
  // If you want a color image, uncomment this line
  cvtColor(M, image, CV_YUV2BGR_Y422);
  // For a grayscale image, use this one
//  cvtColor(M, image, CV_YUV2GRAY_Y422);

  // Blur it, because we can
//  blur(image, image, Size(5, 5));

  // Canny edges, only works with grayscale image
//  int edgeThresh = 35;
//  Canny(image, image, edgeThresh, edgeThresh * 3);

  // Convert back to YUV422, and put it in place of the original image
//  grayscale_opencv_to_yuv422(image, img, width, height);
  colorrgb_opencv_to_yuv422(image, img, width, height);


  // Test because WTF
  Mat test_bgr(1, 1, CV_8UC3); // BGR test
  Mat test_yuv(1, 1, CV_8UC3);
  printf("WRITE\n");
  test_bgr.at<Vec3b>(0,0) = Vec3b(255, 0, 0);
  printf("CONVERT\n");
  cvtColor(test_bgr, test_yuv, CV_BGR2YUV);
  printf("PRINT\n");
  printf("B = %d, G = %d, R = %d\n",
      test_bgr.at<Vec3b>(0,0)[0],
      test_bgr.at<Vec3b>(0,0)[1],
      test_bgr.at<Vec3b>(0,0)[2]);
  printf("Y = %d, U = %d, V = %d\n",
        test_yuv.at<Vec3b>(0,0)[0],
        test_yuv.at<Vec3b>(0,0)[1],
        test_yuv.at<Vec3b>(0,0)[2]);

  return 0;
}
