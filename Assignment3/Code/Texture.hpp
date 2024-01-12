//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
using namespace std;
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        //u,v may be out of range, introducing to serious problem
        //my code starts
        if (u < 0) { std::cout << "u<0 u=" << u << std::endl; u = 0;}
        if (u > 1) { std::cout << "u>1 u=" << u << std::endl; u = 1;}
        if (v < 0) { std::cout << "v<0 v=" << v << std::endl; v = 0;}
        if (v > 1) { std::cout << "v>1 v=" << v << std::endl; v = 1;}
        //my code ends
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        //std::cout << u_img << ", " << v_img << std::endl;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        //my code starts
        if (u < 0) { std::cout << "u<0 u=" << u << std::endl; u = 0; }
        if (u > 1) { std::cout << "u>1 u=" << u << std::endl; u = 1; }
        if (v < 0) { std::cout << "v<0 v=" << v << std::endl; v = 0; }
        if (v > 1) { std::cout << "v>1 v=" << v << std::endl; v = 1; }
        
        auto u_img = u * (width);
        auto v_img = (1 - v) * (height);

        auto u00 = getColor(floor(u_img) / width, 1 - (floor(v_img)) / height);
        auto u10 = getColor(ceil(u_img) / width, 1 - (floor(v_img)) / height);
        auto u01 = getColor(floor(u_img) / width, 1 - (ceil(v_img)) / height);
        auto u11 = getColor(ceil(u_img) / width, 1 - (ceil(v_img)) / height);

        auto u0 = u00 + (u_img - floor(u_img)) * (u10 - u00);
        auto u1 = u01 + (u_img - floor(u_img)) * (u11 - u01);
        auto u_bilinear = u0 + (v_img - floor(v_img)) * (u1 - u0);
        
        return Eigen::Vector3f(u_bilinear[0], u_bilinear[1], u_bilinear[2]);
        //my code ends

    }
};
#endif //RASTERIZER_TEXTURE_H
