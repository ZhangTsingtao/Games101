// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f pixcelCenter(x, y, 0);
    Vector3f points[3];
    for (int i = 0; i < 3; i++)
    {
        points[i] << _v[i](0), _v[i](1), 0;
    }

    Vector3f edges[3], vecs[3];

    edges[0] = points[0] - points[1];
    edges[1] = points[1] - points[2];
    edges[2] = points[2] - points[0];

    vecs[0] = points[0] - pixcelCenter;
    vecs[1] = points[1] - pixcelCenter;
    vecs[2] = points[2] - pixcelCenter;

    int judge = 0;
    for (int k = 0; k < 3;k++)
    {
        //std::cout << "z " << edges[k].cross(vecs[k]).z() << std::endl;

        if (edges[k].cross(vecs[k]).z() >= 0) judge++;
        else judge--;
    }
    //std::cout << "judge: " << judge << std::endl;
    if (judge != 3 && judge != -3)
    {
        return false;
    }
    else return true;

}
//static bool insideTriangle(float x, float y, const Vector3f* _v)
//{
//    // TODO: Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
//    //MYCODE
//    Eigen::Vector3f la, lb, lc;
//    la << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0;
//    lb << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0;
//    lc << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0;
//    Eigen::Vector3f ap, bp, cp;
//    ap << x - _v[0].x(), y - _v[0].y(), 0;
//    bp << x - _v[1].x(), y - _v[1].y(), 0;
//    cp << x - _v[2].x(), y - _v[2].y(), 0;
//    float za, zb, zc;
//    za = la.cross(ap).z();
//    zb = lb.cross(bp).z();
//    zc = lc.cross(cp).z();
//    if ((za >= 0 && zb >= 0 && zc >= 0) || (za < 0 && zb < 0 && zc < 0))
//        return true;
//    else
//        return false;
//    //MYCODE END
//}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//    auto v = t.toVector4();
//    
//    // TODO : Find out the bounding box of current triangle.
//
//    int xLeft = width - 1, xRight = 0, yBottom = height - 1, yTop = 0;
//    
//    for (int i = 0; i < 3; i++) 
//    {
//        if (v[i](0) < xLeft) xLeft = v[i](0);
//        if (v[i](0) > xRight) xRight = v[i](0);
//        if (v[i](1) < yBottom) yBottom = v[i](1);
//        if (v[i](1) > yTop) yTop = v[i](1);
//    }
//    
//    int boundingBoxPixcels = (xRight - xLeft + 1) * (yTop - yBottom + 1);
//    int inTriangleCount = 0;
//    // iterate through the pixel and find if the current pixel is inside the triangle
//    for (int i = xLeft; i <= xRight; i++)
//    {
//        for (int j = yBottom; j <= yTop; j++)
//        {
//
//            if (insideTriangle(i + 0.5f, j + 0.5f, t.v))
//            {
//                inTriangleCount++;
//
//                // If so, use the following code to get the interpolated z value.
//                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
//                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                z_interpolated *= w_reciprocal;
//
//                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//                
//                if (z_interpolated < depth_buf[get_index(i, j)]) 
//                {
//                    depth_buf[get_index(i, j)] = z_interpolated;
//                    set_pixel(Vector3f(i, j, 0), t.getColor());
//                }
//                
//            }
//        }
//    }
//
//    //examine how many pixcels are in triangle within the bounding box
//    /*float inTriangleRatio = (float)inTriangleCount / (float)boundingBoxPixcels;
//    std::cout << "in triangle pixcels: " << inTriangleCount << std::endl;
//    std::cout << "bounding box pixcels: " << boundingBoxPixcels << std::endl;
//    std::cout << "in triangle ratio: " << inTriangleRatio << std::endl;*/
//}

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.

    //my code starts
    int xLeft = width - 1, xRight = 0, yBottom = height - 1, yTop = 0;

    for (int i = 0; i < 3; i++)
    {
        if (v[i](0) < xLeft) xLeft = v[i](0);
        if (v[i](0) > xRight) xRight = v[i](0);
        if (v[i](1) < yBottom) yBottom = v[i](1);
        if (v[i](1) > yTop) yTop = v[i](1);
    }

    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int i = xLeft; i <= xRight; i++)
    {
        for (int j = yBottom; j <= yTop; j++)
        {
            //super-sampling anti-aliasing
            Vector3f sampleList[4];
            sampleList[0] << i + 0.25, j + 0.25, 1000.0f;
            sampleList[1] << i + 0.75, j + 0.25, 1000.0f;
            sampleList[2] << i + 0.25, j + 0.75, 1000.0f;
            sampleList[3] << i + 0.75, j + 0.75, 1000.0f;
            
            int sampleCount = 0;
            int depthCount = 0;

            Vector3f pixelColor = { 0,0,0 };
            for (int k = 0; k < 4;k++)
            {
                if (insideTriangle(sampleList[k].x(), sampleList[k].y(), t.v))
                {
                    sampleCount++;
                    pixelColor += t.getColor() / 4.0f;
                    
                    // If so, use the following code to get the interpolated z value.
                    auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    if (z_interpolated < depth_buf[get_index(i, j) * 4 + k])
                    {
                        depthCount++;
                        depth_buf[get_index(i, j) * 4 + k] = z_interpolated;
                    }
                }
            }
            if (depthCount > 0)
            {
                set_pixel(Vector3f(i, j, 0), t.getColor() * (float)sampleCount / 4.0f);
            }
        }
    }
    //my code ends
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on