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
    Vector3f A = _v[0], 
                B = _v[1], 
                C = _v[2];
    Vector3f P = Vector3f(x, y, 0);
    Vector3f AB = B - A,
                AP = P - A,
                BC = C - B,
                BP = P - B,
                CA = A - C,
                CP = P - C;
    Vector3f cross_AB_AP = AB.cross(AP),
                cross_BC_BP = BC.cross(BP),
                cross_CA_CP = CA.cross(CP);
    
    if (cross_AB_AP.dot(cross_BC_BP) >= 0 && cross_BC_BP.dot(cross_CA_CP) >= 0 && cross_CA_CP.dot(cross_AB_AP) >= 0)
    {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type, int samples_per_row, int threshold)
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

        rasterize_triangle(t, samples_per_row, threshold);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, int s, int threshold) {
    auto v = t.toVector4();
    bool __use_MSAA = true; // Set to true if you want to use MSAA, false otherwise.
    
    for (int x = std::floor(std::min({v[0].x(), v[1].x(), v[2].x()})); x <= std::ceil(std::max({v[0].x(), v[1].x(), v[2].x()})); x++) {
        for (int y = std::floor(std::min({v[0].y(), v[1].y(), v[2].y()})); y <= std::ceil(std::max({v[0].y(), v[1].y(), v[2].y()})); y++) {
            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;
            
            // first make sure that the pixel is before the exist pixel
            if (z_interpolated < depth_buf[get_index(x, y)]) {
                if (__use_MSAA)
                {
                    const int samples_per_row = s == -1 ? 3 : s;  // Number of samples per pixel for MSAA
                    const bool against_black_edge = true; // Set to true if you want to use black edge detection, false otherwise.

                    /* MSAA part */
                    const int N = samples_per_row * samples_per_row;    // Number of samples per pixel for MSAA
                    const float step = 1.0f / samples_per_row; // Step size for each sample
                    const float half = step * 0.5f;
                    int covered = 0;
                    for (int i = 0; i < samples_per_row; ++i)
                        for (int j = 0; j < samples_per_row; ++j)
                        {
                            float sx = x + half + step * i;
                            float sy = y + half + step * j;
                            // Check whether you have modified the insideTriangle function to accept float parameters
                            if (insideTriangle(sx, sy, t.v))
                                covered++;
                        }

                    /* against black edge part */
                    // threshold is for black edge avoidance
                    // the bigger the threshold, the better the black edge avoidance effect,
                    threshold = std::max(1, std::min(threshold, N));
                    if (covered >= threshold)  // Only render the pixel if it is covered by enough samples and is in front of the existing pixel
                    {
                        depth_buf[get_index(x, y)] = z_interpolated;
                        Eigen::Vector3f color = t.getColor() * (float(covered) / N);
                        set_pixel(Eigen::Vector3f(x, y, 0), color);
                    }
                }
                else if (insideTriangle(x, y, t.v))
                {
                    // If the pixel is inside the triangle, update the depth buffer and set the pixel color
                    depth_buf[get_index(x, y)] = z_interpolated;
                    set_pixel(Eigen::Vector3f(x, y, 0), t.getColor());
                }
            }
        }
    }
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
    depth_buf.resize(w * h);
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