// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

// Image height
const int H = 480;

// Camera settings
const float near_plane = 1.5; // AKA focal length
const float far_plane = near_plane * 100;
const float field_of_view = 0.7854; // 45 degrees
const float aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3f camera_position(0, 0, 3);
const Vector3f camera_gaze(0, 0, -1);
const Vector3f camera_top(0, 1, 0);

// Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

// Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
// Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

// Fills the different arrays
void setup_scene()
{
    // Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    // Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    // Setup uniform
    uniform.view_transform << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Setup camera, compute w, u, v
    const Vector3f w = -1.0 * camera_gaze / camera_gaze.norm();
    const Vector3f u = camera_top.cross(w) / camera_top.cross(w).norm();
    const Vector3f v = w.cross(u);

    // Compute the camera transformation
    Matrix4f temp;
    temp << u(0), v(0), w(0), camera_position(0),
        u(1), v(1), w(1), camera_position(1),
        u(2), v(2), w(2), camera_position(2),
        0, 0, 0, 1;

    const Matrix4f C = temp.inverse();

    // Setup projection matrix
    const double n = -near_plane;
    const double f = -far_plane;
    const double t = near_plane * tan(field_of_view / 2);
    const double r = t * aspect_ratio;
    const double b = -t;
    const double l = -r;

    Matrix4f P;
    if (is_perspective)
    {
        // TODO setup prespective camera
        Matrix4f ortho;
        ortho << (2 / (r - l)), 0, 0, -((r + l) / (r - l)),
            0, (2 / (t - b)), 0, -((t + b) / (t - b)),
            0, 0, (2 / (n - f)), -((n + f) / (n - f)),
            0, 0, 0, 1;

        P << n, 0, 0, 0,
            0, n, 0, 0,
            0, 0, (n + f), -(f * n),
            0, 0, 1, 0;

        P = ortho * P;
    }
    else
    {
        P << (2 / (r - l)), 0, 0, -((r + l) / (r - l)),
            0, (2 / (t - b)), 0, -((t + b) / (t - b)),
            0, 0, (2 / (n - f)), -((n + f) / (n - f)),
            0, 0, 0, 1;
    }

    uniform.view_transform = P * C;
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Fill the shader
        VertexAttributes out;
        out.position = uniform.view_transform * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // Fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    // Build the vertex attributes from vertices and facets
    for (int i = 0; i < facets.rows(); i++)
    {
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2)));
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2)));
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2)));
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4f compute_rotation(const double alpha)
{
    // TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4f res;
    res << cos(alpha), 0., sin(alpha), 0,
        0, 1, 0, 0,
        -sin(alpha), 0, cos(alpha), 0,
        0, 0, 0, 1;

    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4f trafo = compute_rotation(alpha);

    program.VertexShader = [trafo](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Fill the shader
        VertexAttributes out;
        out.position = uniform.view_transform * trafo * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Fill the shader

        FragmentAttributes out(1, 0, 0);
        out.position = va.position;
        out.position[2] = -1 * out.position[2];
        return out;

        // return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        // Fill the shader
        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        }
        else
        {
            return previous;
        }
    };

    std::vector<VertexAttributes> vertex_attributes;

    // Generate the vertex attributes for the edges and rasterize the lines
    // Use the transformation matrix
    for (int i = 0; i < facets.rows(); i++)
    {
        // Edge 1
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2)));
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2)));

        // Edge 2
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2)));
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2)));

        // Edge 3
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2)));
        vertex_attributes.emplace_back(VertexAttributes(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2)));
    }

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Transform the position and the normal
        // Compute the correct lighting
        VertexAttributes out;
        out.position = uniform.view_transform * uniform.transform * va.position;

        Vector3d lights_color(0, 0, 0);
        for (int i = 0; i < light_positions.size(); i++)
        {
            const Vector3d &light_pos = light_positions[i];
            const Vector3d &light_color = light_colors[i];

            const Vector3d diff_color = obj_diffuse_color;
            const Vector3d spec_color = obj_specular_color;

            // Light
            const Vector3d p(out.position(0), out.position(1), out.position(2));
            const Vector3d Li = (light_pos - p).normalized();

            const Vector3d diffuse = diff_color * std::max(Li.dot(va.normal.cast<double>()), 0.0);

            Vector3d h = (p - camera_position.cast<double>()) + (light_pos - p);
            h = h / h.norm();
            const Vector3d specular = spec_color * std::pow(std::max(h.dot(va.normal.cast<double>()), 0.0), obj_specular_exponent);

            const Vector3d D = light_pos - p;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }

        out.color = (ambient_light + lights_color).cast<float>();
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform)
    {
        // Create the correct fragment
        FragmentAttributes out(va.color(0), va.color(1), va.color(2));
        out.position = va.position;
        // out.position[2] = -1 * out.position[2];
        return out;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous)
    {
        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        }
        else
        {
            return previous;
        }
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4f trafo = compute_rotation(alpha);
    uniform.transform = trafo;

    std::vector<VertexAttributes> vertex_attributes;
    // Compute the normals
    // Set material colors

    for (int i = 0; i < facets.rows(); i++)
    {
        const Vector3d a = vertices.row(facets(i, 0));
        const Vector3d b = vertices.row(facets(i, 1));
        const Vector3d c = vertices.row(facets(i, 2));

        const Vector3d triag_u = b - a;
        const Vector3d triag_v = c - a;
        Vector3d N = triag_u.cross(triag_v).normalized();

        VertexAttributes v1(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v1.normal = N.cast<float>();
        VertexAttributes v2(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v2.normal = N.cast<float>();
        VertexAttributes v3(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));
        v3.normal = N.cast<float>();

        vertex_attributes.emplace_back(v1);
        vertex_attributes.emplace_back(v2);
        vertex_attributes.emplace_back(v3);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4f trafo = compute_rotation(alpha);
    uniform.transform = trafo;

    // Compute the vertex normals as vertex normal average
    std::vector<Vector3f> vertex_normals(vertices.rows());
    for (int i = 0; i < vertex_normals.size(); ++i)
        vertex_normals[i] = Vector3f(0, 0, 0);

    for (int i = 0; i < facets.rows(); i++)
    {
        const Vector3d a = vertices.row(facets(i, 0));
        const Vector3d b = vertices.row(facets(i, 1));
        const Vector3d c = vertices.row(facets(i, 2));

        const Vector3d triag_u = b - a;
        const Vector3d triag_v = c - a;
        Vector3d N = triag_u.cross(triag_v).normalized();

        vertex_normals[facets(i, 0)] += N.cast<float>();
        vertex_normals[facets(i, 1)] += N.cast<float>();
        vertex_normals[facets(i, 2)] += N.cast<float>();
    }

    for (int i = 0; i < vertex_normals.size(); ++i)
        vertex_normals[i] = vertex_normals[i].normalized();

    // Create vertex attributes
    // Set material colors
    std::vector<VertexAttributes> vertex_attributes;
    for (int i = 0; i < facets.rows(); ++i)
    {
        VertexAttributes v1(vertices(facets(i, 0), 0), vertices(facets(i, 0), 1), vertices(facets(i, 0), 2));
        v1.normal = vertex_normals[facets(i, 0)];
        VertexAttributes v2(vertices(facets(i, 1), 0), vertices(facets(i, 1), 1), vertices(facets(i, 1), 2));
        v2.normal = vertex_normals[facets(i, 1)];
        VertexAttributes v3(vertices(facets(i, 2), 0), vertices(facets(i, 2), 1), vertices(facets(i, 2), 2));
        v3.normal = vertex_normals[facets(i, 2)];

        vertex_attributes.emplace_back(v1);
        vertex_attributes.emplace_back(v2);
        vertex_attributes.emplace_back(v3);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    // TODO: add the animation
    int delay = 20;
    GifWriter g;

    // Wireframe Animation
    GifBegin(&g, "wireframe.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 2 * M_PI; i += M_PI / 12)
    {
        std::cout << i << "/" << 2 * M_PI << std::endl;
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    std::cout << std::endl;
    GifEnd(&g);

    // Flat Shading Animation
    GifBegin(&g, "flat_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 2 * M_PI; i += M_PI / 12)
    {
        std::cout << i << "/" << 2 * M_PI << std::endl;
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    std::cout << std::endl;
    GifEnd(&g);

    // PV Shading Animation
    GifBegin(&g, "pv_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 2 * M_PI; i += M_PI / 12)
    {
        std::cout << i << "/" << 2 * M_PI << std::endl;
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    std::cout << std::endl;
    GifEnd(&g);

    return 0;
}
