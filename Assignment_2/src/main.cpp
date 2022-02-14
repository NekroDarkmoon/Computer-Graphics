// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your
                                       // project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
  std::cout << "Simple ray tracer, one sphere with orthographic projection"
            << std::endl;

  const std::string filename("sphere_orthographic.png");
  MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
  MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

  const Vector3d camera_origin(0, 0, 3);
  const Vector3d camera_view_direction(0, 0, -1);

  // The camera is orthographic, pointing in the direction -z and covering the
  // unit square (-1,1) in x and y
  const Vector3d image_origin(-1, 1, 1);
  const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
  const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

  // Single light source
  const Vector3d light_position(-1, 1, 1);

  for (unsigned i = 0; i < C.cols(); ++i)
  {
    for (unsigned j = 0; j < C.rows(); ++j)
    {
      const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;
      // Prepare the ray
      const Vector3d ray_origin = pixel_center;
      const Vector3d ray_direction = camera_view_direction;

      // Intersect with the sphere
      // NOTE: this is a special case of a sphere centered in the origin and for
      // orthographic rays aligned with the z axis
      Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
      const double sphere_radius = 0.9;

      if (ray_on_xy.norm() < sphere_radius)
      {
        // The ray hit the sphere, compute the exact intersection point
        Vector3d ray_intersection(
            ray_on_xy(0), ray_on_xy(1),
            sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

        // Compute normal at the intersection point
        Vector3d ray_normal = ray_intersection.normalized();

        // Simple diffuse model
        C(i, j) = (light_position - ray_intersection).normalized().transpose() *
                  ray_normal;

        // Clamp to zero
        C(i, j) = std::max(C(i, j), 0.);

        // Disable the alpha mask for this pixel
        A(i, j) = 1;
      }
    }
  }

  // Save to png
  write_matrix_to_png(C, C, C, A, filename);
}

bool intersects(Vector3d rayOrigin, Vector3d rayDir, Vector3d pOrigin,
                Vector3d u, Vector3d v, Vector3d &intersection)
{
  Vector3d p = rayDir.cross(v);
  float det = u.dot(p);

  float detInverse = 1.0 / det;
  Vector3d t = rayOrigin - pOrigin;
  Vector3d q = t.cross(u);

  float a = t.dot(p) * detInverse;
  // std::cout << "a: " << a << std::endl;
  float b = rayDir.dot(q) * detInverse;
  // std::cout << "b: " << b << std::endl;

  if (a < 0 || a > 1 || b < 0 || b > 1)
    return false;

  Vector3d temp = rayOrigin + (rayDir * (detInverse * v.dot(q)));
  intersection = temp;

  return true;
}

void raytrace_parallelogram()
{
  std::cout
      << "Simple ray tracer, one parallelogram with orthographic projection"
      << std::endl;

  const std::string filename("plane_orthographic.png");
  MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
  MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

  const Vector3d camera_origin(0, 0, 3);
  const Vector3d camera_view_direction(0, 0, -1);

  // The camera is orthographic, pointing in the direction -z and covering the
  // unit square (-1,1) in x and y
  const Vector3d image_origin(-1, 1, 1);
  const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
  const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

  // Parameters of the parallelogram (position of the lower-left corner +
  // two sides)
  const Vector3d pgram_origin(-0.5, -0.5, 0);
  const Vector3d pgram_u(0, 0.7, -10);
  const Vector3d pgram_v(1, 0.4, 0);

  // Single light source
  const Vector3d light_position(-1, 1, 1);

  for (unsigned i = 0; i < C.cols(); ++i)
  {
    for (unsigned j = 0; j < C.rows(); ++j)
    {
      const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

      // Prepare the ray
      const Vector3d ray_origin = pixel_center;
      const Vector3d ray_direction = camera_view_direction;
      Vector3d intersection;

      // Check if the ray intersects with the parallelogram
      if (intersects(ray_origin, ray_direction, pgram_origin, pgram_u,
                     pgram_v, intersection))
      {
        // The ray hit the parallelogram, compute the exact intersection
        // point
        Vector3d ray_intersection = intersection;

        // Compute normal at the intersection point
        const Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

        // Simple diffuse model
        C(i, j) = (light_position - ray_intersection).normalized().transpose() *
                  ray_normal;

        // Clamp to zero
        C(i, j) = std::max(C(i, j), 0.);

        // Disable the alpha mask for this pixel
        A(i, j) = 1;
      }
    }
  }

  // Save to png
  write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
  std::cout
      << "Simple ray tracer, one parallelogram with perspective projection"
      << std::endl;

  const std::string filename("plane_perspective.png");
  MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
  MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

  const Vector3d camera_origin(0, 0, 3);
  const Vector3d camera_view_direction(0, 0, -1);

  // The camera is perspective, pointing in the direction -z and covering the
  // unit square (-1,1) in x and y
  const Vector3d image_origin(-1, 1, 1);
  const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
  const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

  // Parameters of the parallelogram (position of the lower-left corner +
  // two sides)
  const Vector3d pgram_origin(-0.5, -0.5, 0);
  const Vector3d pgram_u(0, 0.7, -10);
  const Vector3d pgram_v(1, 0.4, 0);

  // Single light source
  const Vector3d light_position(-1, 1, 1);

  for (unsigned i = 0; i < C.cols(); ++i)
  {
    for (unsigned j = 0; j < C.rows(); ++j)
    {
      const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

      // Prepare the ray (origin point and direction)
      const Vector3d ray_origin = camera_origin;
      const Vector3d ray_direction = pixel_center - ray_origin;
      Vector3d intersection;

      // Check if the ray intersects with the parallelogram
      if (intersects(ray_origin, ray_direction, pgram_origin, pgram_u,
                     pgram_v, intersection))
      {
        // The ray hit the parallelogram, compute the exact intersection
        // point
        const Vector3d ray_intersection = intersection;

        // Compute normal at the intersection point
        const Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

        // Simple diffuse model
        C(i, j) = (light_position - ray_intersection).normalized().transpose() *
                  ray_normal;

        // Clamp to zero
        C(i, j) = std::max(C(i, j), 0.);

        // Disable the alpha mask for this pixel
        A(i, j) = 1;
      }
    }
  }

  // Save to png
  write_matrix_to_png(C, C, C, A, filename);
}

// Helper functions for raytrace_shading
inline bool sphereIntersects(const Vector3d ray_origin, const Vector3d ray_direction,
                             const Vector3d sphere_center, const double sphere_radius,
                             double t)
{
  // Set up variables
  double x0, x1;
  Vector3d length = ray_origin - sphere_center;
  double a = ray_direction.dot(ray_direction);
  double b = ray_direction.dot(length) * 2;
  double c = length.dot(length) - sphere_radius;

  // Solve quadratic equation using numberically stable formula
  double discrim = (b * b) - (4 * a * c);
  if (discrim < 0)
    return false;
  else if (discrim == 0)
    x0 = x1 = -0.5 * (b / a);
  else
  {
    double x = (b > 0) ? (-0.5 * (b + sqrt(discrim))) : (-0.5 * (b - sqrt(discrim)));
    x0 = x / a;
    x1 = c / x;
  }

  if (x0 > x1)
    std::swap(x0, x1);
  if (x0 < 0 && x1 < 0)
    return false;
  if (x0 < 0)
    x0 = x1;

  t = x0;
  return true;
}

void raytrace_shading()
{
  std::cout << "Simple ray tracer, one sphere with different shading"
            << std::endl;

  const std::string filename("shading.png");
  MatrixXd CR = MatrixXd::Zero(800, 800); // Store red color
  MatrixXd CG = MatrixXd::Zero(800, 800); // Store green color
  MatrixXd CB = MatrixXd::Zero(800, 800); // Store blue color
  MatrixXd A = MatrixXd::Zero(800, 800);  // Store the alpha mask

  const Vector3d camera_origin(0, 0, 3);
  const Vector3d camera_view_direction(0, 0, -1);

  // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
  const Vector3d image_origin(-1, 1, 1);
  const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
  const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

  // Sphere setup
  const Vector3d sphere_center(0, 0, 0);
  const double sphere_radius = 0.9;

  // Material Params
  const Vector3d diffuse_color(1, 0, 1);
  const double specular_exponent = 100;
  const Vector3d specular_color(0., 0, 1);

  // Single light source
  const Vector3d light_position(-1, 1, 1);
  double ambient = 0.1;

  for (unsigned i = 0; i < CR.cols(); ++i)
  {
    for (unsigned j = 0; j < CR.rows(); ++j)
    {
      const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;
      // Prepare the ray
      const Vector3d ray_origin = camera_origin;
      const Vector3d ray_direction = pixel_center - ray_origin;

      // Intersect with the sphere
      double t;
      const bool isIntersecting = sphereIntersects(ray_origin, ray_direction, sphere_center,
                                                   sphere_radius, t);

      if (isIntersecting)
      {
        // The ray hit the sphere, compute the exact intersection point
        const Vector3d ray_intersection = ray_origin - (t * ray_direction);

        // Compute normal at the intersection point
        const Vector3d ray_normal = ((ray_intersection - sphere_center) / sphere_radius).normalized();

        // TODO: Add shading parameter here
        const double diffuseR = (light_position - ray_intersection).normalized().dot(ray_normal) * diffuse_color(0);
        const double diffuseG = (light_position - ray_intersection).normalized().dot(ray_normal) * diffuse_color(1);
        const double diffuseB = (light_position - ray_intersection).normalized().dot(ray_normal) * diffuse_color(2);

        const Vector3d v = (camera_origin - ray_intersection).normalized();
        const Vector3d l = (light_position - ray_intersection).normalized();
        const Vector3d h = (v + l);

        const double specularR = pow((h).normalized().dot(ray_normal), specular_exponent) * specular_color(0);
        const double specularG = pow((h).normalized().dot(ray_normal), specular_exponent) * specular_color(1);
        const double specularB = pow((h).normalized().dot(ray_normal), specular_exponent) * specular_color(2);

        // const double specularR = (light_position - ray_intersection).normalized().dot(ray_normal) * specular_color(0);
        // const double specularG = (light_position - ray_intersection).normalized().dot(ray_normal) * specular_color(1);
        // const double specularB = (light_position - ray_intersection).normalized().dot(ray_normal) * specular_color(2);

        // Simple diffuse model
        CR(i, j) = ambient + fmax(0, diffuseR) + fmax(0, specularR);
        CG(i, j) = ambient + fmax(0, diffuseG) + fmax(0, specularG);
        CB(i, j) = ambient + fmax(0, diffuseB) + fmax(0, specularB);

        // Clamp to zero
        CR(i, j) = std::max(CR(i, j), 0.);
        CG(i, j) = std::max(CG(i, j), 0.);
        CB(i, j) = std::max(CB(i, j), 0.);

        // Disable the alpha mask for this pixel
        A(i, j) = 1;
      }
    }
  }

  // Save to png
  write_matrix_to_png(CR, CG, CB, A, filename);
}

int main()
{
  // raytrace_sphere();
  // raytrace_parallelogram();
  // raytrace_perspective();
  raytrace_shading();

  return 0;
}
