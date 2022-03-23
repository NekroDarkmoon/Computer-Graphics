////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
  class Node
  {
  public:
    AlignedBox3d bbox;
    int parent;   // Index of the parent node (-1 for root)
    int left;     // Index of the left child (-1 for a leaf)
    int right;    // Index of the right child (-1 for a leaf)
    int triangle; // Index of the node triangle (-1 for internal nodes)
  };

  std::vector<Node> nodes;
  int root;

  AABBTree() = default;                           // Default empty constructor
  AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

// Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; // 45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Max num of recursive calls
const int max_bounce = 3;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

// Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

// Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
// Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

// Fills the different arrays
void setup_scene()
{
  // Loads file
  std::ifstream in(mesh_filename);
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

  // setup tree
  bvh = AABBTree(vertices, facets);

  // Spheres
  sphere_centers.emplace_back(10, 0, 1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(7, 0.05, -1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(4, 0.1, 1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(1, 0.2, -1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(-2, 0.4, 1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(-5, 0.8, -1);
  sphere_radii.emplace_back(1);

  sphere_centers.emplace_back(-8, 1.6, 1);
  sphere_radii.emplace_back(1);

  // parallelograms
  parallelograms.emplace_back();
  parallelograms.back() << -100, 100, -100,
      -1.25, 0, -1.2,
      -100, -100, 100;

  // Lights
  light_positions.emplace_back(8, 8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(6, -8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(4, 8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(2, -8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(0, 8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(-2, -8, 0);
  light_colors.emplace_back(16, 16, 16, 0);

  light_positions.emplace_back(-4, 8, 0);
  light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
  AlignedBox3d box;
  box.extend(a);
  box.extend(b);
  box.extend(c);
  return box;
}

int build_tree(const MatrixXi &F, const MatrixXd &V,
               const MatrixXd &centroids, std::vector<int> primIds,
               std::vector<AABBTree::Node> &nodes)
{
  // Base Case
  if (primIds.size() < 2)
  {
    // Create leaf node
    nodes.emplace_back();
    AABBTree::Node &leaf = nodes.back();

    // Set Attribs
    leaf.parent = -1; // Needs to be set in prev
    leaf.left = -1;
    leaf.right = -1;
    leaf.triangle = primIds[0];

    const MatrixXi v = F.row(leaf.triangle);
    leaf.bbox = bbox_from_triangle(V.row(v(0)), V.row(v(1)), V.row(v(2)));

    // Return node index
    return nodes.size() - 1;
  }

  // Create bounding box
  AlignedBox3d bbox;
  for (int i : primIds)
  {
    const Vector3d x(centroids(i, 0), centroids(i, 1), centroids(i, 2));
    bbox.extend(x);
  }

  // Choose split axis
  int axis;
  if ((bbox.max().x() - bbox.min().x()) > (bbox.max().y() - bbox.min().y()) &&
      (bbox.max().x() - bbox.min().x()) > (bbox.max().z() - bbox.min().z()))
    axis = 0;
  else if ((bbox.max().y() - bbox.min().y()) > (bbox.max().z() - bbox.min().z()))
    axis = 1;
  else
    axis = 2;

  // std::cout << axis << std::endl;

  // const Vector3d d = bbox.diagonal();
  // axis = std::max(d.x(), std::max(d.y(), d.z()));

  // std::cout << axis << std::endl
  //           << std::endl;

  // Sort based on axis
  std::sort(primIds.begin(), primIds.end(),
            [&centroids, axis](int i1, int i2)
            { return centroids(i1, axis) < centroids(i2, axis); });

  // // Get splits
  std::vector<int> primIdsA(primIds.begin(), primIds.begin() + (primIds.size() / 2));
  std::vector<int> primIdsB(primIds.begin() + (primIds.size() / 2), primIds.end());

  // Create node
  nodes.emplace_back();
  // AABBTree::Node &node = nodes.back();
  int idn = nodes.size() - 1;

  // Recurse
  int l = build_tree(F, V, centroids, primIdsA, nodes);
  int r = build_tree(F, V, centroids, primIdsB, nodes);

  nodes[idn].left = l;
  nodes[idn].right = r;

  if (idn == 0)
    nodes[idn].parent = -1;

  // Set parent attribute on children
  nodes[l].parent = idn;
  nodes[r].parent = idn;

  // Set bbox & triangle
  nodes[idn].bbox.extend(nodes[l].bbox);
  nodes[idn].bbox.extend(nodes[r].bbox);
  nodes[idn].triangle = -1;

  return idn;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
  // Compute the centroids of all the triangles in the input mesh
  MatrixXd centroids(F.rows(), V.cols());
  centroids.setZero();
  for (int i = 0; i < F.rows(); ++i)
  {
    for (int k = 0; k < F.cols(); ++k)
    {
      centroids.row(i) += V.row(F(i, k));
    }
    centroids.row(i) /= F.cols();
  }

  // Split each set of primitives into 2 sets of roughly equal size,
  // based on sorting the centroids along one direction or another.

  // Create indicies and sort centroids
  std::vector<int> indices(centroids.rows());
  std::iota(indices.begin(), indices.end(), 0);

  // Make nodes vector
  std::vector<AABBTree::Node> nodes;

  // Build tree
  build_tree(F, V, centroids, indices, nodes);
  this->nodes = nodes;

  // Print tree
  int x = 0;
  for (auto n : nodes)
  {
    int leaf = (n.left == -1 && n.right == -1);
    std::cout << x++ << " " << n.parent << " " << n.left << " " << n.right << " "
              << leaf << " (" << n.bbox.min().transpose() << ") (" << n.bbox.max().transpose() << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction,
                                 const Vector3d &a, const Vector3d &b, const Vector3d &c,
                                 Vector3d &p, Vector3d &N)
{
  // Compute whether the ray intersects the given triangle.
  // If you have done the parallelogram case, this should be very similar to it.

  // Create Vectors
  const Vector3d triag_origin = a;
  const Vector3d triag_u = b - triag_origin;
  const Vector3d triag_v = c - triag_origin;

  // Set the correct intersection point, update p and N to the correct values
  const Vector3d P = ray_direction.cross(triag_v);
  const double det = triag_u.dot(P);
  const double det_inverse = 1. / det;

  const Vector3d T = ray_origin - triag_origin;
  const Vector3d Q = T.cross(triag_u);

  const double v1 = T.dot(P) * det_inverse;
  const double v2 = ray_direction.dot(Q) * det_inverse;

  if (v1 < 0 || v1 > 1 || v2 < 0 || v1 + v2 >= 1)
    return -1;

  const double t = det_inverse * triag_v.dot(Q);
  const Vector3d intersection = ray_origin + (ray_direction * t);

  p = intersection;
  N = -triag_v.cross(triag_u).normalized();

  return t;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
  // TODO:
  // Compute whether the ray intersects the given box.
  // we are not testing with the real surface here anyway.

  double tx1 = (box.min().x() - ray_origin(0)) * (1.0 / ray_direction.x());
  double tx2 = (box.max().x() - ray_origin(0)) * (1.0 / ray_direction.x());

  double tmin = std::min(tx1, tx2);
  double tmax = std::max(tx1, tx2);

  double ty1 = (box.min().y() - ray_origin(1)) * (1.0 / ray_direction.y());
  double ty2 = (box.max().y() - ray_origin(1)) * (1.0 / ray_direction.y());

  tmin = std::min(tmin, std::min(ty1, ty2));
  tmax = std::max(tmax, std::max(ty1, ty2));

  double tz1 = (box.min().z() - ray_origin(2)) * (1.0 / ray_direction.z());
  double tz2 = (box.max().z() - ray_origin(2)) * (1.0 / ray_direction.z());

  tmin = std::min(tmin, std::min(tz1, tz2));
  tmax = std::max(tmax, std::max(tz1, tz2));

  return tmax >= tmin;
}

double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
  // Implement the intersection between the ray and the sphere at index index.
  // return t or -1 if no intersection

  // Variables Set up
  const Vector3d sphere_center = sphere_centers[index];
  const double sphere_radius = sphere_radii[index];
  double t = -1;

  double x0, x1;
  Vector3d length = ray_origin - sphere_center;
  double a = ray_direction.dot(ray_direction);
  double b = ray_direction.dot(length) * 2;
  double c = length.dot(length) - sphere_radius;

  // Solve quadratic using numerically stable formula
  double discrim = (b * b) - (4 * a * c);
  if (discrim < 0)
    return -1;
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
    return -1;

  if (x0 < 0)
    x0 = x1;

  t = x0;

  // Set the correct intersection point, update p to the correct value
  p = ray_origin + (t * ray_direction);
  N = ((p - sphere_center) / sphere_radius).normalized();

  return t;
}

// Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
  // Implement the intersection between the ray and the parallelogram at index index.
  // return t or -1 if no intersection

  const Vector3d pgram_origin = parallelograms[index].col(0);
  const Vector3d A = parallelograms[index].col(1);
  const Vector3d B = parallelograms[index].col(2);
  const Vector3d pgram_u = A - pgram_origin;
  const Vector3d pgram_v = B - pgram_origin;

  // Set the correct intersection point, update p and N to the correct values

  Vector3d P = ray_direction.cross(pgram_v);
  double det = pgram_u.dot(P);
  double det_inverse = 1.0 / det;

  Vector3d T = ray_origin - pgram_origin;
  Vector3d Q = T.cross(pgram_u);

  double a = T.dot(P) * det_inverse;
  double b = ray_direction.dot(Q) * det_inverse;

  if (a < 0 || a > 1 || b < 0 || b > 1)
    return -1;

  double t = det_inverse * pgram_v.dot(Q);
  Vector3d intersection = ray_origin + (ray_direction * t);

  p = intersection;
  N = pgram_v.cross(pgram_u).normalized();

  return t;
}

// Finds the closest intersecting object returns its index
// In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
  int closest_index = -1;
  double closest_t = std::numeric_limits<double>::max();
  Vector3d tmp_p, tmp_N;
  std::vector<AABBTree::Node> nodes = bvh.nodes;
  std::queue<int> q;

  // Traverse Tree
  q.push(0);

  while (!q.empty())
  {
    const int idx = q.front();

    // Check if leaf node
    if (nodes[idx].left == -1 && nodes[idx].right == -1)
    {
      // Fetch triangle and evaluate
      const int tri = nodes[idx].triangle;
      assert(tri != -1);

      const Vector3d a = vertices.row(facets.coeff(tri, 0)),
                     b = vertices.row(facets.coeff(tri, 1)),
                     c = vertices.row(facets.coeff(tri, 2));

      const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

      if (t >= 0)
      {
        if (t < closest_t)
        {
          closest_index = idx;
          closest_t = t;
          p = tmp_p;
          N = tmp_N;
        }
      }
    }

    // Check if intersects with box
    if (!ray_box_intersection(ray_origin, ray_direction, nodes[idx].bbox))
      break;

    // Enqueue children
    if (nodes[idx].left != -1)
      q.push(nodes[idx].left);
    if (nodes[idx].right != -1)
      q.push(nodes[idx].right);

    // Pop front
    q.pop();
  }

  for (int i = 0; i < sphere_centers.size(); ++i)
  {
    // returns t and writes on tmp_p and tmp_N
    const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
    // We have intersection
    if (t >= 0)
    {
      // The point is before our current closest t
      if (t < closest_t)
      {
        closest_index = i;
        closest_t = t;
        p = tmp_p;
        N = tmp_N;
      }
    }
  }

  for (int i = 0; i < parallelograms.size(); ++i)
  {
    // returns t and writes on tmp_p and tmp_N
    const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
    // We have intersection
    if (t >= 0)
    {
      // The point is before our current closest t
      if (t < closest_t)
      {
        closest_index = sphere_centers.size() + i;
        closest_t = t;
        p = tmp_p;
        N = tmp_N;
      }
    }
  }

  if (closest_index < 0)
    return false;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////
// Checks if the light is visible
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction,
                      const Vector3d &light_position)
{
  // Determine if the light is visible here
  Vector3d p, N;

  // Create Epsilon
  Vector3d e = ray_direction * 0.00001;
  const bool nearest_object = find_nearest_object(ray_origin + e, ray_direction, p, N);

  // Check if nothing intersects
  if (!nearest_object)
    return true;

  // // Check if behind light
  double inter_dist = (ray_origin - p).squaredNorm();
  double light_dist = (ray_origin - light_position).squaredNorm();

  if (inter_dist >= light_dist)
    return true;

  return false;
}

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, const int max_bounce)
{
  // Intersection point and normal, these are output of find_nearest_object
  Vector3d p, N;

  const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

  if (!nearest_object)
  {
    // Return a transparent color
    return Vector4d(0, 0, 0, 0);
  }

  // Ambient light contribution
  const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

  // Punctual lights contribution (direct lighting)
  Vector4d lights_color(0, 0, 0, 0);
  for (int i = 0; i < light_positions.size(); ++i)
  {
    const Vector3d &light_position = light_positions[i];
    const Vector4d &light_color = light_colors[i];

    Vector4d diff_color = obj_diffuse_color;

    // Light
    const Vector3d Li = (light_position - p).normalized();

    // Shadows
    if (!is_light_visible(p, Li, light_position))
      continue;

    // Diffuse contribution
    const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

    // Specular contribution
    const Vector3d Hi = (Li - ray_direction).normalized();
    const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
    // Vector3d specular(0, 0, 0);

    // Attenuate lights according to the squared distance to the lights
    const Vector3d D = light_position - p;
    lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
  }

  // Reflection
  Vector4d refl_color = obj_reflection_color;

  if (nearest_object == 4)
    refl_color = Vector4d(0.5, 0.5, 0.5, 0);

  // Compute the color of the reflected ray and add its contribution to the current point color.
  // use refl_color
  const Vector3d refl_direction = ray_direction - (2 * ray_direction.dot(N) * N);
  const Vector3d e = refl_direction * 0.00001;
  Vector4d reflection_color = refl_color;

  // Re-bounce till max_depth has been reached

  if (max_bounce > 0)
    reflection_color = refl_color.cwiseProduct(shoot_ray(p + e, refl_direction, max_bounce - 1));

  // Rendering equation
  Vector4d C = ambient_color + lights_color + reflection_color;

  // Set alpha to 1
  C(3) = 1;

  return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
  std::cout << "Simple ray tracer." << std::endl;

  int w = 640;
  int h = 480;
  MatrixXd R = MatrixXd::Zero(w, h);
  MatrixXd G = MatrixXd::Zero(w, h);
  MatrixXd B = MatrixXd::Zero(w, h);
  MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

  // The camera always points in the direction -z
  // The sensor grid is at a distance 'focal_length' from the camera center,
  // and covers an viewing angle given by 'field_of_view'.
  double aspect_ratio = double(w) / double(h);
  double image_y = focal_length * tan(field_of_view / 2);
  double image_x = image_y * aspect_ratio;

  // The pixel grid through which we shoot rays is at a distance 'focal_length'
  const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
  const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
  const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

  for (unsigned i = 0; i < w; ++i)
  {
    for (unsigned j = 0; j < h; ++j)
    {
      // std::cout << i << ", " << j << std::endl;
      const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

      // Prepare the ray
      Vector3d ray_origin;
      Vector3d ray_direction;

      if (is_perspective)
      {
        // Perspective camera
        ray_origin = camera_position;
        ray_direction = (pixel_center - camera_position).normalized();
      }
      else
      {
        // Orthographic camera
        ray_origin = pixel_center;
        ray_direction = Vector3d(0, 0, -1);
      }

      const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
      R(i, j) = C(0);
      G(i, j) = C(1);
      B(i, j) = C(2);
      A(i, j) = C(3);
    }
  }

  // Save to png
  write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  setup_scene();

  raytrace_scene();
  return 0;
}
