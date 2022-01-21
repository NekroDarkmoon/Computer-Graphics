////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// TODO
	return 0;
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// TODO
		return true;
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// TODO
	return false;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// TODO
	order.p0 = Point(0, 0);
	std::sort(points.begin(), points.end(), order);
	Polygon hull;
	// TODO
	// use salientAngle(a, b, c) here
	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	
	// Check if filename is valid
	if (!in) {
		std::cerr << "Filename: " << filename << " not found." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Scan input from file
	std::string inputString;
	bool isCount = true;

	while (std::getline(in, inputString)) {
		if (isCount) { isCount = false; continue; }

		// Split into doubles
		std::vector<double> threePoints;
		std::string inputPoint;
		std::istringstream inputStream;
		inputStream.str(inputString);

		while (std::getline(inputStream, inputPoint, ' ')) {
			threePoints.push_back(std::stod(inputPoint));
		}

		// Create Point and push to vector
		Point point(threePoints[0], threePoints[1]);
		points.push_back(point);
	}

	//  DEBUG PRINT
	for (auto const& value : points) std::cout << value << std::endl;

	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	// Validate args
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	// Load points
	std::vector<Point> points = load_xyz(argv[1]);

	// Generate convex hull
	Polygon hull = convex_hull(points);

	// Save as obj
	// save_obj(argv[2], hull);

	return 0;
}
