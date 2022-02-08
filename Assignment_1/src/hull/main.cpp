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

double inline distance(const Point &p1, const Point &p2) {
	return (std::pow(p1.real() - p2.real(), 2) + std::pow(p1.imag() - p2.imag(), 2));
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// Get Orientation
		double retVal = ((p1.imag() - p0.imag()) * (p2.real() - p1.real())) - 
								 ((p1.real() - p0.real()) * (p2.imag() - p1.imag()));
		
		// Colinear
		if (retVal == 0) {
			// Get distance count without sqrt	
			return ( distance(p0, p1) <= distance(p0, p2) );
		}

		// CLockwise
		if (retVal > 0) return false;
		return true;	
	}
};


/*
*/
bool inline salientAngle(Point &a, Point &b, Point &c) {
	// Use points to create vectors
	// Point u( (b.real() - a.real()), (b.imag() - a.imag()) ); 
	// Point v( (b.real() - c.real()), (b.imag() - c.imag()) ); 

	// // Angle = arccos( (a.b) / |a||b| )
	// double uv = (u.real() * v.real()) + (u.imag() * v.imag());
	// double magU = std::sqrt( std::pow(u.real(), 2) + std::pow(u.imag(), 2) );
	// double magV = std::sqrt( std::pow(v.real(), 2) + std::pow(v.imag(), 2) );
	// double angle = std::acos( uv / (magU * magV) );

	// std::cout << angle << std::endl;
	// if (angle > 3.14159) return false;

	// return true;

	double retVal = ((b.imag() - a.imag()) * (c.real() - b.real())) - 
								 ((b.real() - a.real()) * (c.imag() - b.imag()));

	if (retVal >=0 ) return false;
	
	return true;
}


////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;

	// Get left most point
	double minY = points[0].imag();
	int min = 0, pos = 0;
	for (auto const& value : points) {
		if ((value.imag() < minY) || 
				(minY == value.imag() && value.real() < points[min].real())) {
			min = pos;
		}
		pos++;
	}

	std::cout << points[min] << std::endl;
	std::cout << "---------" << std::endl;

	order.p0 = points[min];
	std::sort(points.begin(), points.end(), order);
	Polygon hull;

	//  DEBUG PRINT
	// for (auto const& value : points) std::cout << value << std::endl;

	// TODO
	for (int i=0; i < points.size(); i++){
		if (i == 0 || i == 1 ) { hull.push_back(points[i]); continue; }
		
		std::cout << "i-2" << points[i-2] << std::endl;
		std::cout << "i-1" << points[i - 1] << std::endl;
		std::cout << "i" << points[i] << std::endl;

		if ( salientAngle(points[i-2], points[i-1], points[i]) ) {
			std::cout << "Popped " << i << std::endl;
			points.pop_back();
		}


		hull.push_back(points[i]);
	}
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
	std::cout << '\n';
	std::cout << '\n';
	std::cout << '\n';

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
	save_obj(argv[2], hull);

	return 0;
}
