#pragma once

#include "bpa.h"

#include <filesystem>
#include <fstream>

inline void saveTriangles(std::filesystem::path path, const std::vector<bpa::Triangle>& triangles) {
	if (path.has_parent_path())
		create_directories(path.parent_path());
	std::ofstream f{path, std::ios::binary};
	const char header[80] = "STL whatever";
	f.write(header, sizeof(header));

	uint32_t count = static_cast<uint32_t>(triangles.size());
	f.write(reinterpret_cast<const char*>(&count), sizeof(count));

	const uint16_t attributeCount = 0;
	for (const auto& t : triangles) {
		const auto normal = glm::normalize(glm::cross(t[0] - t[1], t[0] - t[2]));
		f.write(reinterpret_cast<const char*>(&normal), sizeof(normal));
		f.write(reinterpret_cast<const char*>(&t), sizeof(t));
		f.write(reinterpret_cast<const char*>(&attributeCount), sizeof(attributeCount));
	}

	f.seekp(sizeof(header), std::ios::beg);
	f.write(reinterpret_cast<const char*>(&count), sizeof(count));

	f.close();
}

inline void savePoints(std::filesystem::path path, const std::vector<bpa::Point>& points) {
	if (path.has_parent_path())
		create_directories(path.parent_path());
	std::ofstream f{path, std::ios::binary};
	f << "ply\n";
	f << "format binary_little_endian 1.0\n";
	f << "element vertex " << points.size() << "\n";
	f << "property float x\n";
	f << "property float y\n";
	f << "property float z\n";
	f << "property float nx\n";
	f << "property float ny\n";
	f << "property float nz\n";
	f << "end_header\n";
	f.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(points[0]));
	f.close();
}

inline void savePoints(std::filesystem::path path, const std::vector<glm::vec3>& points) {
	if (path.has_parent_path())
		create_directories(path.parent_path());
	std::ofstream f{path, std::ios::binary};
	f << "ply\n";
	f << "format binary_little_endian 1.0\n";
	f << "element vertex " << points.size() << "\n";
	f << "property float x\n";
	f << "property float y\n";
	f << "property float z\n";
	f << "end_header\n";
	f.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(points[0]));
	f.close();
}

inline auto loadXYZ(std::filesystem::path path) -> std::vector<bpa::Point> {
	std::ifstream f{path};
	if (!f)
		throw std::runtime_error("Faild to read file " + path.string());

	std::vector<bpa::Point> result;
	while (!f.eof()) {
		bpa::Point p;
		f >> p.pos.x >> p.pos.y >> p.pos.z >> p.normal.x >> p.normal.y >> p.normal.z;
		result.push_back(p);
	}
	return result;
}
