#include <boost/math/constants/constants.hpp>
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "../src/bpa.h"
#include "../src/IO.h"

namespace {
	constexpr auto pi = boost::math::constants::pi<double>();

	auto createSphericalCloud(int slices, int stacks) {
		std::vector<glm::vec3> points;
		points.emplace_back(0, 0, -1);
		for (auto slice = 0; slice < slices; slice++) {
			for (auto stack = 1; stack < stacks; stack++) {
				const auto yaw = (static_cast<double>(slice) / slices) * 2 * pi;
				const auto z = std::sin((static_cast<double>(stack) / stacks - 0.5) * pi);
				const auto r = std::sqrt(1 - z * z);

				glm::dvec3 v;
				v.x = r * std::sin(yaw);
				v.y = r * std::cos(yaw);
				v.z = z;
				points.push_back(v);
			}
		}
		points.emplace_back(0, 0, 1);
		return points;
	}
}

TEST(reconstruct, sphere_36_18) {
	const auto cloud = createSphericalCloud(36, 18);
	savePoints("sphere_36_18_cloud.ply", cloud);
	const auto mesh = bpa::reconstruct(cloud, 0.3f);
	saveTriangles("sphere_36_18_mesh.stl", mesh);
}

TEST(reconstruct, sphere_100_50) {
	const auto cloud = createSphericalCloud(100, 50);
	savePoints("sphere_100_50_cloud.ply", cloud);
	const auto mesh = bpa::reconstruct(cloud, 0.1f);
	saveTriangles("sphere_100_50_mesh.stl", mesh);
}

TEST(reconstruct, sphere_200_100) {
	const auto cloud = createSphericalCloud(200, 100);
	savePoints("sphere_200_100_cloud.ply", cloud);
	const auto mesh = bpa::reconstruct(cloud, 0.04f);
	saveTriangles("sphere_200_100_mesh.stl", mesh);
}

TEST(reconstruct, tetrahedron) {
	const auto cloud = std::vector<glm::vec3>{
		{0, 0, 0},
		{0, 1, 0},
		{1, 0, 0},
		{0, 0, 1},
	};
	savePoints("tetrahedron_cloud.ply", cloud);
	const auto mesh = bpa::reconstruct(cloud, 2);
	saveTriangles("tetrahedron_mesh.stl", mesh);
}

TEST(reconstruct, cube) {
	const auto cloud = std::vector<glm::vec3>{
		{0, 0, 0},
		{0, 1, 0},
		{1, 1, 0},
		{1, 0, 0},
		{0, 0, 1},
		{0, 1, 1},
		{1, 1, 1},
		{1, 0, 1},
	};
	savePoints("cube_cloud.ply", cloud);
	const auto mesh = bpa::reconstruct(cloud, 2);
	saveTriangles("cube_mesh.stl", mesh);
}

TEST(reconstruct, bunny) {
	const auto cloud = loadXYZ("../test/data/bunny.xyz");
	const auto mesh = bpa::reconstruct(cloud, 0.002f);
	saveTriangles("bunny_mesh.stl", mesh);
}