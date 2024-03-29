#include <IO.h>
#include <bpa.h>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <glm/glm.hpp>
#include <iostream>
#include <numbers>

using namespace bpa;

namespace {
	auto createSphericalCloud(int slices, int stacks) -> std::vector<Point> {
		std::vector<Point> points;
		points.emplace_back(Point{{0, 0, -1}, {0, 0, -1}});
		for (auto slice = 0; slice < slices; slice++) {
			for (auto stack = 1; stack < stacks; stack++) {
				const auto yaw = (static_cast<double>(slice) / slices) * 2 * std::numbers::pi;
				const auto z = std::sin((static_cast<double>(stack) / stacks - 0.5) * std::numbers::pi);
				const auto r = std::sqrt(1 - z * z);

				glm::vec3 v;
				v.x = static_cast<float>(r * std::sin(yaw));
				v.y = static_cast<float>(r * std::cos(yaw));
				v.z = static_cast<float>(z);
				points.push_back({v, glm::normalize(v - glm::vec3{})});
			}
		}
		points.emplace_back(Point{{0, 0, 1}, {0, 0, 1}});
		return points;
	}

	auto measuredReconstruct(const std::vector<Point>& points, float radius) -> std::vector<Triangle> {
		const auto start = std::chrono::high_resolution_clock::now();
		auto result = reconstruct(points, radius);
		const auto end = std::chrono::high_resolution_clock::now();
		const auto seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
		std::cerr << "Points: " << points.size() << ", Triangles: " << result.size() << ", T/s: " << result.size() / seconds << '\n';
		return result;
	}
} // namespace

TEST_CASE("sphere_36_18", "[reconstruct]") {
	const auto cloud = createSphericalCloud(36, 18);
	savePoints("sphere_36_18_cloud.ply", cloud);
	const auto mesh = measuredReconstruct(cloud, 0.3f);
	CHECK(!mesh.empty());
	saveTriangles("sphere_36_18_mesh.stl", mesh);
}

TEST_CASE("sphere_100_50", "[reconstruct]") {
	const auto cloud = createSphericalCloud(100, 50);
	savePoints("sphere_100_50_cloud.ply", cloud);
	const auto mesh = measuredReconstruct(cloud, 0.1f);
	CHECK(!mesh.empty());
	saveTriangles("sphere_100_50_mesh.stl", mesh);
}

TEST_CASE("sphere_200_100", "[reconstruct]") {
	const auto cloud = createSphericalCloud(200, 100);
	savePoints("sphere_200_100_cloud.ply", cloud);
	const auto mesh = measuredReconstruct(cloud, 0.04f);
	CHECK(!mesh.empty());
	saveTriangles("sphere_200_100_mesh.stl", mesh);
}

TEST_CASE("tetrahedron", "[reconstruct]") {
	const auto cloud = std::vector<Point>{{{0, 0, 0}, glm::normalize(glm::vec3{-1, -1, -1})}, {{0, 1, 0}, glm::normalize(glm::vec3{0, 1, 0})},
		{{1, 0, 0}, glm::normalize(glm::vec3{1, 0, 0})}, {{0, 0, 1}, glm::normalize(glm::vec3{0, 0, 1})}};
	savePoints("tetrahedron_cloud.ply", cloud);
	const auto mesh = measuredReconstruct(cloud, 2);
	CHECK(!mesh.empty());
	saveTriangles("tetrahedron_mesh.stl", mesh);
}

TEST_CASE("cube", "[reconstruct]") {
	const auto cloud = std::vector<Point>{
		{{-1, -1, -1}, glm::normalize(glm::vec3{-1, -1, -1})},
		{{-1, +1, -1}, glm::normalize(glm::vec3{-1, +1, -1})},
		{{+1, +1, -1}, glm::normalize(glm::vec3{+1, +1, -1})},
		{{+1, -1, -1}, glm::normalize(glm::vec3{+1, -1, -1})},
		{{-1, -1, +1}, glm::normalize(glm::vec3{-1, -1, +1})},
		{{-1, +1, +1}, glm::normalize(glm::vec3{-1, +1, +1})},
		{{+1, +1, +1}, glm::normalize(glm::vec3{+1, +1, +1})},
		{{+1, -1, +1}, glm::normalize(glm::vec3{+1, -1, +1})},
	};
	savePoints("cube_cloud.ply", cloud);
	const auto mesh = measuredReconstruct(cloud, 2);
	CHECK(!mesh.empty());
	saveTriangles("cube_mesh.stl", mesh);
}

TEST_CASE("bunny", "[reconstruct]") {
	const auto cloud = loadXYZ("../test/data/bunny.xyz");
	const auto mesh = measuredReconstruct(cloud, 0.002f);
	CHECK(!mesh.empty());
	saveTriangles("bunny_mesh.stl", mesh);
}