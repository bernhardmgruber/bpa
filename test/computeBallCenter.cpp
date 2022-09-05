#include "../src/bpa.cpp"

#include <catch2/catch_test_macros.hpp>
#include <glm/gtx/io.hpp>

using namespace bpa;

TEST_CASE("isosceles", "[computeBallCenter]") {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = computeBallCenter(f, 10);
	REQUIRE(center);
	CHECK(center.value() == (vec3{5, 5, 7.07106781f}));
}

TEST_CASE("isoscelesLargerRadius", "[computeBallCenter]") {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = computeBallCenter(f, 100);
	REQUIRE(center);
	CHECK(center.value() == (vec3{5, 5, 99.7496872f}));
}

TEST_CASE("equilateral", "[computeBallCenter]") {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{5, std::sqrt(3.0f) * 10.0f / 2.0f, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = computeBallCenter(f, 10);
	REQUIRE(center);
	CHECK(center.value() == (vec3{4.99999952f, 2.88675070f, 8.16496658f}));
}

TEST_CASE("radiusTooSmall", "[computeBallCenter]") {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = computeBallCenter(f, 1);
	CHECK(!center);
}
