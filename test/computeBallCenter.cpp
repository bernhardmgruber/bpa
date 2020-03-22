#include <gtest/gtest.h>
#include <glm/gtx/io.hpp>

#include "../src/bpa.cpp"

using namespace bpa;

TEST(computeBallCenter, isosceles) {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = bpa::computeBallCenter(f, 10);
	EXPECT_TRUE(center);
	EXPECT_EQ(center.value(), (vec3{5, 5, 7.07106781f}));
}

TEST(computeBallCenter, isoscelesLargerRadius) {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = bpa::computeBallCenter(f, 100);
	EXPECT_TRUE(center);
	EXPECT_EQ(center.value(), (vec3{5, 5, 99.7496872f}));
}

TEST(computeBallCenter, equilateral) {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{5, std::sqrt(3.0f) * 10.0f / 2.0f, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = bpa::computeBallCenter(f, 10);
	EXPECT_TRUE(center);
	EXPECT_EQ(center.value(), (vec3{4.99999952f, 2.88675070f, 8.16496658f}));
}

TEST(computeBallCenter, radiusTooSmall) {
	MeshPoint a{vec3{0, 0, 0}};
	MeshPoint b{vec3{10, 0, 0}};
	MeshPoint c{vec3{0, 10, 0}};
	MeshFace f{&a, &b, &c};
	const auto center = bpa::computeBallCenter(f, 1);
	EXPECT_FALSE(center);
}
