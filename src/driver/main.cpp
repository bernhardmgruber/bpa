#include <IO.h>
#include <bpa.h>
#include <iostream>

int main(int argc, char* argv[]) try {
	if (argc != 3 && argc != 4) {
		std::cerr << " Usage: " << argv[0] << " <inputPointCloudFile> <radius> [<outputMeshFile>]\n";
		return 1;
	}

	const std::filesystem::path inputFile = argv[1];
	const float radius = std::stof(argv[2]);
	const std::filesystem::path outputFile = argc == 4 ? argv[3] : (std::string{argv[1]} + ".stl");

	const auto points = loadXYZ(inputFile);
	const auto triangles = bpa::reconstruct(points, radius);
	saveTriangles(outputFile, triangles);

	return 0;
} catch (const std::exception& e) {
	std::cerr << "Exception occurred: " << e.what() << '\n';
	return 2;
}
