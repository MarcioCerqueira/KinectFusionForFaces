#include "Mediators/MeshGenerationMediator.h"

MeshGenerationMediator::MeshGenerationMediator() {
	marchingCubes = new MarchingCubes();
}

MeshGenerationMediator::~MeshGenerationMediator() {
	delete marchingCubes;
}

void MeshGenerationMediator::saveMesh(TsdfVolume *tsdfVolume) {

	Mesh *mesh = new Mesh();
	mesh->extractFromGrid(marchingCubes, tsdfVolume);
	mesh->computePolygonMesh();

	
	char fileToSave[1000];
	std::cout << "Write filename (.ply)..." << std::endl;
	std::cin >> fileToSave;
	std::cout << fileToSave << std::endl;
	pcl::io::savePLYFile(fileToSave, mesh->getPolygonMesh());
	std::cout << "Model saved..." << std::endl;
	
	delete mesh;

}