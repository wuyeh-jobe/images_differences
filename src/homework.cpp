#include "homework.h"

void printUsage(){
	std::cout << "\n\nThese files are used to submit homework for the ACV course.\n\n";
	std::cout << "Note - run the code from the \"bin\" folder:\n\n";
	std::cout << "Usage is as follows:\n\n";
	std::cout << "./homework diff inputimage1.jpeg inputimage2.jpeg output_file_prefix \n\n";
}

int main(int argc, char** argv) {
	if (argc < 2){
		printUsage();
		return 0;
	}

	if (strcmp(argv[1], "diff") == 0) {
			std::string inputImage1 = argv[2];
			std::string inputImage2 = argv[3];
			std::string outputImage = argv[4];
			SpotDifferences::spotImageDifferenceWithDistortion(inputImage1, inputImage2, outputImage);
			return 0;
	}

}
