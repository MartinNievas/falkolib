/**
 * FALKOLib - Fast Adaptive Laser Keypoint Orientation-invariant
 * Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
 *
 * This file is part of FALKOLib.
 *
 * FALKOLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * FALKOLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FALKOLib.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <iostream>


#include <falkolib/Feature/FALKO.h>
#include <falkolib/Feature/CGH.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKOExtractor.h>

#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

#include <falkolib/Matching/NNMatcher.h>
#include <falkolib/Matching/AHTMatcher.h>

#include "testData.h"

using namespace std;
using namespace falkolib;

int main(int argc, char** argv) {
  cout << "Generate scan1" << endl;
	LaserScan scan1(0.0054827099666, 2.0 * M_PI, 1147);
	scan1.fromRanges(testRanges3);

  cout << "Generate scan2" << endl;
	LaserScan scan2(0.0054827099666, 2.0 * M_PI, 1147);
	scan2.fromRanges(testRanges5);

	FALKOExtractor fe;
	fe.setMinExtractionRange(0);
	fe.setMaxExtractionRange(10);
	fe.enableSubbeam(true);
	fe.setNMSRadius(0.05);
	fe.setNeighB(0.07);
	fe.setBRatio(2.5);
	fe.setGridSectors(16);

	std::vector<FALKO> keypoints1;
	std::vector<FALKO> keypoints2;


  cout << "Extract keypoints1: " << scan1.ranges.size() << endl;
	fe.extract(scan1, keypoints1);
  cout << "Extract keypoints2:" << scan1.ranges.size() << endl;
	fe.extract(scan2, keypoints2);

	cout << "num keypoints1 extracted: " << keypoints1.size() << endl;
	cout << "num keypoints2 extracted: " << keypoints2.size() << endl;

	CGHExtractor<FALKO> cgh(16);
	BSCExtractor<FALKO> bsc(16, 8);

	vector<CGH> cghDesc1;
	vector<CGH> cghDesc2;
	vector<BSC> bscDesc1;
	vector<BSC> bscDesc2;


	cgh.compute(scan1, keypoints1, cghDesc1);
	bsc.compute(scan1, keypoints1, bscDesc1);
	cgh.compute(scan2, keypoints2, cghDesc2);
	bsc.compute(scan2, keypoints2, bscDesc2);
	
	
	
	cout << endl;
	NNMatcher<FALKO> matcher;
	matcher.setDistanceThreshold(1.6);
	std::vector<std::pair<int, int> > asso;
	std::cout << "num matching NN: " << matcher.match(keypoints1, keypoints2, asso) << endl;
	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << "\t CHG Distance: " << (cghDesc1[i1].distance(cghDesc2[i2])) << "\t BSC Distance: " << (bscDesc1[i1].distance(bscDesc2[i2])) << endl;
		}
	}
	
	cout << endl;
	NNMatcher<FALKO, BSC> matcherFALKOBSC;
	matcherFALKOBSC.setDistanceThreshold(1.1);
	matcherFALKOBSC.setDescriptorThreshold(30);
	std::cout << "num matching NN FALKO BSC: " << matcherFALKOBSC.match(keypoints1, bscDesc1, keypoints2, bscDesc2, asso) << endl;
	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << "\t CHG Distance: " << (cghDesc1[i1].distance(cghDesc2[i2])) << "\t BSC Distance: " << (bscDesc1[i1].distance(bscDesc2[i2])) << endl;
		}
	}
	
	cout << endl;
	NNMatcher<FALKO, CGH> matcherFALKOCGH;
	matcherFALKOCGH.setDistanceThreshold(0.1);
	matcherFALKOCGH.setDescriptorThreshold(0.2);
	std::cout << "num matching NN FALKO CGH: " << matcherFALKOCGH.match(keypoints1, cghDesc1, keypoints2, cghDesc2, asso) << endl;
	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << "\t CHG Distance: " << (cghDesc1[i1].distance(cghDesc2[i2])) << "\t BSC Distance: " << (bscDesc1[i1].distance(bscDesc2[i2])) << endl;
		}
	}
	
	cout << endl;
	NNMatcher<BSC> matcherBSC;
	matcherBSC.setDistanceThreshold(15);
	std::cout << "num matching NN BSC: " << matcherBSC.match(bscDesc1, bscDesc2, asso) << endl;
	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << "\t CHG Distance: " << (cghDesc1[i1].distance(cghDesc2[i2])) << "\t BSC Distance: " << (bscDesc1[i1].distance(bscDesc2[i2])) << endl;
		}
	}
	cout << endl;
	NNMatcher<CGH> matcherCGH;
	matcherCGH.setDistanceThreshold(0.4);
	std::cout << "num matching NN CGH: " << matcherCGH.match(cghDesc1, cghDesc2, asso) << endl;
	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << "\t CHG Distance: " << (cghDesc1[i1].distance(cghDesc2[i2])) << "\t BSC Distance: " << (bscDesc1[i1].distance(bscDesc2[i2])) << endl;
		}
	}
	
	AHTMatcher<FALKO> aht;
	

	return 0;
}

