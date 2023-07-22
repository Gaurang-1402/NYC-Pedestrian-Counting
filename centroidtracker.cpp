/*
Usaid Malik
centroidtracker.cpp

this is an object for centroid tracker
it has not been tested yet not commented.

The algorithm has some weakpoints that can be fixed
such as memory allocation and speed.

furthermore this needs to be put into a header and
implementation files for use in yolo.cpp
*/

#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <set>
using namespace std;

struct bboxCoords {
	int x1, y1, x2, y2;
};

struct objectAttrs {
	pair<float, float> objectCenter; // defined as x, y
	int objectID;
	int timeVanished;
	bboxCoords bboxCoords;
};


class centroidTracker {
public:
	centroidTracker(const int maxFrames, const int maxDistance) : maxFramesVanished(maxFrames),
	maxDistanceChange(maxDistance), nextIDToAssign(0) {}
	

	void updateObjects(const vector<objectAttrs>& inputObjects) {
		// seeing if input empty
		if (!inputObjects.size()) {			
			for (auto& entry : previousObjectPositions) {
				entry.second.timeVanished += 1;
				if (entry.second.objectID > maxFramesVanished) {
					previousObjectPositions.erase(entry.first);
				}
			}
			return;
		}

		// finding the centroids
		vector<pair<float, float>> inputObjectsCentroids;
		vector<bboxCoords> inputBboxs;
		for (const objectAttrs& inputObject : inputObjects) {
			pair<float, float> pair = findCentroid(inputObject.bboxCoords);
			inputObjectsCentroids.push_back(pair);
			inputBboxs.push_back(inputObject.bboxCoords);
		}

		// if not tracking anything enter new objs
		if (previousObjectPositions.empty()) {
			for (size_t i = 0; i < inputObjects.size(); ++i) {
				registerObject(inputObjectsCentroids[i], inputBboxs[i]);
			}
		}
		else {
			vector<int> prevObjectIDs;
			vector<pair<float, float>> prevObjectCentroids;
			for (const auto& entry : previousObjectPositions) {
				prevObjectIDs.push_back(entry.first);
				prevObjectCentroids.push_back(entry.second.objectCenter);
			}

			vector<vector<int>> distanceMatrix = calcDistanceMatrix(prevObjectCentroids,
				inputObjectsCentroids);

			vector<pair<size_t, size_t>> sortedElems = calcSorted(distanceMatrix);

			set<int> allRows;
			set<int> allCols;

			for (const pair<size_t, size_t>& elems : sortedElems) {
				allRows.insert(elems.first);
				allCols.insert(elems.second);
			}

			set<int> usedRows;
			set<int> usedCols;

			for (const pair<size_t, size_t>& location : sortedElems) {
				
				if (find(usedRows.begin(), usedRows.end(), location.first) != usedRows.end()
					|| find(usedCols.begin(), usedCols.end(), location.second) != usedCols.end()) {
					continue;
				}

				if (distanceMatrix[location.first][location.second] > maxDistanceChange) {
					continue;
				}
				
				int objID = prevObjectIDs[location.first];
				objectAttrs& obj = previousObjectPositions[objID];
				obj.timeVanished = 0;
				obj.bboxCoords = inputBboxs[location.second];
				obj.objectCenter = inputObjectsCentroids[location.second];

				usedRows.insert(location.first);
				usedCols.insert(location.second);
			}

			vector<int> unusedRows;

			set_difference(allRows.begin(), allRows.end(),
				usedRows.begin(), usedRows.end(),
				back_inserter(unusedRows));

			vector<int> unusedCols;
			set_difference(allCols.begin(), allCols.end(),
				usedCols.begin(), usedCols.end(),
				back_inserter(unusedCols));


			if (distanceMatrix.size() >= distanceMatrix[0].size()) {
				for (const int& row : unusedRows) {
					int objID = prevObjectIDs[row];
					objectAttrs& obj = previousObjectPositions[objID];
					int timeVanished = ++obj.timeVanished;


					if( timeVanished > maxFramesVanished){
						deregisterObject(objID);
					}
				}
			}
			else {
				for (const int& col : unusedCols) {
					registerObject(inputObjectsCentroids[col], inputBboxs[col]);
				}

			}
		}

	}

	vector<vector<int>> calcDistanceMatrix(vector<pair<float, float>> currObjectCentroids,
		vector<pair<float, float>> inputObjectCentroids) {
		// finds the absolute distance betwen the curr objects and the new input objects
		// stored as an n x m matrix where n is the currobject len and m is the inputobject length
		// returns that matrix
		vector<vector<int>> distanceMatrix;
		for (const pair<float, float>& currObj : currObjectCentroids) {
			vector<int> distances; // vector of the distance an element in curr obj has with all other new objs
			for (const pair<float, float>& inpObj : inputObjectCentroids) {
				int distance = round(abs(sqrt(pow(currObj.first + inpObj.first, 2)
					+ pow(currObj.second + inpObj.second, 2))));
				distances.push_back(distance);
			}
			distanceMatrix.push_back(distances);
		}
		return distanceMatrix;
	}

	vector<pair<size_t, size_t>> calcSorted(const vector<vector<int>>& distanceMatrix) {

		vector<pair<int, pair<size_t, size_t>>> smallestElements; //  element, pair<row, col>

		for (size_t i = 0; i < distanceMatrix.size(); ++i) {
			int smallest = distanceMatrix[i][0];
			size_t col = 0;
			for (size_t j = 0; j < distanceMatrix[i].size(); ++j) {
				if (distanceMatrix[i][j] < smallest) {
					smallest = distanceMatrix[i][j];
					col = j;
				}
			}
			smallestElements.push_back(pair<int, pair<size_t, size_t>>{ smallest, make_pair(i, col) });
		}

		sort(smallestElements.begin(), smallestElements.end());
		vector<pair<size_t, size_t>> sortedElems;
		for (const pair<int, pair<size_t, size_t>>& elem : smallestElements) {
			sortedElems.push_back(elem.second);
		}

		return sortedElems;
	}

	pair<float, float> findCentroid(const bboxCoords& objectBbox) {
		// Finds centroid up to 2 decimal places
		float centroidX = (objectBbox.x1 + objectBbox.x2) / 2;
		float centroidY = (objectBbox.y1 + objectBbox.y2) / 2;
		centroidX = round(centroidX * 100) / 100;
		centroidY = round(centroidY * 100) / 100;
		return make_pair(centroidX, centroidY);
	}

	void registerObject(const pair<float, float>& centroid, const bboxCoords& inputBboxCoord) {

		previousObjectPositions[nextIDToAssign] = objectAttrs{
		centroid, nextIDToAssign, 0, inputBboxCoord};

		nextIDToAssign++;
	}

	void deregisterObject(int objectID) {
		previousObjectPositions.erase(objectID);
	}

	const unordered_map<int, objectAttrs>& getPreviousPositions() {
		return previousObjectPositions;
	}

private:
	

	unordered_map<int, objectAttrs> previousObjectPositions; // ID , object
	int nextIDToAssign; //the next available ID that hasn't been assigned yet
	const int maxFramesVanished; // in frames
	const int maxDistanceChange; // in pixels
};



int main() {
	centroidTracker tracker(59, 59);
	vector<vector<int>> distanceMatrix = { {8, 9, 7, -2, 0},
										   {7, 2, 1, 7, 1},
										   {9, 10, -10, 6, 3},
		                                   {0, 4, 3, 5, 2}, 
		                                   {-9, 0, 9, 4, 2} };

}