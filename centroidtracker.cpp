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
	uint16_t x1, y1, x2, y2;
};

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
	uint8_t timeVanished;
	uint8_t objectID;
	pair<uint16_t, uint16_t> objectCenter;
	uint8_t confidence;
	uint8_t target;
} yolo_t;


class centroidTracker {
public:
	centroidTracker(const int maxFrames, const int maxDistance) : maxFramesVanished(maxFrames),
	maxDistanceChange(maxDistance), nextIDToAssign(0) {}
	

	unordered_map<int, yolo_t> updateObjects(vector<yolo_t>& inputObjects) {
		/*takes list of all yolo objects and then 
		updates their IDS and returns the new 
		updated list to the caller.*/

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
		for (yolo_t& inputObject : inputObjects) {
			
			inputObject.objectCenter = findCentroid(inputObject.x, inputObject.y, 
				inputObject.w, inputObject.h);
		}

		// if not tracking anything enter new objs
		if (previousObjectPositions.empty()) {
			for (const yolo_t& inputObject : inputObjects) {
				registerObject(inputObject);
			}
		}
		else {
			vector<int> prevObjectIDs;
			vector<pair<float, float>> prevObjectCentroids;
			for (const auto& entry : previousObjectPositions) {
				prevObjectIDs.push_back(entry.first);
				prevObjectCentroids.push_back(entry.second.objectCenter);
			}

			vector<vector<int>> distanceMatrix = calcDistanceMatrix(prevObjectCentroids, inputObjects);

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
				yolo_t& obj = previousObjectPositions[objID];
				obj.timeVanished = 0;
				obj.x = inputObjectsCentroids[location.second];
				obj.y
					obj.w
					obj.h
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
					yolo_t& obj = previousObjectPositions[objID];
					int timeVanished = ++obj.timeVanished;


					if( timeVanished > maxFramesVanished){
						deregisterObject(objID);
					}
				}
			}
			else {
				for (const int& col : unusedCols) {
					registerObject(inputObjects[col]);
				}

			}
		}

		return previousObjectPositions;
	}

private:
	
	void deregisterObject(int objectID) {
		previousObjectPositions.erase(objectID);
	}

	void registerObject(const yolo_t& inputObject) {
		// assigns the new object an ID 
		// currently this code assigns the new object in the 
		// previous position and updates teh ID, this porbably isnt neccesaey
		// to update the ID.
		previousObjectPositions[nextIDToAssign] = inputObject;
		previousObjectPositions[nextIDToAssign].objectID = nextIDToAssign;
		nextIDToAssign++;
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

	pair<uint16_t, uint16_t> findCentroid(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
		// Finds centroid of an object returns a pair
		// the calculation is based off line 177 of yolo.cpp 
		// the coordinates passed to teh bounding box from yolo_t
		// this calculation could be wrong if the yolo.cpp coordinates
		// arent what what they should be
		uint16_t x2 = (x - w) / 2;
		uint16_t y2 = (y - h) / 2;
		uint16_t centroidX = uint16_t((w + x2) / 2);
		uint16_t centroidY = uint16_t((h + y2) / 2);
		return make_pair(centroidX, centroidY);
	}

	unordered_map<int, yolo_t> previousObjectPositions; // ID , object
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
