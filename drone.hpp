#ifndef drone_hpp
#define drone_hpp

#include <stdio.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <vector>
#include <cmath>
#include <cstring>

using namespace std;

enum Mode {MST, FASTTSP, OPTTSP};

struct Vertex {
    int x;
    int y;
    bool wild;
    bool wall;
    bool safe;
    bool k;
    double d;
    size_t p;
    bool visited;
};


struct V {
    bool k;
    double d;
};

class Drone {
private:
    int gotopt;
    Mode mode;
    size_t SIZE;
    vector<Vertex> vertices;
    
    //MST
    double weightMST;

    // TSP
    vector<size_t> bestPath;
    double bestWeight;
    vector<size_t> temp;
    vector<size_t> currPath;
    double currDist;
    double startingEdge;
    double endingEdge;
    double mst_lb;
    double estimate;
    
    
    
public:
    // takes in command line arguments and sets parameters
    Drone(int argc, char * argv[]) {
        option longOpts[] = {
            { "mode", required_argument, nullptr, 'm' },
            { "help", no_argument, nullptr, 'h' },
            { nullptr, 0, nullptr, '\0' }
        };
        
        while((gotopt = getopt_long(argc, argv, "hm:" , longOpts, nullptr)) != -1) {
            switch (gotopt) {
                case 'h': {
                    cout << "Read the spec\n";
                    exit(0);
                }
                case 'm': {
                    if (!strcmp(optarg, "MST")) mode = MST;
                    else if (!strcmp(optarg, "FASTTSP")) mode = FASTTSP;
                    else if (!strcmp(optarg, "OPTTSP")) mode = OPTTSP;
                    else {
                        cout << "Error: Invalid mode\n";
                        exit(0);
                    }
                    break;
                }
            }
        }
        readInput();
        
    }

    // reads input and stores it in a vector of Vertex objects, then calls Part A, B or C
    void readInput() {
        cin >> SIZE;
        vertices.resize(SIZE);
        temp.resize(SIZE);
        currPath.resize(SIZE);
        bestPath.resize(SIZE);
        
        for (size_t count = 0; count < SIZE; count++) {
            int num1, num2;
            cin >> num1 >> num2;
            vertices[count].x = num1;
            vertices[count].y = num2;
            if (num1 < 0 && num2 < 0) {
                vertices[count].wild = true;
                vertices[count].wall = false;
                vertices[count].safe = false;
            }
            else if ((num1 == 0 && num2 < 0) || (num2 == 0 && num1 < 0) || (num2 == 0 && num1 == 0))  {
                vertices[count].wall = true;
                vertices[count].wild = false;
                vertices[count].safe = false;
            }
            else {
                vertices[count].wild = false;
                vertices[count].wall = false;
                vertices[count].safe = true;
            }
            vertices[count].visited = false;
            vertices[count].k = false;
            vertices[count].d = numeric_limits<double>::infinity();
        }
        switch(mode) {
            case MST: {
                createMST();
                break;
            }
            case FASTTSP: {
                fastTSP();
                break;
            }
            case OPTTSP: {
                optimalTSP();
                break;
            }
        }
        
    }
    
    
    // PART A -----------------------------------------------------------------
    
    //builds a minimum spanning tree on input data, while calculating the weight. If valid tree is built, calls printMST()
    void createMST() {
        weightMST = 0;
        vertices[0].d = 0.0;
        
        
        
        for (size_t count = 0; count < SIZE; count++) {

            // Step 1: from the set with k = false, select the vertex with the smallest d
            
            size_t currentVertex = 0;
            // find if there's any other false vertex with a lower d
            double currentDistance = numeric_limits<double>::infinity();
            for (size_t j = 0; j < SIZE; j++) {
                if (!vertices[j].k) {
                    if (vertices[j].d < currentDistance) {
                        currentVertex = j;
                        currentDistance = vertices[j].d;
                    }
                }
            }
            
            
            // Step 2: set k = true for v
            vertices[currentVertex].k = true;
            weightMST += vertices[currentVertex].d;
            
            // Step 3: for each vertex with an adjacent to v for which k of the adjacent is false, see if the recorded d is greater than the distance. If so, reset d and p
             
            for (size_t j = 0; j < SIZE; j++) {
                if (j != currentVertex) {
                    if (!vertices[j].k) {
                        double dist = getDistanceMST(currentVertex, j);

                        if ( dist < vertices[j].d) {
                            vertices[j].d = dist;
                            vertices[j].p = currentVertex;
                        }
                    }
                }
            }
        } // for count < size
        
        // check if a valid MST was created
        bool valid = 1;
        for (size_t count = 0; count < SIZE; count++) {
            if (!vertices[count].k) {
                cout << "Cannot construct MST\n";
                valid = 0;
            }
        }
        if (valid) {
            printMST();
        }
        
    }
    
    // helper function that returns the distance between two vertices a and b if an edge exists, if not, it returns inf
    double getDistanceMST(size_t a, size_t b) {
        Vertex first = vertices[a];
        Vertex second = vertices[b];
        double temp = 0.0;
        
        if (first.wild && second.safe) {
            return numeric_limits<double>::infinity();
        }
        else if (second.wild && first.safe) {
            return numeric_limits<double>::infinity();
        }
        else {
            double x = first.x - second.x;
            x = x*x;
            double y = first.y - second.y;
            y = y*y;
            temp = x + y;
            return sqrt(temp);
        }
    }
    
    // once MST is created, prints the resultant weight and edges
    void printMST() {
        cout << weightMST << "\n";
        for (size_t i = 1; i < SIZE; i++) {
            size_t p = vertices[i].p;
            if (p < i) {
                cout << p << " " << i << "\n";
            }
            else {
                cout << i << " " << p << "\n";
            }
        }
        
    }

    // PART B -----------------------------------------------------------------
    
    // calls 1) nearest neighbour, then 2) 2-opt, and then prints the result
    void fastTSP() {
        bestWeight = numeric_limits<double>::infinity();
        nearestNeighbor();
        twoOpt();
        cout << bestWeight << "\n";
        for (size_t i = 0; i < SIZE; i++) {
            cout << bestPath[i] << " ";
        }
        cout << "\n";
    }
    
    // first heuristic: nearest neighbor
    void nearestNeighbor() {
        bestWeight = numeric_limits<double>::infinity();
        size_t current = 0;
        bestPath[0] = 0;

        vertices[0].visited = true;
        for (size_t count = 1; count < SIZE; count++) {
            double currDist = numeric_limits<double>::infinity();
            for (size_t i = 0; i < SIZE; i++) {
                if (count != i && !vertices[i].visited) {
                    double dist = getDistanceTSP(count, i);
                    if (dist < currDist*currDist) {
                        current = i;
                        currDist = sqrt(dist);
                    }
                }
            }
            bestPath[count]= current;
            vertices[current].visited = true;
        }
        
    }
    
    // optimizes path created after Nearest neighbor by using 2-opt heuristics
    void twoOpt() {
        int improve = 0;
        while (improve < 2) {
            for (size_t i = 1; i < SIZE; i++) {
                for (size_t j = i+1; j < SIZE; j++) {
                    twoOptSwap(i, j);
                    double dist = distance();
                    if (dist < bestWeight) {
                        improve = 0;
                        bestWeight = dist;
                        swap(bestPath, temp);
                    }
                }
            }
            improve++;
        }
        
    }
    
    //helper for twoOPT()
    void twoOptSwap(size_t a, size_t b) {
        for (size_t i = 0; i < a; i++) {
            temp[i] = bestPath[i];
        }
        
        size_t count = a;
        for (size_t i = b; i > (a-1); i--) {
            temp[count] = bestPath[i];
            count++;
        }
        
        for (size_t i = b+1; i < SIZE; i++) {
            temp[i] = bestPath[i];
        }
        
    }
    
    // helper for twoOpt(), tells us the distance of the route provided
    double distance() {
        double weight = 0.0;
        for (size_t i = 1; i < SIZE; i++) weight += sqrt(getDistanceTSP(temp[i-1], temp[i]));
        weight += sqrt(getDistanceTSP(temp[0], temp[SIZE-1]));
        return weight;
        
    }
    
    
    // PART C -----------------------------------------------------------------
    
    //calls fastTSP and genPerms;
    void optimalTSP() {
        currDist = 0.0;
        startingEdge = numeric_limits<double>::infinity();
        endingEdge = numeric_limits<double>::infinity();
        estimate = 0.0;
        mst_lb = 0.0;
        nearestNeighbor();
        twoOpt();
        currPath = bestPath;
        
        genPerms(1);
        cout << bestWeight << "\n";
        for (size_t i = 0; i < SIZE; i++) {
            cout << bestPath[i] << " ";
        }
        cout << "\n";
        
    }
    
    //finds that optimal path, calls promising
    void genPerms(size_t permLength) {
        if (permLength == currPath.size()) {
            double lastEdge = sqrt(getDistanceTSP(0, currPath.back()));
            currDist += lastEdge;
            if (currDist < bestWeight) {
                bestWeight = currDist;
                bestPath = currPath;
            }
            currDist -= lastEdge;
            return;
        }

        if (!promising(permLength)) return;

        for (size_t i = permLength; i < currPath.size(); ++i) {
            swap(currPath[permLength], currPath[i]);
            double newEdge = sqrt(getDistanceTSP(currPath[permLength-1], currPath[permLength]));
            currDist += newEdge;
            genPerms(permLength + 1);
            currDist -= newEdge;
            swap(currPath[permLength], currPath[i]);
        }
        
    }
    
    //true if the lower bound of some proposed path is acceptable, called lowerBound()
    bool promising(size_t permLength) {
        
        double est = lowerbound(permLength);
        bool promise = (est < bestWeight);

        if (SIZE-permLength < 5) return true;
        if (!promise) return false;
        return true;
        
    }
    
    // returns the lowerBound of the estimated cost of this path by adding the currentCost, MST weight of unconnected points, and the edges that connect the two
    double lowerbound(size_t permLength) {
        auto begin = currPath.begin() + static_cast<int>(permLength);
        auto end = currPath.end();
        mst_lb = MSTforTSP(begin, end, permLength);
        estimate = currDist + mst_lb;
        if (estimate >= bestWeight) return estimate;
        
        
        size_t first = currPath[0];
        size_t last;
        if (permLength > 0) last = currPath[permLength-1];
        else last = 0;
        
        
        startingEdge = numeric_limits<double>::infinity();
        endingEdge = numeric_limits<double>::infinity();
        
        
        for (size_t i = permLength; i < SIZE; i++) {
            double distStart = getDistanceTSP(currPath[i], first);
            if (distStart < (startingEdge*startingEdge)) startingEdge = sqrt(distStart);
            double distEnd = getDistanceTSP(currPath[i], last);
            if (distEnd < (endingEdge*endingEdge)) endingEdge = sqrt(distEnd);
        }
        estimate += startingEdge;
        estimate += endingEdge;
        
        return estimate;
        
    }
    
    // called by lowerBound() to predicts the weight of the MST of the unconnected vertices
    double MSTforTSP(vector<size_t>::iterator begin, vector<size_t>::iterator end, size_t permLength) {
        size_t size = static_cast<size_t>(std::distance(begin, end));
        vector<V> table(size);
        for (size_t i = 0; i < size; i++) {
            table[i].k = false;
            table[i].d = numeric_limits<double>::infinity();
        }
        
        mst_lb = 0.0;
        table[0].d = 0.0;
        
        for (size_t count = 0; count < size; count++) {
            
            size_t currIndex = 0;
            double currWeight = numeric_limits<double>::infinity();
            for (size_t j = 0; j < size; j++) {
                if (!table[j].k) {
                    if (table[j].d < currWeight) {
                        currIndex = j;
                        currWeight = table[j].d;
                    }
                }
            }
            
            table[currIndex].k = true;
            mst_lb += table[currIndex].d;
            
            for (size_t j = 0; j < size; j++) {
                if (j!=currIndex) {
                    if (!table[j].k) {
                        size_t first = currPath[permLength + currIndex];
                        size_t second = currPath[permLength + j];
                        double dist = getDistanceTSP(first, second);
                        if (dist < (table[j].d*table[j].d)) {
                            table[j].d = sqrt(dist);
                        }
                    }

                }
            }
        }
        return mst_lb;
        
    }
    
    
    // helper function for parts B and C that gives you the distance between any two vertices from a complete graph
    double getDistanceTSP(size_t a, size_t b) {
        double x = vertices[a].x - vertices[b].x;
        x = x*x;
        double y = vertices[a].y - vertices[b].y;
        y = y*y;
        return x + y;
        
    }
    
    
    
    
    

    
};


#endif /* drone_hpp */
