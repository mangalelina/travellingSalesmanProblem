# Traveling Salesmen Problem

This project implements a solution to the [Travelling Salesmen Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem) in three parts. 

## Part 1: Minimum Spanning Tree

A minimum spanning tree (MST) is the set of all edges in a graph that have the least possible weight whilst also connecting all the vertices. This is solved using Prim's Algorithm. Alternatively, if the input is given as a sparse graph, Kruskal's algorithm can also be used to find the MST. 

## Part 2: Optimizations on MST

The solution makes use of two of the many heuristics used to approximate the optimal route for the traveling salesman problem. Firstly the [Nearest Neighbor](https://en.wikipedia.org/wiki/Nearest_neighbour_algorithm) heuristic is implemented. It begins at an arbitrary vertex and chooses neighbor that has the least weight but is unvisited until all edges have been included. Secondly, the [2-opt](https://en.wikipedia.org/wiki/2-opt) heuristic is implemented. This swaps edges to connect certain edges in a different order to minimize the weight. A helpful visualization can be found [here](https://github.com/adavis-85/Traveling-Salesman-2-opt-with-Visualization). 

## Part 3: Backtracking

This solves the Travelling Salesman Problem using backtracking. It determines if something is promising by adding the current cost to the MST of the remaining vertices and connecting edges. 


Using any material from here is against the University of Michigan honor code.

