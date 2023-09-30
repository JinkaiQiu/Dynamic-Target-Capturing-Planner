/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
#include <unordered_map>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct Node {
    // open list node
    int x, y, t;
    int g, h, f;
    Node* parent; 
    Node(int x, int y, int t, int g, int h, int f, Node* parent)
      : x(x), y(y), t(t), g(g), h(h), f(f), parent(parent) {}
};

struct CompareNode {
    // priority queue comparator
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

struct hash_tuple {
    // Hash function for tuple for g values
    template <class T1, class T2, class T3>
    size_t operator() (const std::tuple<T1, T2, T3>& p) const {
        size_t seed = 0;
        const size_t prime1 = 73856093;
        const size_t prime2 = 19349669;
        const size_t prime3 = 83492791;
        auto h1 = std::hash<T1>{}(std::get<0>(p));
        auto h2 = std::hash<T2>{}(std::get<1>(p));
        auto h3 = std::hash<T3>{}(std::get<2>(p));
        seed ^= h1 * prime1;
        seed ^= h2 * prime2;
        seed ^= h3 * prime3;

        return seed;
    }
};

void saveToImage(const std::vector<std::vector<int>>& heuristic_g_table, const char* filename) {
    // Save the heuristic g table to image
    int x_size = heuristic_g_table.size();
    int y_size = heuristic_g_table[0].size();
    int min_val = 0;
    int max_val = 0;
    for (int y = 0; y < y_size; ++y) {
        for (int x = 0; x < x_size; ++x) {
            int val = heuristic_g_table[x][y];
            if (val > max_val && val != std::numeric_limits<int>::max()) {
                max_val = val;
            }
        }
    }
    std::vector<uint8_t> image_data(x_size * y_size);
    for (int y = 0; y < y_size; ++y) {
        for (int x = 0; x < x_size; ++x) {
            int val = heuristic_g_table[x][y];
            if (val == std::numeric_limits<int>::max()) {
                val = max_val;
            }
            uint8_t normalized_val = static_cast<uint8_t>(((val - min_val) / static_cast<float>(max_val - min_val)) * 255);
            image_data[y * x_size + x] = normalized_val;
        }
    }
    stbi_write_png(filename, x_size, y_size, 1, image_data.data(), x_size);
}

int heuristic (int initposeX, int initposeY, int posX, int posY, int* target_traj, int target_steps){
    // Looking forward heuristic to estimate distance to target
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    int robotMoved = abs(initposeX - posX) + abs(initposeY - posY);
    int distanceEstimate;
    if (robotMoved >= target_steps) {
        distanceEstimate = sqrt((posX - goalposeX)*(posX - goalposeX) + (posY - goalposeY)*(posY - goalposeY));
    }
    else {
        int targetFutureX = target_traj[robotMoved-1];
        int targetFutureY = target_traj[robotMoved-1+target_steps];
        distanceEstimate = sqrt((posX - targetFutureX)*(posX - targetFutureX) + (posY - targetFutureY)*(posY - targetFutureY));
    }
    return distanceEstimate;
}

int heuristic_simpleDistance(int initposeX, int initposeY, int finalposeX, int finalposeY){
    // Simple distance heuristic
    int distanceEstimate = sqrt((initposeX - finalposeX)*(initposeX - finalposeX) + (initposeY - finalposeY)*(initposeY - finalposeY));
    return distanceEstimate;
}

std::vector<std::vector<int>> heristicAstar(int initposeX, int initposeY, int targetFutureGoalX, int targetFutureGoalY, int x_size, int y_size, int* map, int threshold){
    // Using A* to compute the heuristic g table
    // NOT used in the final version
    // Shows inferior performance compared to distance looking forward heuristic calculation*
    int INF = std::numeric_limits<int>::max();
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
    std::vector<std::vector<bool>> closedList(x_size, std::vector<bool>(y_size, false));
    std::vector<std::vector<int>> heuristic_g_table(x_size, std::vector<int>(y_size, INF));
    heuristic_g_table[targetFutureGoalX - 1][targetFutureGoalY - 1] = 0;

    int h = heuristic_simpleDistance(initposeX, initposeY, targetFutureGoalX, targetFutureGoalY);
    Node* start = new Node(targetFutureGoalX, targetFutureGoalY, 0, 0, h, 0, nullptr);
    start->f = start->g + start->h;
    openList.push(start);
    // std::cout << "Starting Node Set" << std::endl;
    Node* current = nullptr;
    while (!openList.empty()){
        current = openList.top();
        openList.pop();
        closedList[current->x - 1][current->y - 1] = true;
        for (int dir = 0; dir < NUMOFDIRS; ++dir){
            int newX = current->x + dX[dir];
            int newY = current->y + dY[dir];
            if (newX < 1 || newY < 1 || newX > x_size || newY > y_size){
                continue;
            }
            int cost = map[GETMAPINDEX(newX, newY, x_size, y_size)];
            if (cost >= threshold || closedList[newX - 1][newY - 1]) {
                continue;   
            }
            if (cost == 0) {
                cost = 1;
            }
            int h = heuristic_simpleDistance(newX, newY, initposeX, initposeY);
            int newG = cost + heuristic_g_table[current->x - 1][current->y - 1];
            if (newG < heuristic_g_table[newX - 1][newY - 1]) {              
                heuristic_g_table[newX - 1][newY - 1] = newG;
                Node* successor = new Node(newX, newY, 0, newG, h, newG + h, current);                    
                openList.push(successor);
            }
        }
    }

    return heuristic_g_table;
}

std::vector<std::vector<int>> heristicBackward(int initposeX, int initposeY, int* target_traj, int target_steps, int x_size, int y_size, int* map, int threshold){
    // Using A* to compute the heuristic g table
    // NOT used in the final version
    // Shows inferior performance compared to distance looking forward heuristic calculation
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    std::vector<std::vector<int>> heuristic_g_table_goal = heristicAstar(initposeX, initposeY, goalposeX, goalposeY, x_size, y_size, map, threshold);
    saveToImage(heuristic_g_table_goal, "heuristic_g_table_goal.png");
    return heuristic_g_table_goal;
}

std::vector<Node*> A_star(int initposeX, int initposeY, int targetFutureGoalX, int targetFutureGoalY, int x_size, int y_size, int* map, int collision_thresh, bool ignoreCost = false){
    // implementation of A* search in 2D
    int INF = std::numeric_limits<int>::max();
    
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    std::vector<Node*> fullPath;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
    std::vector<std::vector<bool>> closedList(x_size, std::vector<bool>(y_size, false));
    std::vector<std::vector<int>> g_table(x_size, std::vector<int>(y_size, INF));
    g_table[initposeX - 1][initposeY - 1] = 0;
    
    int h = heuristic_simpleDistance(initposeX, initposeY, targetFutureGoalX, targetFutureGoalY);
    Node* start = new Node(initposeX, initposeY, 0, 0, h, 0, nullptr);
    start->f = start->g + start->h;
    openList.push(start);
    Node* current = nullptr;
    while (!openList.empty()){
        current = openList.top();
        openList.pop();
        if (current->x == targetFutureGoalX && current->y == targetFutureGoalY){
            break;
        }
        closedList[current->x - 1][current->y - 1] = true;
        for (int dir = 0; dir < NUMOFDIRS; ++dir){
            int newX = current->x + dX[dir];
            int newY = current->y + dY[dir];
            if (newX < 1 || newY < 1 || newX > x_size || newY > y_size){
                continue;
            }
            int cost = map[GETMAPINDEX(newX, newY, x_size, y_size)];
            if (cost >= collision_thresh || closedList[newX - 1][newY - 1]) {
                continue;   
            }
            if (cost == 0 || ignoreCost) {
                cost = 1;
            }  

            int h = heuristic_simpleDistance(newX, newY, targetFutureGoalX, targetFutureGoalY);
            int newG = cost + g_table[current->x - 1][current->y - 1];
            if (newG < g_table[newX - 1][newY - 1]) {              
                g_table[newX - 1][newY - 1] = newG;
                Node* successor = new Node(newX, newY, 0, newG, h, newG + 10*h, current);                    
                openList.push(successor);
            }
        }
    }

    while (current != nullptr){
        fullPath.push_back(current);
        current = current->parent;
    }
    std::reverse(fullPath.begin(), fullPath.end());   
    return fullPath;
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{

    // Static variables to store the full path and the current index
    static std::vector<Node*> fullPath;
    static std::vector<Node*> fullPath_backup;
    static int current_index;    
    static bool useVec = false;
    // toggle between using vector or hash table for g values depending on map size
    // using vector is faster but requires more memory
    if (target_steps >  1000 || x_size > 1000 || y_size > 1000) {
        useVec = false;
    }
    else {
        useVec = true;
    }
    const int INF = std::numeric_limits<int>::max();
    // 8-connected grid
    const int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    const int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int goalposeX = target_traj[target_steps-1];
    int goalposeY = target_traj[target_steps-1+target_steps];
    int closedListCount = 0;
    int openListCount = 0;
    int ReachTOffset = int(0.05*target_steps);
    bool targetFound;
    std::vector<std::vector<int>> h_table;
    static std::vector<std::vector<std::vector<int> > > g_table;

    if (fullPath.empty()){ // if the path is empty, we need to plan, run only once
        std::cout << "Planning!" << std::endl;
        current_index = 1;
        targetFound = false;
        // initialize the open list / closed list
        static std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;     
        std::cout << "Initialized open list" << std::endl;
        static std::vector<std::vector<std::vector<bool> > > closedList(x_size, std::vector<std::vector<bool> >(y_size, std::vector<bool>(target_steps, false)));
        std::cout << "Initialized closed lists" << std::endl;
        // initialize the trajectory 
        static std::vector<std::vector<std::vector<bool> > > trajectory(x_size, std::vector<std::vector<bool> >(y_size, std::vector<bool>(target_steps , false)));
        for (int i = 0; i < target_steps; ++i) {
            trajectory[target_traj[i] - 1][target_traj[i + target_steps] - 1][i] = true;
        }
        std::cout << "Initialized trajectory" << std::endl;
        // initialize g table 
        static std::unordered_map<std::tuple<int, int, int>, int, hash_tuple> g_table_map;
        auto key = std::make_tuple(robotposeX - 1, robotposeY - 1, 0);
        g_table_map[key] = 0;
        if (useVec) {
            g_table.resize(x_size, std::vector<std::vector<int> >(y_size, std::vector<int>(target_steps, INF)));
            g_table[robotposeX - 1][robotposeY - 1][0] = 0;
            std::cout << "Using Vector, Initialized g table" << std::endl;
        }
        // put the starting node on the open list
        int h = heuristic (robotposeX, robotposeY, robotposeX, robotposeY, target_traj, target_steps);
        Node* start = new Node(robotposeX, robotposeY, curr_time, 0, h, 0, nullptr);
        start->f = start->g + start->h;
        openList.push(start);
        std::cout << "Starting Node Set" << std::endl;
        Node* current = nullptr;

        // compute a backup path to goal by directly heading to goal position
        fullPath_backup = A_star(robotposeX, robotposeY, goalposeX, goalposeY, x_size, y_size, map, collision_thresh);
        int backup_len = fullPath_backup.size();
        int timeOut = target_steps - backup_len;
        // prevent timeout from being too large 
        if (timeOut > 1000){timeOut = 1000;}
        if (timeOut < 0) {
            std::cout << "WARNING: NO feasible backup found with A* to goal" << std::endl;
            timeOut = INF;
            // second backup, ignore cost except collision, attempt to find a shorter path
            fullPath_backup = A_star(robotposeX, robotposeY, goalposeX, goalposeY, x_size, y_size, map, collision_thresh, true);
            backup_len = fullPath_backup.size();
            timeOut = target_steps - backup_len;
            if (timeOut > 1000){timeOut = 1000;}
            if (timeOut < 0){
                std::cout << "WARNING: NO feasible backup found with A* to goal, ignore cost" << std::endl;
                timeOut = INF;
            }
            else{
                std::cout << "Backup path found, ignoring cost, with time out: " << timeOut << std::endl;
            }
            }
        else{
            std::cout << "Backup path found with time out: " << timeOut << std::endl;
        }
        
        // start STA*
        std::cout << "Starting search" << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        while (!openList.empty()){
            current = openList.top();
            auto end_time = std::chrono::high_resolution_clock::now();
            for (int offset = ReachTOffset; offset <= 1.5*ReachTOffset; ++offset) {
                if (current->t + offset < target_steps && trajectory[current->x - 1][current->y - 1][current->t + offset]) {
                    std::cout << "========== Found target! =======" << std::endl;
                    std::cout << "Target @: " << current->x << ", " << current->y << ", Offset: " << offset << std::endl;
                     std::cout << "===============================" << std::endl;
                    targetFound = true;
                    break;
                }
            }
            if (targetFound) {
                break;
            }
            if (end_time - start_time > std::chrono::seconds(timeOut)) {
                std::cout << "Time out!" << std::endl;
                break;
            }
            openList.pop(); // pop smallest f from openList
            closedList[current->x - 1][current->y - 1][current->t] = true; // add to closedList
            closedListCount++;            
            if (closedListCount % 1000000 == 0) {
                std::cout << "Closed list count: " << closedListCount/1000000 <<  "e6" << std::endl;
            }
            for (int dir = 0; dir < NUMOFDIRS; ++dir){
                int newX = current->x + dX[dir];
                int newY = current->y + dY[dir];
                int newT = current->t + 1;
                if (newX < 1 || newY < 1 || newX > x_size || newY > y_size || newT > target_steps){
                    continue;
                }
                int cost = map[GETMAPINDEX(newX, newY, x_size, y_size)];
                if (cost >= collision_thresh || closedList[newX - 1][newY - 1][newT]) {
                    continue;   
                }
                if (cost == 0) {
                    cost = 1;
                }
                int h = heuristic (robotposeX, robotposeY, newX, newY, target_traj, target_steps);
                if (useVec){
                    int newG = cost + g_table[current->x - 1][current->y - 1][current->t];
                    if (newG < g_table[newX - 1][newY - 1][newT]) {   
                        g_table[newX - 1][newY - 1][newT] = newG;
                        Node* successor = new Node(newX, newY, newT, newG, h, newG + h, current);                    
                        openList.push(successor);
                        openListCount++;
                    }                    
                }
                else {
                    auto key = std::make_tuple(newX - 1, newY - 1, newT);
                    auto key_current = std::make_tuple(current->x - 1, current->y - 1, current->t); 
                    int newG = cost + g_table_map[key_current]; 
                    auto it = g_table_map.find(key);
                    if (it == g_table_map.end() || newG < it->second) {
                        g_table_map[key] = newG;
                        Node* successor = new Node(newX, newY, newT, newG, h, newG + 10*h, current);
                        openList.push(successor);
                        openListCount++;
                    }
                }
            }
            if (openListCount % 1000000 == 0) {
                std::cout << "Open list count: " << openListCount/1000000 <<  "e6" << std::endl;
            } 
        }
        // If target is found within time out, backtrace the path
        if (targetFound){
            while (current != nullptr){
                fullPath.push_back(current);
                current = current->parent;
            }
            std::cout << "Backtracking" << std::endl;
            std::reverse(fullPath.begin(), fullPath.end());
            std::cout << "PATH Retrieved" << std::endl;
        }
        // If target is not found, use the backup path
        else {
            std::cout << "No path found in STA*" << std::endl;
            fullPath = fullPath_backup;
        }

    }

    // get the next action
    if (current_index == fullPath.size()) {
        std::cout << "___Path Complete!!! ____" << std::endl;
        current_index++;
        return;
    }
    else if (current_index > fullPath.size()){
        std::cout << "Time is:        " << curr_time << std::endl;
        std::cout << "Target @:       " << targetposeX << ", " << targetposeY << std::endl;
        std::cout << "Robot @:        " << robotposeX << ", " << robotposeY << std::endl;
        std::cout << "________________________" << std::endl;
        if (!targetFound){ 
            // Backtrace target Trajectory when robot moved to the last target position first IF running backup path
            if (heuristic_simpleDistance(robotposeX, robotposeY, targetposeX, targetposeY) < 5) {
                std::cout << "Target reached!" << std::endl;
                return;
            }
            action_ptr[0] = target_traj[target_steps - 1 - current_index + fullPath.size()];
            action_ptr[1] = target_traj[target_steps - 1 + target_steps - current_index + fullPath.size()];
        }
        current_index++;
        return;
    }
    
    std::cout << "Time is:        " << curr_time << std::endl;
    std::cout << "Target @:       " << targetposeX << ", " << targetposeY << std::endl;
    std::cout << "Robot @:        " << robotposeX << ", " << robotposeY << std::endl;
    std::cout << "Next action is: " << fullPath[current_index]->x << ", " << fullPath[current_index]->y << std::endl;
    std::cout << "________________________" << std::endl;
    
    action_ptr[0] = fullPath[current_index]->x;
    action_ptr[1] = fullPath[current_index]->y;
    current_index++;
    
    return;
}