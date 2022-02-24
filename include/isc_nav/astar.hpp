// MIT License
//
// Copyright (c) Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// TODO a* go here

// get ocuppancy grid, current location, goal location

#include <stdlib.h>
#include <vector>
#include <queue>
using namespace std;

// coordinates on occupancy grid
struct coordinate_pair
{
    int x;
    int y;
};

// holds path that leads to latest coordinate position
// along with g(n), h(n), and f(n)
struct node
{
    int g_n;
    int h_n;
    int f_n;
    vector<coordinate_pair> path;

    bool operator==(const node &a) const
    {
        return a.f_n == f_n;
    }

    bool operator<(const node &a) const
    {
        return f_n < a.f_n;
    }

    bool operator>(const node &a) const
    {
        return f_n > a.f_n;
    }

    bool operator!=(const node &a) const
    {
        return f_n != a.f_n;
    }

    bool operator<=(const node &a) const
    {
        return f_n <= a.f_n;
    }

    bool operator>=(const node &a) const
    {
        return f_n >= a.f_n;
    }
};

// given goal location and current robot location
coordinate_pair goal_location;
coordinate_pair robot_location;

// given occupancy grid
// assumes that 1's indicate obstacles
vector<vector<int>> occupancy_grid;

// paths being explored
priority_queue<node *> frontier;

// finds the h(n) using the Manhattan Distance
int findHn(coordinate_pair current_location, coordinate_pair goal_location)
{

    return abs(goal_location.x - current_location.x) + abs(goal_location.y - current_location.y);
};

node *makeNewNode(int direction_shift, node *expanding_node)
{

    // creates a new node and copies over g(n) and current path
    node *expansion = new node;
    expansion->path = expanding_node->path;
    expansion->g_n = expanding_node->g_n;
    expansion->path.push_back(expanding_node->path.back());

    // for every direction shift:
    //       checks that direction shift is within occupancy grid
    //       checks if direction shift is into obstacle

    // right shift
    if (direction_shift == 0 && occupancy_grid.size() > expanding_node->path.back().x + 1 && occupancy_grid.at(expanding_node->path.back().x + 1).at(expanding_node->path.back().y) != 1)
    {

        expansion->path.back().x++;
    }

    // left shift
    else if (direction_shift == 1 && expanding_node->path.back().x != 0 && occupancy_grid.at(expanding_node->path.back().x - 1).at(expanding_node->path.back().y) != 1)
    {

        expansion->path.back().x--;
    }

    // forward shift
    else if (direction_shift == 2 && occupancy_grid.size() > expanding_node->path.back().y + 1 && occupancy_grid.at(expanding_node->path.back().x).at(expanding_node->path.back().y + 1) != 1)
    {

        expansion->path.back().y++;
    }

    // backward shift
    else if (direction_shift == 3 && expanding_node->path.back().x != 0 && occupancy_grid.at(expanding_node->path.back().x).at(expanding_node->path.back().y - 1) != 1)
    {

        expansion->path.back().y--;
    }

    else
        return NULL;

    // updates g(n), h(n), and f(n)
    expansion->h_n = findHn(expansion->path.back(), goal_location);
    expansion->g_n++;
    expansion->f_n = expansion->g_n + expansion->f_n;

    // returns newly created node
    return expansion;
};

void expandNode()
{

    // takes the starting location as the first node
    node *expanded_node = frontier.top();
    frontier.pop();

    while (expanded_node->h_n != 0)
    {

        // expands the new node in all valid directions
        // then places it in the correct location in the frontier priority queue
        for (int i = 0; i < 4; i++)
        {
            node *child = makeNewNode(i, expanded_node);
            if (child != NULL)
            {
                frontier.push(child);
            }
        }

        // deletes expanded node
        expanded_node = NULL;
        delete expanded_node;

        // sets expanded node the next node in the priority queue
        expanded_node = frontier.top();
        frontier.pop();
    }
}

// int main(vector<vector<int>> occupancy_grid, int* bot_loc, int* goal_loc){

int main()
{

    // creates mock occupancy grid for testing
    for (int i = 0; i < 10; i++)
    {
        vector<int> new_row;
        for (int j = 0; j < 10; j++)
        {

            if (j % 3 == 0)
            {
                new_row.push_back(1);
            }
            else
            {
                new_row.push_back(0);
            }
        }

        occupancy_grid.push_back(new_row);
    }

    // creates the coordinates for the robot location and goal location
    robot_location.x = 9;
    robot_location.y = 9;

    goal_location.x = 0;
    goal_location.y = 0;

    // creates the node for the starting location
    node *start = new node;
    start->g_n = 0;
    start->h_n = findHn(robot_location, goal_location);
    start->path.push_back(robot_location);
    frontier.push(start);

    // starts searching for the solution
    expandNode();

    return 0;
}