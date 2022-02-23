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


//get ocuppancy grid, current location, goal location

#include <stdlib.h>
#include <vector>
using namespace std;



struct pair{
    int x;
    int y;
};

struct node{
    pair coordinates;
    int g_n;
    int h_n;
    int f_n; 

};


pair goal_location;
pair robot_location;

vector<vector<int>> occupancy_grid;


vector<node*> frontier;

vector<node*> closed_list;



//finds the h(n) using the Manhattan Distance 
 int findHn(int *current_location, int *goal_location){

    return abs(goal_location[0]-current_location[0]) + abs(goal_location[0]-current_location[0]);               

 };

node* makeNewNode(int direction_shift, node* expanding_node){

        node *expansion = new node;

        expansion -> coordinates = expanding_node -> coordinates;
        expansion -> g_n = expanding_node -> g_n;

        //right shift
        if(direction_shift == 0){
            expansion -> coordinates.x+1;
            expansion->g_n++; 
        }

        //left shift
        else if(direction_shift == 1){
            expansion -> coordinates.x-1;
            expansion->g_n++;
        }

        //forward shift
        else if(direction_shift == 2){
            expansion -> coordinates.y+1;
            expansion->g_n++;
        }

        //backward shift
        else if(direction_shift == 3){
            expansion -> coordinates.y-1;
            expansion->g_n++;
        }

        else
            return NULL;

        
        expansion -> h_n = findHn(expansion->coordinates, goal_location);
        expansion -> f_n = expansion-> g_n + expansion -> f_n;   

};

//places a newly created node into a priority queue using a binary search
void placeInPQ(node *enqueue){

    int low = 0;
    int high = frontier.size()-1;
    int mid = 0;
 
    while(low<=high){
 
        mid = (high+low)/2;
 
        if(frontier[mid] -> f_n < enqueue-> f_n){
            low = mid+1;
        }
 
        else if(frontier[mid] -> f_n > enqueue -> f_n){
            high = mid-1;

        }
 
        else
            frontier.insert(mid, enqueue);

    }

    if(frontier.size()==0){
        frontier.push_back(enqueue);
    }
 
};

void expandNode(){

    node *expanded_node = frontier.front();
    frontier.erase(frontier.begin());

    while(expanded_node->h_n != 0){
 
        //adds the node to the explored set hash table
 
        //expands the new node in all valid directions
        //then places it in the correct location in the frontier priority queue
        for(int i = 0; i<4; i++){
            node * child = makeNewNode(i,expanded_node);
            if (child != NULL){
                placeInPQ(child);
            }

        } 
        //selects the next node to be expanded and prints it
        
        expanded_node = frontier.front();
        frontier.erase(frontier.begin());

    }


}


int main(){

node * start = new node;
start->g_n = 0;
start->h_n = findHn(robot_location, goal_location);
frontier.push_back(start);
 
 
//starts searching for the solution
expandNode();

return 0;
}