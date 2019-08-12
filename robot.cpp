/*
  * Author: Ramadan Jaafari
  * purpose: This program applys A* algorithm to find the
  *     optimal path for a robot to move from the starting
  *     position(S) to the Goal position(G), this is represented
  *     by a grid in the program.
  * notes: a user may enter an argument, "order", to output the
  *     order in which the nodes/positions were expanded.
*/

#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include "pos.cpp"

using namespace std;

//CONSTANTS
#define GOAL_ROW 2      //The goal position
#define GOAL_COLUMN 2   //The goal position
#define START_ROW 5     //The starting position
#define START_COLUMN 3  //The starting position
#define START_G 0       //The distance between the start node and itself
#define START -1        //represents the start
#define GOAL -2         //represents the Goal
#define FORBIDDEN -3    //represents forbidden
#define INOPEN -4       //represents positions in open queue
#define INCLOSED -4     //represents positions in closed array/list
#define UP -5           //
#define DOWN -6         //represent arrows on the grid
#define LEFT -7         //leading to the goal
#define RIGHT -8        //

//FLAG, prints the path to the Goal if set to 0, prints the order
// in which the nodes/positions are checked/expanded if set to 1
int ORDER;

stack<Position> pathSearch(int grid[8][8]);
int f(int row, int column, int g);
int h(int row, int column);
bool inOpen(priority_queue<Position, vector<Position>, CompareF> open, Position p);
void showGrid(int grid[8][8]);
string getLabel(int value);
stack<Position> path(Position goal, Position positions[], int n);
void outputPath(stack<Position> solPath, int grid[8][8]);

int main(int argc, char *argv[])
{
  //check arguments
  if(argc < 2)
    ORDER = 0;
  else if(argc > 2)
  {
    cout << "Too many arguements, enter \"order\" as an argument or enter no argument" << endl;
    exit(1);
  }else if (argc == 2)
  {
    string order = argv[1];
    if(order == "order")
      ORDER = 1;
    else
      {
        cout << "Invalid argument, enter \"order\" as an argument or enter no argument" << endl;
        exit(1);
      }
  }
  

  //defining & initializing the grid
  stack<Position> solutionPath;
  int grid[8][8];

  for(int i = 0; i < 8; i++)
    for(int j = 0; j < 8; j++)
      grid[i][j] = 0;

  grid[START_ROW][START_COLUMN] = START;
  grid[GOAL_ROW][GOAL_COLUMN] = GOAL;
  
  //setting the forbidden positions
  grid[3][2] = FORBIDDEN;
  grid[3][3] = FORBIDDEN;
  grid[3][4] = FORBIDDEN;
  grid[3][5] = FORBIDDEN;
  grid[3][6] = FORBIDDEN;
  grid[4][1] = FORBIDDEN;
  grid[5][2] = FORBIDDEN;
  grid[5][6] = FORBIDDEN;
  //grid[4][0] = FORBIDDEN;
  //grid[3][7] = FORBIDDEN;

  //start searching for the path & 
  solutionPath = pathSearch(grid);

  //print the path
  if(!solutionPath.empty())
    outputPath(solutionPath, grid);
  else
  {
    system("clear");
    printf("the path to the goal was not found\n");
  }
  

  return 0;
}

//searches for the goal using A* alg.
stack<Position> pathSearch(int grid[8][8])
{
  int checkedNodes = 0;
  priority_queue<Position, vector<Position>, CompareF> Open;
  Position Closed[55];

  Open.push(Position(START_ROW, START_COLUMN, START_G, f(START_ROW, START_COLUMN, START_G), -1, -1));
  while(!Open.empty())
  { 
    //save node & remove it from Open
    Position p = Open.top();
    Open.pop();
    //Position Closed[40]; //I did not use closed list, I labeled the checked positions instead
    
    //label when checked on the grid
    if(checkedNodes && grid[p.row][p.column] != GOAL)
      if(!ORDER)
        grid[p.row][p.column] = INCLOSED;
      else
        grid[p.row][p.column] = checkedNodes;

    //check if p is the goal
    if(p.row == GOAL_ROW && p.column == GOAL_COLUMN)
      return path(p, Closed, checkedNodes); //should return the path <<<
    
    //generate children
    if(grid[p.row + 1][p.column] == 0 || grid[p.row + 1][p.column] == GOAL)
    {
      Open.push(Position(p.row + 1, p.column, p.g + 1, f(p.row + 1, p.column, p.g + 1), p.row, p.column));
      if (grid[p.row + 1][p.column] != GOAL)
        grid[p.row + 1][p.column] = -4;
    }
    if(grid[p.row][p.column + 1] == 0 || grid[p.row][p.column + 1] == GOAL)
    {
      Open.push(Position(p.row, p.column + 1, p.g + 1, f(p.row, p.column + 1, p.g + 1), p.row, p.column));
      if (grid[p.row][p.column + 1] != GOAL)
        grid[p.row][p.column + 1] = -4;
    }
    if(grid[p.row - 1][p.column] == 0 || grid[p.row - 1][p.column] == GOAL)
    {
      Open.push(Position(p.row - 1, p.column, p.g + 1, f(p.row - 1, p.column, p.g + 1), p.row, p.column));
      if (grid[p.row - 1][p.column] != GOAL)
        grid[p.row - 1][p.column] = -4;
    }
    if(grid[p.row][p.column - 1] == 0 || grid[p.row][p.column - 1] == GOAL)
    {
      Open.push(Position(p.row, p.column - 1, p.g + 1, f(p.row, p.column - 1, p.g + 1), p.row, p.column));
      if (grid[p.row][p.column - 1] != GOAL)
        grid[p.row][p.column - 1] = -4;
    }
    
    //add p to closed
    Closed[checkedNodes] = p;
    checkedNodes += 1;

    //print grid
    showGrid(grid);
  }
  stack<Position> garbage;
  return garbage;
}

//f*(n) = g*(n) + h*(n), Admissible Heurestic Evaluation function
int f(int row, int column, int g)
{
  return g + h(row, column);
}

//h*(n): The heuristic function, estimates the distance to the goal
int h(int row, int column)
{
  int h = 0;
  h += abs(GOAL_ROW - row);
  h += abs(GOAL_COLUMN - column);
  
  return h;
}

//Clears the Screen and prints the grid
void showGrid(int grid[8][8])
{
  system("clear");
  cout << "\t=================================================================================\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|   " << getLabel(grid[0][0]) << "    |   " << getLabel(grid[0][1]) << "    |   " << getLabel(grid[0][2]) << "    |   " << getLabel(grid[0][3]) << "    |   " << getLabel(grid[0][4]) << "    |   " << getLabel(grid[0][5]) << "    |   " << getLabel(grid[0][6]) << "    |   " << getLabel(grid[0][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[1][0]) << "   |   " << getLabel(grid[1][1]) << "    |   " << getLabel(grid[1][2]) << "    |   " << getLabel(grid[1][3]) << "    |   " << getLabel(grid[1][4]) << "    |   " << getLabel(grid[1][5]) << "    |   " << getLabel(grid[1][6]) << "    |   " << getLabel(grid[1][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[2][0]) << "   |   " << getLabel(grid[2][1]) << "    |   " << getLabel(grid[2][2]) << "    |   " << getLabel(grid[2][3]) << "    |   " << getLabel(grid[2][4]) << "    |   " << getLabel(grid[2][5]) << "    |   " << getLabel(grid[2][6]) << "    |   " << getLabel(grid[2][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[3][0]) << "   |   " << getLabel(grid[3][1]) << "    |   " << getLabel(grid[3][2]) << "    |   " << getLabel(grid[3][3]) << "    |   " << getLabel(grid[3][4]) << "    |   " << getLabel(grid[3][5]) << "    |   " << getLabel(grid[3][6]) << "    |   " << getLabel(grid[3][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[4][0]) << "   |   " << getLabel(grid[4][1]) << "    |   " << getLabel(grid[4][2]) << "    |   " << getLabel(grid[4][3]) << "    |   " << getLabel(grid[4][4]) << "    |   " << getLabel(grid[4][5]) << "    |   " << getLabel(grid[4][6]) << "    |   " << getLabel(grid[4][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[5][0]) << "   |   " << getLabel(grid[5][1]) << "    |   " << getLabel(grid[5][2]) << "    |   " << getLabel(grid[5][3]) << "    |   " << getLabel(grid[5][4]) << "    |   " << getLabel(grid[5][5]) << "    |   " << getLabel(grid[5][6]) << "    |   " << getLabel(grid[5][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[6][0]) << "   |   " << getLabel(grid[6][1]) << "    |   " << getLabel(grid[6][2]) << "    |   " << getLabel(grid[6][3]) << "    |   " << getLabel(grid[6][4]) << "    |   " << getLabel(grid[6][5]) << "    |   " << getLabel(grid[6][6]) << "    |   " << getLabel(grid[6][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t---------------------------------------------------------------------------------\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t|    " << getLabel(grid[7][0]) << "   |   " << getLabel(grid[7][1]) << "    |   " << getLabel(grid[7][2]) << "    |   " << getLabel(grid[7][3]) << "    |   " << getLabel(grid[7][4]) << "    |   " << getLabel(grid[7][5]) << "    |   " << getLabel(grid[7][6]) << "    |   " << getLabel(grid[7][7]) << "    |\n"
       << "\t|         |         |         |         |         |         |         |         |\n"
       << "\t=================================================================================\n";
}

//returns the Label of the position on the grid
string getLabel(int value)
{
  string Label = "  ";
  if(value == 0 || value == INOPEN || value == INCLOSED)
    ;//Label = "  "
  else if (value == START)
    Label[1] = 'S';
  else if (value == GOAL)
    Label[1] = 'G';
  else if (value == FORBIDDEN)
    Label[1] = 'F';
  else if (value == UP)
    Label[1] = '^';
  else if (value == DOWN)
    Label[1] = 'v';
  else if (value == LEFT)
    Label = "<-";
  else if (value == RIGHT)
    Label = "->";
  else if (value < 10)
    Label[1] = value + '0';
  else 
    Label = to_string(value);
  return Label;
}

//extracts the path to the goal
stack<Position> path(Position goal, Position positions[], int n)
{
  stack<Position> solutionPath;
  solutionPath.push(goal);
  int prev_row = goal.prev_row;
  int prev_col = goal.prev_col;
  int i;
  while(prev_row != -1) // && prev_col != -1
  {
    i = 0;
    for(i; i <= n; i++)
      if(positions[i].row == prev_row && positions[i].column == prev_col)
      {
        solutionPath.push(positions[i]);
        prev_row = positions[i].prev_row;
        prev_col = positions[i].prev_col;
        break;
      }
  }
  return solutionPath;
}

//prints the path
void outputPath(stack<Position> solPath, int grid[8][8])
{
  //used if SHOW_PATH
  Position prev;//saves the previous position
  bool first_iteration = 1;
  Position path_[solPath.size()];
  int i = 0;
  while(!solPath.empty())
  {
    path_[i] = solPath.top();
    i++;

    if(!ORDER)
    {
      if(first_iteration)
        first_iteration = 0;
      else
      {
        if(grid[prev.row][prev.column] == START)
          ;//do nothing
        else
        {
          if(prev.row > solPath.top().row)
            grid[prev.row][prev.column] = UP;
          else if(prev.row < solPath.top().row)
            grid[prev.row][prev.column] = DOWN;
          else if(prev.column > solPath.top().column)
            grid[prev.row][prev.column] = LEFT;
          else if(prev.column < solPath.top().column)
            grid[prev.row][prev.column] = RIGHT;
            
        }
      }
      prev = solPath.top();
    }
    solPath.pop();
  }
  cout << endl;
  if(!ORDER)
    showGrid(grid);

  for(int j = 0; j < i; j++)
  {    
    cout << "( " << path_[j].row + 1 << ", " << path_[j].column + 1;
    if(j + 1 == i)
      cout << ")" << endl;
    else
      cout << "), ";
  }
}
