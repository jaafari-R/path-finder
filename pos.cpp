struct Position
{
  int row;    //row on the grid
  int column; //column on the grid
  int g;      //distance from start
  int f;      //f() for this position
  int prev_row; //previous node row, and
  int prev_col; // column, to identify the path

  Position(int row, int column, int g, int f, int prev_row, int prev_col)
            : row(row), column(column), g(g), f(f), prev_row(prev_row), prev_col(prev_col)
  {
  }
  Position()
  {
  }
};

struct CompareF
{
  bool operator()(Position const& p1, Position const& p2)
  {
    return p1.f > p2.f;
  }
};