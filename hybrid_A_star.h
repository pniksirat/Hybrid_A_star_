#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <vector>

using std::vector;

class HBF {
 public:
  // Constructor
  HBF();

  // Destructor
  virtual ~HBF();

  // HBF structs
  struct maze_s {
    int g;  // iteration
    int f;
    double x;
    double y;
    double theta;
  };

  struct maze_path {
    vector<vector<vector<int>>> closed;
    vector<vector<vector<maze_s>>> came_from;
    maze_s final;
  };
  
  struct less_than_f
{
    inline bool operator() (const maze_s& struct1, const maze_s& struct2)
    {
        return (struct1.f < struct2.f);
    }
};
  
  // HBF functions
  int theta_to_stack_number(double theta);

  int idx(double float_num);
  int Heuristic(int gridx, int gridy,vector<int> &goal);

  vector<maze_s> expand(maze_s &state, vector<int> &goal);

  vector<maze_s> reconstruct_path(vector<vector<vector<maze_s>>> &came_from, 
                                  vector<double> &start, HBF::maze_s &final);

  maze_path search(vector<vector<int>> &grid, vector<double> &start, 
                   vector<int> &goal);

 private:
  const int NUM_THETA_CELLS = 90;
  const double SPEED = 1.45;
  const double LENGTH = 0.5;
};


#endif  // HYBRID_A_STAR_H_