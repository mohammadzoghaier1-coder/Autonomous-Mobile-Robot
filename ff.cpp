#include <iostream>
#include <string>
#include <stack>
#include <vector>
#include <utility>
#include <algorithm>

#include "API.h"
using namespace std;

int dy[4] = {2, -2, 0, 0};
int dx[4] = {0, 0, 2, -2};

void log(const std::string &text)
{
    std::cerr << text << std::endl;
}
// enter n : n = (maze length )^2 - 1
//  test for 16*16 maze
const int n = 31;

vector maze(n, vector<int>(n, 0));
vector vis(n, vector<bool>(n, 0));

vector parent(n, vector<pair<int, int>>(n, {-1, -1}));

vector<char> global_direction = {'R', 'L', 'D', 'U'};

stack<pair<int, int>> st;

bool up = true, down = false, rgt = false, lft = false;

void correctDirection(char globalDirection)
{
    if (up)
    {
        if (globalDirection == 'R')
        {
            API::turnRight();
            up = 0;
            rgt = 1;
        }
        else if (globalDirection == 'D')
        {
            API::turnRight();
            API::turnRight();
            up = 0;
            down = 1;
        }
        else if (globalDirection == 'L')
        {
            API::turnLeft();
            up = 0;
            lft = 1;
        }
    }
    else if (rgt)
    {
        if (globalDirection == 'U')
        {
            API::turnLeft();
            up = 1;
            rgt = 0;
        }
        else if (globalDirection == 'D')
        {
            API::turnRight();
            rgt = 0;
            down = 1;
        }
        else if (globalDirection == 'L')
        {
            API::turnRight();
            API::turnRight();
            rgt = 0;
            lft = 1;
        }
    }
    else if (lft)
    {
        if (globalDirection == 'U')
        {
            API::turnRight();
            up = 1;
            lft = 0;
        }
        else if (globalDirection == 'D')
        {
            API::turnLeft();
            lft = 0;
            down = 1;
        }
        else if (globalDirection == 'R')
        {
            API::turnRight();
            API::turnRight();
            lft = 0;
            rgt = 1;
        }
    }
    else if (down)
    {
        if (globalDirection == 'L')
        {
            API::turnRight();
            down = 0;
            lft = 1;
        }
        else if (globalDirection == 'R')
        {
            API::turnLeft();
            down = 0;
            rgt = 1;
        }
        else if (globalDirection == 'U')
        {
            API::turnRight();
            API::turnRight();
            down = 0;
            up = 1;
        }
    }
}

void moveForward(int x, int y, char globalDirection)
{

    if (globalDirection == 'R')
    {
        parent[x][y] = {x, y - 2};
    }
    else if (globalDirection == 'L')
    {
        parent[x][y] = {x, y + 2};
    }
    else if (globalDirection == 'U')
    {
        parent[x][y] = {x + 2, y};
    }
    else if (globalDirection == 'D')
    {
        parent[x][y] = {x - 2, y};
    }

    st.push({x, y});
    vis[x][y] = 1;

    correctDirection(globalDirection);

    API::moveForward();
}

void moveToPrevCell(int &x, int &y)
{
    while (parent[x][y].first != -1)
    {

        int parent_x = parent[x][y].first;
        int parent_y = parent[x][y].second;

        char backDirection;

        if (parent_x == x && parent_y == y - 2)
            backDirection = 'L';
        else if (parent_x == x && parent_y == y + 2)
            backDirection = 'R';
        else if (parent_x == x - 2 && parent_y == y)
            backDirection = 'U';
        else if (parent_x == x + 2 && parent_y == y)
            backDirection = 'D';
        else
            return;

        correctDirection(backDirection);
        API::moveForward();

        x = parent_x;
        y = parent_y;

        for (int k = 0; k < 4; ++k)
        {
            int xx = x + dx[k];
            int yy = y + dy[k];

            if (xx >= 0 && yy >= 0 &&
                xx < n && yy < n &&
                !vis[xx][yy])
            {
                if (global_direction[k] == 'R')
                {
                    if (maze[x][y + 1] == 1)
                    {
                        moveForward(x, y + 2, 'R');
                        return;
                    }
                }
                else if (global_direction[k] == 'L')
                {
                    if (maze[x][y - 1] == 1)
                    {
                        moveForward(x, y - 2, 'L');
                        return;
                    }
                }
                else if (global_direction[k] == 'U')
                {
                    if (maze[x - 1][y] == 1)
                    {
                        moveForward(x - 2, y, 'U');
                        return;
                    }
                }
                else if (global_direction[k] == 'D')
                {
                    if (maze[x + 1][y] == 1)
                    {
                        moveForward(x + 2, y, 'D');
                        return;
                    }
                }
            }
        }
    }
}
void first_run()
{
    int beg_x = n - 1, beg_y = 0;

    st.push({beg_x, beg_y});
    vis[beg_x][beg_y] = true;

    while (!st.empty())
    {
        int x = st.top().first;
        int y = st.top().second;

        st.pop();

        bool nwf = !API::wallFront(); // 0-> wall , 1-> free
        bool nwr = !API::wallRight();
        bool nwl = !API::wallLeft();

        if (up)
        {
            if (x - 1 >= 0)
                maze[x - 1][y] = nwf;
            if (y + 1 < n)
                maze[x][y + 1] = nwr;
            if (y - 1 >= 0)
                maze[x][y - 1] = nwl;

            if (x - 2 >= 0 and nwf and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else if (y + 2 < n and nwr and !vis[x][y + 2])
                moveForward(x, y + 2, 'R');
            else if (y - 2 >= 0 and nwl and !vis[x][y - 2])
                moveForward(x, y - 2, 'L');
            else
                moveToPrevCell(x, y);
        }
        else if (down)
        {
            if (x + 1 < n)
                maze[x + 1][y] = nwf;
            if (y + 1 < n)
                maze[x][y + 1] = nwl;
            if (y - 1 >= 0)
                maze[x][y - 1] = nwr;

            if (x + 2 < n and nwf and !vis[x + 2][y])
                moveForward(x + 2, y, 'D');
            else if (y - 2 >= 0 and nwr and !vis[x][y - 2])
                moveForward(x, y - 2, 'L');
            else if (y + 2 < n and nwl and !vis[x][y + 2])
                moveForward(x, y + 2, 'R');
            else
                moveToPrevCell(x, y);
        }
        else if (rgt)
        {
            if (y + 1 < n)
                maze[x][y + 1] = nwf;
            if (x + 1 < n)
                maze[x + 1][y] = nwr;
            if (x - 1 >= 0)
                maze[x - 1][y] = nwl;

            if (y + 2 < n and nwf and !vis[x][y + 2])
                moveForward(x, y + 2, 'R');
            else if (x + 2 < n and nwr and !vis[x + 2][y])
                moveForward(x + 2, y, 'D');
            else if (x - 2 >= 0 and nwl and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else
                moveToPrevCell(x, y);
        }
        else if (lft)
        {
            if (y - 1 >= 0)
                maze[x][y - 1] = nwf;
            if (x - 1 >= 0)
                maze[x - 1][y] = nwr;
            if (x + 1 < n)
                maze[x + 1][y] = nwl;

            if (y - 2 >= 0 and nwf and !vis[x][y - 2])
                moveForward(x, y - 2, 'L');
            else if (x - 2 >= 0 and nwr and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else if (x + 2 < n and nwl and !vis[x + 2][y])
                moveForward(x + 2, y, 'D');
            else
                moveToPrevCell(x, y);
        }
    }
}

int main()
{
    log("Running...");
    log("Flood Fill Algorithm");

    first_run();
}
