#include <iostream>
#include <string>
#include <stack>
#include <vector>
#include <utility>
#include <algorithm>
#include <queue>
#include <chrono>
#include <thread>

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
const int n = 9;

vector maze(n, vector<int>(n, 0));
vector vis(n, vector<bool>(n, 0));

vector parent(n, vector<pair<int, int>>(n, {-1, -1}));
queue<pair<int, int>> q;
vector step(n, vector<char>(n, '-'));
vector<char> global_direction = {'R', 'L', 'D', 'U'};

stack<pair<int, int>> st;

bool up = true, down = false, rgt = false, lft = false;
bool checkDone()
{
    for (int i = 0; i < n; i += 2)
    {
        for (int j = 0; i < n; j += 2)
        {
            if (!vis[i][j])
            {
                return false;
            }
        }
    }
    return true;
}
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
            {
                if (checkDone())
                    return;
                moveToPrevCell(x, y);
            }
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
            {
                if (checkDone())
                    return;
                moveToPrevCell(x, y);
            }
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
            {
                if (checkDone())
                    return;
                moveToPrevCell(x, y);
            }
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
            {
                if (checkDone())
                    return;
                moveToPrevCell(x, y);
            }
        }
    }
}
void second_run()
{
    int beg_x = n - 1, beg_y = 0;

    vector<vector<pair<int, int>>> bfsParent(n, vector<pair<int, int>>(n, {-1, -1}));
    vector<vector<bool>> visited(n, vector<bool>(n, false));

    queue<pair<int, int>> q;
    q.push({beg_x, beg_y});
    visited[beg_x][beg_y] = true;

    // Center goal cells, computed generically from n (works for any odd n = 2*cells - 1)
    int half = (n - 1) / 2;
    vector<pair<int, int>> goals = {
        {half - 1, half - 1}, {half - 1, half + 1}, {half + 1, half - 1}, {half + 1, half + 1}};

    pair<int, int> goalCell = {-1, -1};
    bool found = false;

    while (!q.empty() && !found)
    {
        auto [x, y] = q.front();
        q.pop();

        for (int k = 0; k < 4 && !found; ++k)
        {
            int xx = x + dx[k];
            int yy = y + dy[k];

            if (xx < 0 || yy < 0 || xx >= n || yy >= n)
                continue;
            if (visited[xx][yy])
                continue;

            bool open = false;
            if (global_direction[k] == 'R')
                open = (maze[x][y + 1] == 1);
            else if (global_direction[k] == 'L')
                open = (maze[x][y - 1] == 1);
            else if (global_direction[k] == 'U')
                open = (maze[x - 1][y] == 1);
            else if (global_direction[k] == 'D')
                open = (maze[x + 1][y] == 1);

            if (open)
            {
                visited[xx][yy] = true;
                bfsParent[xx][yy] = {x, y};
                q.push({xx, yy});

                for (auto &g : goals)
                {
                    if (xx == g.first && yy == g.second)
                    {
                        found = true;
                        goalCell = {xx, yy};
                        break;
                    }
                }
            }
        }
    }

    if (!found)
    {
        log("second_run: no path to goal found");
        return;
    }

    vector<pair<int, int>> path;
    path.push_back(goalCell);
    pair<int, int> cur = goalCell;

    while (!(cur.first == beg_x && cur.second == beg_y))
    {
        cur = bfsParent[cur.first][cur.second];
        path.push_back(cur);
    }
    reverse(path.begin(), path.end());

    log("second_run: shortest path length = " + to_string(path.size() - 1));

    for (size_t i = 1; i < path.size(); ++i)
    {
        int x0 = path[i - 1].first, y0 = path[i - 1].second;
        int x1 = path[i].first, y1 = path[i].second;

        char dir;
        if (x1 == x0 - 2 && y1 == y0)
            dir = 'U';
        else if (x1 == x0 + 2 && y1 == y0)
            dir = 'D';
        else if (y1 == y0 + 2 && x1 == x0)
            dir = 'R';
        else if (y1 == y0 - 2 && x1 == x0)
            dir = 'L';
        else
            continue;

        correctDirection(dir);
        API::moveForward();
    }
}

int main()
{

    log("Running...");
    log("Flood Fill Algorithm");
    if (!checkDone())
    {
        first_run();
        API::turnRight();
        API::turnRight();
        down = false, up = true;
    }

    this_thread::sleep_for(chrono::seconds(5));
    second_run();
}
string x(int a)
{
    cout << "ERROR";
    return " ";
}