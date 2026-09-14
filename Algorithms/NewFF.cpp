#include <iostream>
#include <string>
#include <queue>

#include "API.h"

enum Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum RunMode
{
    FIRST_EXPLORATION,
    SECOND_EXPLORATION,
    SPEED_RUN
};

const int SIZE = 16;
const int NUM_GOALS = 4;
const int INF = 999;

struct Cell
{
    int x;
    int y;
};

Direction direction = NORTH;

int mouseX = 0;
int mouseY = 0;

int flood[SIZE][SIZE];

bool walls[SIZE][SIZE][4] = {};

bool known[SIZE][SIZE][4] = {};

bool visited[SIZE][SIZE] = {};

bool traveled[SIZE][SIZE][4] = {};

std::queue<Cell> q;

int goalXs[NUM_GOALS] = {3, 3, 4, 4};
int goalYs[NUM_GOALS] = {3, 4, 3, 4};

void log(const std::string &text)
{
    std::cerr << text << std::endl;
}

bool inBounds(int x, int y)
{
    return x >= 0 &&
           x < SIZE &&
           y >= 0 &&
           y < SIZE;
}

Direction opposite(Direction d)
{
    return (Direction)((d + 2) % 4);
}

void getNeighbor(
    int x,
    int y,
    Direction d,
    int &nx,
    int &ny)
{
    nx = x;
    ny = y;

    if (d == NORTH)
        ny++;

    else if (d == EAST)
        nx++;

    else if (d == SOUTH)
        ny--;

    else if (d == WEST)
        nx--;
}

bool isGoal(int x, int y)
{
    for (int i = 0; i < NUM_GOALS; i++)
    {
        if (goalXs[i] == x &&
            goalYs[i] == y)
        {
            return true;
        }
    }

    return false;
}

void initializeFlood()
{
    for (int x = 0; x < SIZE; x++)
    {
        for (int y = 0; y < SIZE; y++)
        {
            int best = INF;

            for (int i = 0; i < NUM_GOALS; i++)
            {
                int dx = x - goalXs[i];
                int dy = y - goalYs[i];

                if (dx < 0)
                    dx = -dx;

                if (dy < 0)
                    dy = -dy;

                int dist = dx + dy;

                if (dist < best)
                    best = dist;
            }

            flood[x][y] = best;
        }
    }
}

void showFlood()
{
    for (int x = 0; x < SIZE; x++)
    {
        for (int y = 0; y < SIZE; y++)
        {
            if (flood[x][y] >= INF)
            {
                API::setText(x, y, "X");
            }
            else
            {
                API::setText(
                    x,
                    y,
                    std::to_string(flood[x][y]));
            }
        }
    }
}

void recordWall(
    int x,
    int y,
    Direction d)
{
    walls[x][y][d] = true;
    known[x][y][d] = true;

    int nx;
    int ny;

    getNeighbor(x, y, d, nx, ny);

    if (inBounds(nx, ny))
    {
        Direction otherSide = opposite(d);

        walls[nx][ny][otherSide] = true;
        known[nx][ny][otherSide] = true;
    }
}

void recordOpen(
    int x,
    int y,
    Direction d)
{
    walls[x][y][d] = false;
    known[x][y][d] = true;

    int nx;
    int ny;

    getNeighbor(x, y, d, nx, ny);

    if (inBounds(nx, ny))
    {
        Direction otherSide = opposite(d);

        walls[nx][ny][otherSide] = false;
        known[nx][ny][otherSide] = true;
    }
}

void markTraveled(
    int x,
    int y,
    Direction d)
{
    traveled[x][y][d] = true;

    int nx;
    int ny;

    getNeighbor(x, y, d, nx, ny);

    if (inBounds(nx, ny))
    {
        traveled[nx][ny][opposite(d)] = true;
    }
}

void turnRight()
{
    API::turnRight();

    direction =
        (Direction)((direction + 1) % 4);
}

void turnLeft()
{
    API::turnLeft();

    direction =
        (Direction)((direction + 3) % 4);
}

void faceDirection(Direction target)
{
    int difference =
        (target - direction + 4) % 4;

    if (difference == 1)
    {
        turnRight();
    }

    else if (difference == 2)
    {
        turnRight();
        turnRight();
    }

    else if (difference == 3)
    {
        turnLeft();
    }
}

void moveForward()
{
    int oldX = mouseX;
    int oldY = mouseY;

    Direction moveDirection = direction;

    API::moveForward();

    recordOpen(
        oldX,
        oldY,
        moveDirection);

    markTraveled(
        oldX,
        oldY,
        moveDirection);

    getNeighbor(
        oldX,
        oldY,
        moveDirection,
        mouseX,
        mouseY);
}

void senseWalls()
{
    Direction frontDirection =
        direction;

    Direction rightDirection =
        (Direction)((direction + 1) % 4);

    Direction leftDirection =
        (Direction)((direction + 3) % 4);

    if (API::wallFront())
    {
        recordWall(
            mouseX,
            mouseY,
            frontDirection);
    }
    else
    {
        recordOpen(
            mouseX,
            mouseY,
            frontDirection);
    }

    if (API::wallRight())
    {
        recordWall(
            mouseX,
            mouseY,
            rightDirection);
    }
    else
    {
        recordOpen(
            mouseX,
            mouseY,
            rightDirection);
    }

    if (API::wallLeft())
    {
        recordWall(
            mouseX,
            mouseY,
            leftDirection);
    }
    else
    {
        recordOpen(
            mouseX,
            mouseY,
            leftDirection);
    }
}

void markVisited()
{
    visited[mouseX][mouseY] = true;

    API::setColor(
        mouseX,
        mouseY,
        'c');
}

int getMinNeighbor(int x, int y)
{
    int minValue = INF;

    for (int i = 0; i < 4; i++)
    {
        Direction d =
            (Direction)i;

        int nx;
        int ny;

        getNeighbor(
            x,
            y,
            d,
            nx,
            ny);

        if (!inBounds(nx, ny))
            continue;

        if (walls[x][y][d])
            continue;

        if (flood[nx][ny] < minValue)
        {
            minValue =
                flood[nx][ny];
        }
    }

    return minValue;
}

bool updateCell(int x, int y)
{
    if (isGoal(x, y))
        return false;

    int minNeighbor =
        getMinNeighbor(x, y);

    if (minNeighbor >= INF)
        return false;

    int newValue =
        minNeighbor + 1;

    if (flood[x][y] != newValue)
    {
        flood[x][y] =
            newValue;

        return true;
    }

    return false;
}

void reflood()
{
    while (!q.empty())
    {
        q.pop();
    }

    q.push({mouseX,
            mouseY});

    for (int i = 0; i < 4; i++)
    {
        int nx;
        int ny;

        getNeighbor(
            mouseX,
            mouseY,
            (Direction)i,
            nx,
            ny);

        if (inBounds(nx, ny))
        {
            q.push({nx,
                    ny});
        }
    }

    while (!q.empty())
    {
        Cell current =
            q.front();

        q.pop();

        if (!updateCell(
                current.x,
                current.y))
        {
            continue;
        }

        for (int i = 0; i < 4; i++)
        {
            Direction d =
                (Direction)i;

            int nx;
            int ny;

            getNeighbor(
                current.x,
                current.y,
                d,
                nx,
                ny);

            if (!inBounds(nx, ny))
                continue;

            if (walls[current.x]
                     [current.y]
                     [d])
            {
                continue;
            }

            q.push({nx,
                    ny});
        }
    }
}

bool getBestDirection(
    int x,
    int y,
    bool preferUnexplored,
    bool confirmedOnly,
    Direction &bestDirection)
{
    int bestValue = INF;

    Direction candidates[4];

    int candidateCount = 0;

    for (int i = 0; i < 4; i++)
    {
        Direction d =
            (Direction)i;

        int nx;
        int ny;

        getNeighbor(
            x,
            y,
            d,
            nx,
            ny);

        if (!inBounds(nx, ny))
            continue;

        if (walls[x][y][d])
            continue;

        if (confirmedOnly &&
            !known[x][y][d])
        {
            continue;
        }

        if (flood[nx][ny] < bestValue)
        {
            bestValue =
                flood[nx][ny];
        }
    }

    if (bestValue >= INF)
    {
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        Direction d =
            (Direction)i;

        int nx;
        int ny;

        getNeighbor(
            x,
            y,
            d,
            nx,
            ny);

        if (!inBounds(nx, ny))
            continue;

        if (walls[x][y][d])
            continue;

        if (confirmedOnly &&
            !known[x][y][d])
        {
            continue;
        }

        if (flood[nx][ny] ==
            bestValue)
        {
            candidates[candidateCount++] = d;
        }
    }

    if (preferUnexplored)
    {
        // PRIORITY #1

        for (int i = 0;
             i < candidateCount;
             i++)
        {
            Direction d =
                candidates[i];

            if (!traveled[x][y][d])
            {
                bestDirection = d;

                return true;
            }
        }

        // PRIORITY #2

        for (int i = 0;
             i < candidateCount;
             i++)
        {
            Direction d =
                candidates[i];

            int nx;
            int ny;

            getNeighbor(
                x,
                y,
                d,
                nx,
                ny);

            if (!visited[nx][ny])
            {
                bestDirection = d;

                return true;
            }
        }
    }

    bestDirection =
        candidates[0];

    return true;
}

bool runExploration(RunMode mode)
{
    bool preferUnexplored =
        (mode == SECOND_EXPLORATION);

    while (!isGoal(
        mouseX,
        mouseY))
    {
        markVisited();

        senseWalls();

        reflood();

        showFlood();

        Direction best;

        if (!getBestDirection(
                mouseX,
                mouseY,
                preferUnexplored,
                false,
                best))
        {
            log(
                "ERROR: no available "
                "direction during exploration");

            return false;
        }

        faceDirection(best);

        moveForward();
    }

    markVisited();

    senseWalls();

    reflood();

    showFlood();

    return true;
}

bool isConfirmedOpen(
    int x,
    int y,
    Direction d)
{
    int nx;
    int ny;

    getNeighbor(
        x,
        y,
        d,
        nx,
        ny);

    if (!inBounds(nx, ny))
    {
        return false;
    }

    return known[x][y][d] &&
           !walls[x][y][d];
}

void calculateFinalFlood()
{
    std::queue<Cell> bfs;

    for (int x = 0; x < SIZE; x++)
    {
        for (int y = 0; y < SIZE; y++)
        {
            flood[x][y] = INF;
        }
    }

    for (int i = 0;
         i < NUM_GOALS;
         i++)
    {
        int gx =
            goalXs[i];

        int gy =
            goalYs[i];

        flood[gx][gy] = 0;

        bfs.push({gx,
                  gy});
    }

    while (!bfs.empty())
    {
        Cell current =
            bfs.front();

        bfs.pop();

        for (int i = 0; i < 4; i++)
        {
            Direction d =
                (Direction)i;

            if (!isConfirmedOpen(
                    current.x,
                    current.y,
                    d))
            {
                continue;
            }

            int nx;
            int ny;

            getNeighbor(
                current.x,
                current.y,
                d,
                nx,
                ny);

            int nextValue =
                flood[current.x]
                     [current.y] +
                1;

            if (nextValue <
                flood[nx][ny])
            {
                flood[nx][ny] =
                    nextValue;

                bfs.push({nx,
                          ny});
            }
        }
    }
}

bool runSpeedRun()
{
    calculateFinalFlood();

    showFlood();

    if (flood[mouseX][mouseY] >= INF)
    {
        log(
            "ERROR: no confirmed path "
            "from start to goal");

        return false;
    }

    while (!isGoal(
        mouseX,
        mouseY))
    {
        API::setColor(
            mouseX,
            mouseY,
            'y');

        Direction best;

        if (!getBestDirection(
                mouseX,
                mouseY,
                false,
                true,
                best))
        {
            log(
                "ERROR: no confirmed "
                "direction during speed run");

            return false;
        }

        int nx;
        int ny;

        getNeighbor(
            mouseX,
            mouseY,
            best,
            nx,
            ny);

        if (flood[nx][ny] !=
            flood[mouseX][mouseY] - 1)
        {
            log(
                "ERROR: final flood "
                "invariant broken");

            return false;
        }

        faceDirection(best);

        moveForward();
    }

    API::setColor(
        mouseX,
        mouseY,
        'g');

    return true;
}

// Simulator manual reset

void waitForManualReset(
    const std::string &nextRun)
{
    log("");
    log("CENTER REACHED.");

    log(
        "Press the MMS RESET button "
        "to return the mouse to start.");

    log(
        "Next: " + nextRun);

    while (!API::wasReset())
    {
        // Wait
    }

    API::ackReset();

    mouseX = 0;
    mouseY = 0;

    direction = NORTH;

    log(
        "Mouse returned to START.");

    log(
        "Maze memory preserved.");

    log("");
}

int main(
    int argc,
    char *argv[])
{
    log("Running...");

    initializeFlood();

    showFlood();

    for (int i = 0;
         i < NUM_GOALS;
         i++)
    {
        API::setColor(
            goalXs[i],
            goalYs[i],
            'G');
    }

    // RUN 1

    log(
        "================================");

    log(
        "RUN 1: FIRST EXPLORATION");

    log(
        "================================");

    if (!runExploration(
            FIRST_EXPLORATION))
    {
        return 1;
    }

    // MANUALLY RESET TO START

    waitForManualReset(
        "RUN 2");

    // RUN 2

    log(
        "================================");

    log(
        "RUN 2: SMART EXPLORATION");

    log(
        "Prefer unexplored roads "
        "when flood values are equal.");

    log(
        "================================");

    if (!runExploration(
            SECOND_EXPLORATION))
    {
        return 1;
    }

    waitForManualReset(
        "FINAL SPEED RUN");

    // RUN 3

    log(
        "================================");

    log(
        "RUN 3: FINAL SPEED RUN");

    log(
        "Calculating shortest "
        "confirmed path...");

    log(
        "================================");

    if (!runSpeedRun())
    {
        return 1;
    }

    log("");
    log(
        "FINAL SPEED RUN COMPLETE!");

    return 0;
}
