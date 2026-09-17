#include "FloorPlan.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>

namespace apopt {

FloorPlan::FloorPlan(int width, int height, double cellSize)
    : m_width{width}
    , m_height{height}
    , m_cellSize{cellSize}
    , m_occupancy(height, width)
    , m_attenuation(height, width)
{
    m_occupancy.zeros();   // Everything starts Outside (0)
    m_attenuation.zeros();
}

void FloorPlan::stampFree(Rect r)
{
    auto occ = m_occupancy.getManipulator();
    int x0{std::max(0, r.x0)};
    int y0{std::max(0, r.y0)};
    int x1{std::min(m_width, r.x1)};
    int y1{std::min(m_height, r.y1)};

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            occ(y, x) = static_cast<double>(CellType::Free);
        }
    }
}

void FloorPlan::stampObstacle(Rect r, double attenuation)
{
    auto occ = m_occupancy.getManipulator();
    auto att = m_attenuation.getManipulator();
    int x0{std::max(0, r.x0)};
    int y0{std::max(0, r.y0)};
    int x1{std::min(m_width, r.x1)};
    int y1{std::min(m_height, r.y1)};

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            if (occ(y, x) == static_cast<double>(CellType::Free)) {
                occ(y, x) = static_cast<double>(CellType::Obstacle);
                att(y, x) = attenuation;
            }
        }
    }
}

void FloorPlan::detectWalls(double wallAttenuation)
{
    auto occ = m_occupancy.getManipulator();
    auto att = m_attenuation.getManipulator();

    std::vector<std::pair<int, int>> toWall;

    static const int dx[4] = { 1, -1, 0, 0 };
    static const int dy[4] = { 0, 0, 1, -1 };

    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            if (occ(y, x) != static_cast<double>(CellType::Free)) {
                continue;
            }

            bool touchesOutside = false;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || ny < 0 || nx >= m_width || ny >= m_height) {
                    touchesOutside = true;
                    break;
                }
                if (occ(ny, nx) == static_cast<double>(CellType::Outside)) {
                    touchesOutside = true;
                    break;
                }
            }

            if (touchesOutside) {
                toWall.emplace_back(x, y);
            }
        }
    }

    for (const auto& [x, y] : toWall) {
        occ(y, x) = static_cast<double>(CellType::Wall);
        att(y, x) = wallAttenuation;
    }
}

void FloorPlan::addRoom(const std::string& name, Rect bounds)
{
    m_rooms.push_back({ name, bounds });
}

std::string FloorPlan::roomAt(int gx, int gy) const
{
    for (const auto& room : m_rooms) {
        if (gx >= room.bounds.x0 && gx < room.bounds.x1 &&
            gy >= room.bounds.y0 && gy < room.bounds.y1) {
            return room.name;
        }
    }
    return "";
}

FloorPlan FloorPlan::generate(LayoutType type,
                               int width,
                               int height,
                               unsigned seed,
                               double wallAttenuation,
                               double obstacleAttenuation)
{
    FloorPlan plan(width, height, 0.5);
    std::mt19937 rng(seed);

    switch (type) {
        case LayoutType::OpenRoom: {
            std::uniform_int_distribution<int> marginDist(1, 3);
            int margin = marginDist(rng);
            Rect mainArea{ margin, margin, width - margin, height - margin };
            plan.stampFree(mainArea);
            plan.addRoom("Open Office Space", mainArea);

            // Randomize position and dimensions of structural columns
            std::uniform_int_distribution<int> colCountDist(1, 3);
            int numCols = colCountDist(rng);
            std::uniform_int_distribution<int> xDist(mainArea.x0 + 4, mainArea.x1 - 6);
            std::uniform_int_distribution<int> yDist(mainArea.y0 + 4, mainArea.y1 - 6);

            for (int c = 0; c < numCols; ++c) {
                int cx = xDist(rng);
                int cy = yDist(rng);
                plan.stampObstacle({ cx, cy, cx + 2, cy + 2 }, obstacleAttenuation);
            }
            break;
        }
        case LayoutType::Corridor: {
            std::uniform_int_distribution<int> roomWDist(width / 5, width / 3);
            std::uniform_int_distribution<int> roomHDist(height / 3, height / 2);
            std::uniform_int_distribution<int> marginDist(1, 3);

            int m = marginDist(rng);
            int w1 = roomWDist(rng), h1 = roomHDist(rng);
            int w2 = roomWDist(rng), h2 = roomHDist(rng);

            std::uniform_int_distribution<int> yOffsetADist(m, std::max(m, height - h1 - m));
            std::uniform_int_distribution<int> yOffsetBDist(m, std::max(m, height - h2 - m));

            int yA = yOffsetADist(rng);
            int yB = yOffsetBDist(rng);

            Rect roomA{ m, yA, m + w1, yA + h1 };
            Rect roomB{ width - m - w2, yB, width - m, yB + h2 };

            plan.stampFree(roomA);
            plan.stampFree(roomB);

            plan.addRoom("Meeting Room A", roomA);
            plan.addRoom("Kitchen & Breakroom", roomB);

            // Connecting corridor
            int cyA = (roomA.y0 + roomA.y1) / 2;
            int cyB = (roomB.y0 + roomB.y1) / 2;

            Rect corrHorizontal{ roomA.x1, std::min(cyA, cyB) - 1, roomB.x0, std::max(cyA, cyB) + 2 };
            Rect corrVertical{ (roomA.x1 + roomB.x0) / 2 - 1, std::min(cyA, cyB) - 1, (roomA.x1 + roomB.x0) / 2 + 2, std::max(cyA, cyB) + 2 };

            plan.stampFree(corrHorizontal);
            plan.stampFree(corrVertical);
            plan.addRoom("Main Corridor", corrHorizontal);

            // Random interior island obstacle inside Room B
            std::uniform_int_distribution<int> islandW(2, 4);
            int iw = islandW(rng);
            if (roomB.x1 - roomB.x0 > iw + 2 && roomB.y1 - roomB.y0 > 4) {
                plan.stampObstacle({ roomB.x0 + 2, roomB.y0 + 2, roomB.x0 + 2 + iw, roomB.y1 - 2 }, obstacleAttenuation);
            }
            break;
        }
        case LayoutType::LShape: {
            std::uniform_int_distribution<int> marginDist(1, 3);
            int m = marginDist(rng);

            std::uniform_int_distribution<int> splitXDist(width / 3, (2 * width) / 3);
            std::uniform_int_distribution<int> splitYDist(height / 3, (2 * height) / 3);

            int splitX = splitXDist(rng);
            int splitY = splitYDist(rng);

            Rect armA{ m, m, width - m, splitY };
            Rect armB{ m, m, splitX, height - m };

            plan.stampFree(armA);
            plan.stampFree(armB);

            plan.addRoom("North Wing", armA);
            plan.addRoom("West Wing", armB);

            // Random internal partition wall with doorway
            std::uniform_int_distribution<int> partXDist(armA.x0 + 5, armA.x1 - 5);
            int px = partXDist(rng);
            plan.stampObstacle({ px, armA.y0, px + 1, armA.y1 }, obstacleAttenuation);
            
            // Doorway gap
            std::uniform_int_distribution<int> doorYDist(armA.y0 + 1, armA.y1 - 3);
            int dy = doorYDist(rng);
            plan.stampFree({ px, dy, px + 1, dy + 3 });
            break;
        }
        case LayoutType::OfficeComplex: {
            std::uniform_int_distribution<int> marginDist(1, 2);
            int m = marginDist(rng);
            Rect outer{ m, m, width - m, height - m };
            plan.stampFree(outer);

            // Vary vertical and horizontal dividing partitions
            std::uniform_int_distribution<int> midXDist((width * 3) / 8, (width * 5) / 8);
            std::uniform_int_distribution<int> midYDist((height * 3) / 8, (height * 5) / 8);

            int midX = midXDist(rng);
            int midY = midYDist(rng);

            // Room definitions
            Rect kitchen{ m, m, midX, midY };
            Rect meeting{ midX, m, width - m, midY };
            Rect openPlan{ m, midY, width - m, height - m };

            plan.addRoom("Kitchen", kitchen);
            plan.addRoom("Meeting Room", meeting);
            plan.addRoom("Open Workspace", openPlan);

            // Interior dividing partition walls
            plan.stampObstacle({ midX - 1, m, midX + 1, midY }, obstacleAttenuation);
            plan.stampObstacle({ m, midY - 1, width - m, midY + 1 }, obstacleAttenuation);

            // Randomize doorway openings along partitions
            std::uniform_int_distribution<int> door1Dist(m + 2, midY - 3);
            std::uniform_int_distribution<int> door2Dist(midX + 2, width - m - 4);
            std::uniform_int_distribution<int> door3Dist(m + 2, width - m - 4);

            int d1 = door1Dist(rng);
            int d2 = door2Dist(rng);
            int d3 = door3Dist(rng);

            plan.stampFree({ midX - 1, d1, midX + 1, d1 + 3 });
            plan.stampFree({ d2, midY - 1, d2 + 3, midY + 1 });
            plan.stampFree({ d3, midY - 1, d3 + 3, midY + 1 });
            break;
        }
    }

    plan.detectWalls(wallAttenuation);
    return plan;
}

bool FloorPlan::isFree(int gx, int gy) const
{
    if (gx < 0 || gy < 0 || gx >= m_width || gy >= m_height) {
        return false;
    }
    auto occ = const_cast<dense::DblMatrix&>(m_occupancy).getManipulator();
    return occ(gy, gx) == static_cast<double>(CellType::Free);
}

bool FloorPlan::isObstacle(int gx, int gy) const 
{
    if (gx < 0 || gx >= m_width || gy < 0 || gy >= m_height) {
        return true;
    }
    auto occ = const_cast<dense::DblMatrix&>(m_occupancy).getManipulator();
    return occ(gy, gx) == static_cast<double>(CellType::Obstacle);
}

bool FloorPlan::isFreeWorld(Point2D p) const
{
    int gx, gy;
    worldToGrid(p, gx, gy);
    return isFree(gx, gy);
}

double FloorPlan::attenuationAt(int gx, int gy) const 
{
    if (gx < 0 || gx >= m_width || gy < 0 || gy >= m_height) {
        return 1000.0; // Treat out-of-bounds as solid boundary
    }
    auto att = const_cast<dense::DblMatrix&>(m_attenuation).getManipulator();
    return att(gy, gx);
}

Point2D FloorPlan::gridToWorld(int gx, int gy) const
{
    return Point2D{ (gx + 0.5) * m_cellSize, (gy + 0.5) * m_cellSize };
}

void FloorPlan::worldToGrid(Point2D p, int& gx, int& gy) const
{
    gx = static_cast<int>(p.x / m_cellSize);
    gy = static_cast<int>(p.y / m_cellSize);
}

double FloorPlan::lineOfSightAttenuation(Point2D from, Point2D to) const
{
    int x0, y0, x1, y1;
    worldToGrid(from, x0, y0);
    worldToGrid(to, x1, y1);

    int dx{std::abs(x1 - x0)};
    int dy{-std::abs(y1 - y0)};
    int sx{(x0 < x1) ? 1 : -1};
    int sy{(y0 < y1) ? 1 : -1};
    int err{dx + dy};

    double total{0.0};
    int x{x0};
    int y{y0};

    int maxSteps = std::abs(x1 - x0) + std::abs(y1 - y0) + 2;
    int steps = 0;

    while (steps++ < maxSteps) {
        total += attenuationAt(x, y);
        if (x == x1 && y == y1) {
            break;
        }
        int e2{2 * err};
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy; 
        }
    }

    return total;
}

bool FloorPlan::isFullyConnected() const
{
    auto occ = const_cast<dense::DblMatrix&>(m_occupancy).getManipulator();

    int startX{-1}, startY{-1};
    long freeCount{0};

    for (int y = 0; y < m_height && startX < 0; ++y) {
        for (int x = 0; x < m_width; ++x) {
            if (occ(y, x) == static_cast<double>(CellType::Free)) {
                startX = x;
                startY = y;
                break;
            }
        }
    }
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            if (occ(y, x) == static_cast<double>(CellType::Free)) {
                ++freeCount;
            }
        }
    }

    if (startX < 0) return true;

    std::vector<std::vector<bool>> visited(m_height, std::vector<bool>(m_width, false));
    std::queue<std::pair<int, int>> q;
    q.push({ startX, startY });
    visited[startY][startX] = true;
    long reached{1};

    static const int dx[4] = { 1, -1, 0, 0 };
    static const int dy[4] = { 0, 0, 1, -1 };

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        for (int k = 0; k < 4; ++k) {
            int nx{cx + dx[k]};
            int ny{cy + dy[k]};
            if (nx < 0 || ny < 0 || nx >= m_width || ny >= m_height) continue;
            if (visited[ny][nx]) continue;
            if (occ(ny, nx) != static_cast<double>(CellType::Free)) continue;
            visited[ny][nx] = true;
            ++reached;
            q.push({ nx, ny });
        }
    }

    return reached == freeCount;
}

void FloorPlan::saveAttenuationMtx(const std::string& filepath) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }

    auto att = const_cast<dense::DblMatrix&>(m_attenuation).getManipulator();

    std::vector<std::string> triplets;
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            double v = att(y, x);
            if (v != 0.0) {
                std::ostringstream line;
                line << (y + 1) << " " << (x + 1) << " " << v;
                triplets.push_back(line.str());
            }
        }
    }

    file << "%%MatrixMarket matrix coordinate real general\n";
    file << "% FloorPlan attenuation grid (" << m_width << "x" << m_height
         << "), cellSize=" << m_cellSize << "\n";
    file << m_height << " " << m_width << " " << triplets.size() << "\n";
    for (const auto& t : triplets) {
        file << t << "\n";
    }
}

dense::DblMatrix FloorPlan::loadAttenuationMtx(const std::string& filepath,
                                                int& rows,
                                                int& cols,
                                                int& nnz)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filepath);
    }

    std::string line;
    do {
        std::getline(file, line);
    } while (!line.empty() && line[0] == '%');

    std::stringstream dimStream(line);
    dimStream >> rows >> cols >> nnz;

    dense::DblMatrix A(rows, cols);
    A.zeros();
    auto a = A.getManipulator();

    int i{0}, j{0};
    double value{0.0};
    while (file >> i >> j >> value) {
        a(i - 1, j - 1) = value;
    }

    return A;
}

} // namespace apopt