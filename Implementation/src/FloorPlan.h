#pragma once

#include <dense/Matrix.h>
#include <string>
#include <vector>

namespace apopt {

// Cell classification, stored as double values inside m_occupancy
enum class CellType {
    Outside  = 0, // Not part of the building footprint
    Free     = 1, // Walkable interior space
    Wall     = 2, // Outer or structural wall boundary
    Obstacle = 3  // Interior obstruction
};

enum class LayoutType {
    OpenRoom,
    Corridor,
    LShape,
    OfficeComplex
};

struct Rect {
    int x0, y0, x1, y1; 
};

struct Point2D {
    double x;
    double y;
};

// Named region within the floor plan 
struct RoomZone {
    std::string name;
    Rect bounds;
};

class FloorPlan {
public:
    explicit FloorPlan(int width, int height, double cellSize = 0.5);

    // Generates layouts with optional room labels and interior obstacles
    static FloorPlan generate(LayoutType type,
                               int width,
                               int height,
                               unsigned seed,
                               double wallAttenuation = 3.0,
                               double obstacleAttenuation = 5.0);

    bool isFree(int gx, int gy) const;
    bool isFreeWorld(Point2D p) const;
    bool isFreeWorld(double x, double y) const {
        return isFreeWorld(Point2D{x, y});
    }

    bool isObstacle(int gx, int gy) const;

    double attenuationAt(int gx, int gy) const;

    double lineOfSightAttenuation(Point2D from, Point2D to) const;

    double lineOfSightAttenuation(double x0, double y0, double x1, double y1) const {
        return lineOfSightAttenuation(Point2D{x0, y0}, Point2D{x1, y1});
    }

    // Checks connectivity across all walkable Free cells
    bool isFullyConnected() const;

    void addRoom(const std::string& name, Rect bounds);
    const std::vector<RoomZone>& rooms() const { return m_rooms; }
    std::string roomAt(int gx, int gy) const;

    int width() const { return m_width; }
    int height() const { return m_height; }
    double cellSize() const { return m_cellSize; }
    void setCellSize(double cs) { m_cellSize = cs; }

    Point2D gridToWorld(int gx, int gy) const;
    void worldToGrid(Point2D p, int& gx, int& gy) const;

    const dense::DblMatrix& occupancy() const { return m_occupancy; }
    const dense::DblMatrix& attenuation() const { return m_attenuation; }

    void saveAttenuationMtx(const std::string& filepath) const;
    static dense::DblMatrix loadAttenuationMtx(const std::string& filepath, int& rows, int& cols, int& nnz);

private:
    void stampFree(Rect r);
    void stampObstacle(Rect r, double attenuation);
    void detectWalls(double wallAttenuation);

    int m_width;
    int m_height;
    double m_cellSize;
    
    dense::DblMatrix m_occupancy;  
    dense::DblMatrix m_attenuation; 
    std::vector<RoomZone> m_rooms;  
};

} // namespace apopt