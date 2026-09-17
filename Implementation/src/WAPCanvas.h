#pragma once
#include <gui/Canvas.h>
#include <gui/Shape.h>
#include <gui/DrawableString.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "FloorPlan.h"
#include "Objective.h"
#include "Gradient.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class WAPCanvas : public gui::Canvas {
private:
    static constexpr int kGridWidth = 40;
    static constexpr int kGridHeight = 25;
    static constexpr int kMaxAPs = 10;
    static constexpr std::size_t kLayoutCount = 4;

    apopt::LayoutType m_layoutType{ apopt::LayoutType::OfficeComplex };
    int m_numAPs{3};
    double m_apRange{7.0}; // Nominal AP coverage range in meters (at referencePower)
    double m_powerBudget{100.0}; // Total transmit power budget in milliwatts (mW)
    unsigned m_seedCounter{1};

    std::optional<apopt::FloorPlan> m_plan;
    std::optional<apopt::Objective> m_objective;
    std::vector<double> m_currentU;
    apopt::OptimizationResult m_lastResult;
    bool m_hasResult{false};

    std::mt19937 m_rng{ std::random_device{}() };

    // Screen zones
    gui::CoordType leftZoneLeft{0}, leftZoneTop{0}, leftZoneWidth{0}, leftZoneHeight{0};
    gui::CoordType plotTop{0}, plotHeight{160};
    gui::CoordType rightZoneLeft{0}, rightZoneTop{0}, rightZoneWidth{0};

    // Interactive regions
    gui::Rect m_generateButtonRect;
    gui::Rect m_layoutDropdownRect;
    gui::Rect m_layoutDropdownItemRects[kLayoutCount];
    bool m_layoutDropdownExpanded{false};
    gui::Rect m_randomizeButtonRect;
    gui::Rect m_apCountMinusRect, m_apCountPlusRect;
    gui::Rect m_budgetMinusRect, m_budgetPlusRect;
    gui::Rect m_runButtonRect;

    void drawWrappedString(const std::string& text, const gui::Rect& rect,
                           gui::Font::ID fontId, td::ColorID color,
                           gui::CoordType lineHeight = 16,
                           std::size_t approxMaxCharsPerLine = 34) {
        std::stringstream ss(text);
        std::string word;
        std::vector<std::string> lines;
        std::string currentLine;

        while (ss >> word) {
            if (currentLine.empty()) {
                currentLine = word;
            } else if (currentLine.length() + 1 + word.length() <= approxMaxCharsPerLine) {
                currentLine += " " + word;
            } else {
                lines.push_back(currentLine);
                currentLine = word;
            }
        }
        if (!currentLine.empty()) {
            lines.push_back(currentLine);
        }

        gui::CoordType curY = rect.top;
        for (const auto& line : lines) {
            if (curY + lineHeight > rect.bottom) break;
            gui::Rect lineRect(rect.left, curY, rect.right, curY + lineHeight);
            gui::DrawableString::draw(line.c_str(), line.length(), lineRect,
                fontId, color, td::TextAlignment::Left, td::VAlignment::Top);
            curY += lineHeight;
        }
    }

    static const char* layoutName(apopt::LayoutType t) {
        if (t == apopt::LayoutType::OpenRoom)      return "Open Room";
        if (t == apopt::LayoutType::Corridor)      return "Corridor";
        if (t == apopt::LayoutType::LShape)        return "L-Shape";
        if (t == apopt::LayoutType::OfficeComplex) return "Office Complex";
        return "";
    }

    gui::CoordType cellPixelSize() const {
        if (!m_plan) return 0;
        return std::min(leftZoneWidth / m_plan->width(), leftZoneHeight / m_plan->height());
    }

    gui::Point gridOrigin() const {
        gui::CoordType cellPx = cellPixelSize();
        gui::CoordType gridPxW = cellPx * m_plan->width();
        gui::CoordType gridPxH = cellPx * m_plan->height();
        return { leftZoneLeft + (leftZoneWidth - gridPxW) / 2,
                 leftZoneTop + (leftZoneHeight - gridPxH) / 2 };
    }

    gui::Point worldToScreen(apopt::Point2D p) const {
        gui::CoordType cellPx = cellPixelSize();
        gui::Point origin = gridOrigin();
        return { origin.x + static_cast<gui::CoordType>((p.x / m_plan->cellSize()) * cellPx),
                 origin.y + static_cast<gui::CoordType>((p.y / m_plan->cellSize()) * cellPx) };
    }

    apopt::SignalParams currentSignalParams() const {
        apopt::SignalParams params;
        params.totalPowerBudget = m_powerBudget;
        params.referenceRange = m_apRange;
        return params;
    }

    void generateFloorPlan() {
        m_plan.emplace(apopt::FloorPlan::generate(
            m_layoutType, kGridWidth, kGridHeight, m_seedCounter,
            3.0, 5.0
        ));
        m_objective.emplace(*m_plan, currentSignalParams());
        randomizeApPositions();
    }

    void randomizeApPositions() {
        if (!m_plan) return;

        std::vector<std::pair<int, int>> freeCells;
        for (int gy = 0; gy < m_plan->height(); ++gy) {
            for (int gx = 0; gx < m_plan->width(); ++gx) {
                if (m_plan->isFree(gx, gy)) freeCells.emplace_back(gx, gy);
            }
        }

        m_currentU.assign(static_cast<std::size_t>(m_numAPs) * 2, 0.0);
        if (!freeCells.empty()) {
            std::uniform_int_distribution<std::size_t> pick(0, freeCells.size() - 1);
            for (int i = 0; i < m_numAPs; ++i) {
                auto [gx, gy]{ freeCells[pick(m_rng)] };
                apopt::Point2D w{ m_plan->gridToWorld(gx, gy) };
                m_currentU[2 * i] = w.x;
                m_currentU[2 * i + 1] = w.y;
            }
        }

        m_hasResult = false;
        m_lastResult = apopt::OptimizationResult{};
        reDraw();
    }

    void runOptimization() {
        if (!m_plan || !m_objective) return;
        apopt::GradientOptions options;
        m_lastResult = apopt::optimize(*m_objective, *m_plan, m_currentU, options);
        m_currentU = m_lastResult.u;
        m_hasResult = true;
        reDraw();
    }

    std::vector<gui::Point> computeAttenuatedRangeHull(const apopt::Point2D& ap, double maxRadius, int numRays = 180) const {
        std::vector<gui::Point> polygonPts;
        if (!m_plan) return polygonPts;
        polygonPts.reserve(numRays + 1);

        const double stepSize = 0.05;
        const double cs = m_plan->cellSize();

        for (int i = 0; i < numRays; ++i) {
            double angle = (2.0 * M_PI * i) / numRays;
            double dirX = std::cos(angle);
            double dirY = std::sin(angle);

            double distTraveled = 0.0;
            double remainingBudget = maxRadius;
            double lastValidDist = 0.0;

            while (distTraveled < maxRadius && remainingBudget > 0.0) {
                distTraveled += stepSize;

                double currX = ap.x + dirX * distTraveled;
                double currY = ap.y + dirY * distTraveled;

                int gx = static_cast<int>(currX / cs);
                int gy = static_cast<int>(currY / cs);

                if (gx < 0 || gx >= m_plan->width() || gy < 0 || gy >= m_plan->height()) {
                    distTraveled = lastValidDist;
                    break;
                }

                if (m_plan->isObstacle(gx, gy)) {
                    distTraveled = lastValidDist;
                    break;
                }

                double att = m_plan->attenuationAt(gx, gy);
                if (att > 0.0) {
                    remainingBudget -= stepSize * (1.0 + att * 8.0);
                } else {
                    remainingBudget -= stepSize;
                }

                lastValidDist = distTraveled;
            }

            double finalDist = std::max(0.0, std::min(distTraveled, maxRadius));
            polygonPts.push_back(worldToScreen({ ap.x + dirX * finalDist, ap.y + dirY * finalDist }));
        }

        polygonPts.push_back(polygonPts.front());
        return polygonPts;
    }

    void drawFloorPlan() {
        if (!m_plan) {
            const char* msg{"Click \"Generate Floor Plan\" to begin"};
            gui::DrawableString::draw(msg, strlen(msg),
                gui::Rect(leftZoneLeft, leftZoneTop, leftZoneLeft + leftZoneWidth, leftZoneTop + leftZoneHeight),
                gui::Font::ID::SystemNormal, td::ColorID::Black, td::TextAlignment::Center, td::VAlignment::Center);
            return;
        }

        gui::CoordType cellPx{cellPixelSize()};
        gui::Point origin{gridOrigin()};

        std::vector<double> coverage;
        if (m_hasResult && m_objective) {
            coverage = m_objective->perCellCoverage(m_currentU);
        }

        std::size_t evalIdx{0};
        for (int gy = 0; gy < m_plan->height(); ++gy) {
            for (int gx = 0; gx < m_plan->width(); ++gx) {
                gui::CoordType x{origin.x + gx * cellPx};
                gui::CoordType y{origin.y + gy * cellPx};
                gui::Rect cellRect(x, y, x + cellPx, y + cellPx);

                if (m_plan->isObstacle(gx, gy)) {
                    gui::Shape obstacle;
                    obstacle.createRect(cellRect);
                    obstacle.drawFill(td::ColorID::Berry);
                } else if (m_plan->isFree(gx, gy)) {
                    gui::Shape cell;
                    cell.createRect(cellRect);
                    if (m_hasResult && evalIdx < coverage.size()) {
                        double cellSignal = coverage[evalIdx];
                        if (cellSignal >= 0.5) {
                            cell.drawFill(td::ColorID::ForestGreen);
                        } else if (cellSignal >= 0.1) {
                            cell.drawFill(td::ColorID::PaleGreen);
                        } else {
                            cell.drawFill(td::ColorID::DarkGray);
                        }
                    } else {
                        cell.drawFill(td::ColorID::DarkGray);
                    }
                    ++evalIdx;
                } else if (m_plan->attenuationAt(gx, gy) > 0.0) {
                    gui::Shape wall;
                    wall.createRect(cellRect);
                    wall.drawFill(td::ColorID::Gray);
                }
            }
        }

        for (const auto& room : m_plan->rooms()) {
            gui::CoordType rx0 = origin.x + room.bounds.x0 * cellPx;
            gui::CoordType ry0 = origin.y + room.bounds.y0 * cellPx;
            gui::CoordType rx1 = origin.x + room.bounds.x1 * cellPx;
            gui::CoordType ry1 = origin.y + room.bounds.y1 * cellPx;

            gui::Rect roomRect(rx0, ry0, rx1, ry1);
            gui::Shape roomBorder;
            roomBorder.createRect(roomRect);
            roomBorder.drawWire(td::ColorID::MistyRose, 1.5f);

            gui::DrawableString::draw(room.name.c_str(), room.name.length(), roomRect,
                gui::Font::ID::SystemSmallerBold, td::ColorID::MistyRose,
                td::TextAlignment::Center, td::VAlignment::Center);
        }

        gui::Shape border;
        border.createRect(gui::Rect(origin.x, origin.y,
                                     origin.x + cellPx * m_plan->width(),
                                     origin.y + cellPx * m_plan->height()));
        border.drawWire(td::ColorID::Berry, 2.0f);

        std::size_t n = m_currentU.size() / 2;

        double effRange = m_objective ? m_objective->effectiveRangeForCount(n) : m_apRange;

        for (std::size_t i = 0; i < n; ++i) {
            apopt::Point2D w{ m_currentU[2 * i], m_currentU[2 * i + 1] };
            auto hull = computeAttenuatedRangeHull(w, effRange, 180);

            if (hull.size() > 1) {
                std::vector<gui::Point> lineSegments;
                lineSegments.reserve((hull.size() - 1) * 2);
                for (std::size_t k = 0; k + 1 < hull.size(); ++k) {
                    lineSegments.push_back(hull[k]);
                    lineSegments.push_back(hull[k + 1]);
                }

                gui::Shape coverageOutline;
                coverageOutline.createLines(lineSegments.data(), lineSegments.size(), 1.5f);
                coverageOutline.drawWire(td::ColorID::Black);
            }

            gui::Point p = worldToScreen(w);
            gui::CoordType r = std::max<gui::CoordType>(8, cellPx * 0.45);
            gui::Rect apRect(p.x - r, p.y - r, p.x + r, p.y + r);

            gui::Shape apMarker;
            apMarker.createRoundedRect(apRect, r);
            apMarker.drawFill(td::ColorID::White);
            apMarker.drawWire(td::ColorID::Black, 1.5f);

            char label[32];
            snprintf(label, sizeof(label), "AP-%02zu", i + 1);
            gui::DrawableString::draw(label, strlen(label), apRect,
                gui::Font::ID::SystemSmallerBold, td::ColorID::Black,
                td::TextAlignment::Center, td::VAlignment::Center);
        }
    }

    void drawSubplot(gui::CoordType x, gui::CoordType y, gui::CoordType w, gui::CoordType h,
                     const char* title, const char* yLabel,
                     const std::vector<double>& data, td::ColorID lineColor,
                     double fixedMin = -1.0, double fixedMax = -1.0) {
        gui::Shape bg; bg.createRoundedRect(gui::Rect(x, y, x + w, y + h), 6);
        bg.drawFill(td::ColorID::DarkGray);
        gui::Shape border; border.createRoundedRect(gui::Rect(x, y, x + w, y + h), 6);
        border.drawWire(td::ColorID::Berry, 1.5f);

        gui::DrawableString::draw(title, strlen(title), gui::Rect(x + 10, y + 4, x + w - 10, y + 20),
            gui::Font::ID::SystemSmallerBold, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);

        gui::CoordType pLeft{x + 60}, pRight{x + w - 15};
        gui::CoordType pTop{y + 24}, pBottom{y + h - 22};

        gui::Shape plotBox;
        plotBox.createRect(gui::Rect(pLeft, pTop, pRight, pBottom));
        plotBox.drawFill(td::ColorID::Black);

        for (int i{1}; i <= 3; ++i) {
            gui::CoordType gx{pLeft + (pRight - pLeft) * i / 4};
            gui::Point vPts[2] = { {gx, pTop}, {gx, pBottom} };
            gui::Shape vl; vl.createLines(vPts, 2, 1.0f);
            vl.drawWire(td::ColorID::DimGray);

            gui::CoordType gy{pTop + (pBottom - pTop) * i / 4};
            gui::Point hPts[2] = { {pLeft, gy}, {pRight, gy} };
            gui::Shape hl; hl.createLines(hPts, 2, 1.0f);
            hl.drawWire(td::ColorID::DimGray);
        }

        plotBox.drawWire(td::ColorID::Gray, 1.0f);

        gui::DrawableString::draw("Iter", 4, gui::Rect(pLeft, pBottom + 2, pRight, y + h - 2),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Center, td::VAlignment::Top);

        if (!m_hasResult || data.empty()) {
            const char* msg{"No Data"};
            gui::DrawableString::draw(msg, strlen(msg), gui::Rect(pLeft, pTop, pRight, pBottom),
                gui::Font::ID::SystemSmallerBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);
            return;
        }

        double minV{fixedMin >= 0.0 ? fixedMin : *std::min_element(data.begin(), data.end())};
        double maxV{fixedMax >= 0.0 ? fixedMax : *std::max_element(data.begin(), data.end())};
        if (maxV <= minV) {
            maxV = minV + 1.0;
        }
        double span{maxV - minV};

        char topLabelBuf[16], midLabelBuf[16], botLabelBuf[16];
        snprintf(topLabelBuf, sizeof(topLabelBuf), "%.2f", maxV);
        snprintf(midLabelBuf, sizeof(midLabelBuf), "%.2f", minV + span * 0.5);
        snprintf(botLabelBuf, sizeof(botLabelBuf), "%.2f", minV);

        gui::DrawableString::draw(topLabelBuf, strlen(topLabelBuf), gui::Rect(x + 2, pTop - 6, pLeft - 5, pTop + 10),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Right, td::VAlignment::Top);
        gui::DrawableString::draw(midLabelBuf, strlen(midLabelBuf), gui::Rect(x + 2, (pTop + pBottom) / 2 - 8, pLeft - 5, (pTop + pBottom) / 2 + 8),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Right, td::VAlignment::Center);
        gui::DrawableString::draw(botLabelBuf, strlen(botLabelBuf), gui::Rect(x + 2, pBottom - 10, pLeft - 5, pBottom + 6),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Right, td::VAlignment::Bottom);

        std::size_t maxIterIndex = data.size() > 0 ? data.size() - 1 : 0;
        std::size_t xMaxRange = maxIterIndex + 3;

        char startXBuf[8]{"0"};
        char endXBuf[16];
        snprintf(endXBuf, sizeof(endXBuf), "%zu", xMaxRange);

        gui::DrawableString::draw(startXBuf, strlen(startXBuf), gui::Rect(pLeft - 10, pBottom + 2, pLeft + 20, y + h - 2),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);
        gui::DrawableString::draw(endXBuf, strlen(endXBuf), gui::Rect(pRight - 30, pBottom + 2, pRight + 10, y + h - 2),
            gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Right, td::VAlignment::Top);

        std::vector<gui::Point> pts;
        pts.reserve(data.size());
        for (std::size_t i{0}; i < data.size(); ++i) {
            gui::CoordType px = pLeft + static_cast<gui::CoordType>((pRight - pLeft) * (double(i) / double(xMaxRange)));
            gui::CoordType py = pBottom - static_cast<gui::CoordType>((pBottom - pTop) * ((data[i] - minV) / span));
            pts.push_back({px, py});
        }

        if (pts.size() == 1) {
            gui::Point singleSegment[2] = { pts[0], pts[0] };
            gui::Shape singlePoint;
            singlePoint.createLines(singleSegment, 2, 3.0f);
            singlePoint.drawWire(lineColor);
            return;
        }

        std::vector<gui::Point> segments;
        segments.reserve((pts.size() - 1) * 2);
        for (std::size_t i{0}; i + 1 < pts.size(); ++i) {
            segments.push_back(pts[i]);
            segments.push_back(pts[i + 1]);
        }

        gui::Shape line;
        line.createLines(segments.data(), segments.size(), 2.5f);
        line.drawWire(lineColor);
    }

    void drawDualConvergencePlots(gui::CoordType x, gui::CoordType y, gui::CoordType width, gui::CoordType height) {
        gui::CoordType gap{12};
        gui::CoordType subWidth{(width - gap) / 2};

        drawSubplot(x, y, subWidth, height, "Coverage vs. Iteration", "Cov",
                    m_lastResult.objectiveHistory, td::ColorID::Green);

        drawSubplot(x + subWidth + gap, y, subWidth, height, "Line Search Step Size", "Alpha",
                    m_lastResult.stepSizeHistory, td::ColorID::Green, 0.0, 1.0);
    }

    void drawLayoutDropdownOverlay(gui::CoordType x, gui::CoordType y, gui::CoordType width) {
        if (!m_layoutDropdownExpanded) return;

        static const apopt::LayoutType kOptions[kLayoutCount]{
            apopt::LayoutType::OpenRoom, apopt::LayoutType::Corridor,
            apopt::LayoutType::LShape, apopt::LayoutType::OfficeComplex
        };
        gui::CoordType itemH{36};
        gui::CoordType menuY{y};
        gui::Shape menuBg; menuBg.createRoundedRect(gui::Rect(x, menuY, x + width, menuY + kLayoutCount * itemH), 6);
        menuBg.drawFill(td::ColorID::Berry);
        gui::Shape menuBorder; menuBorder.createRoundedRect(gui::Rect(x, menuY, x + width, menuY + kLayoutCount * itemH), 6);
        menuBorder.drawWire(td::ColorID::White, 1);

        for (std::size_t i = 0; i < kLayoutCount; ++i) {
            gui::CoordType iy{menuY + static_cast<gui::CoordType>(i) * itemH};
            m_layoutDropdownItemRects[i] = gui::Rect(x, iy, x + width, iy + itemH);
            if (kOptions[i] == m_layoutType) {
                gui::Shape hi; hi.createRect(gui::Rect(x + 3, iy + 2, x + width - 3, iy + itemH - 2));
                hi.drawFill(td::ColorID::LavenderBlush);
            }
            const char* n{layoutName(kOptions[i])};
            gui::DrawableString::draw(n, strlen(n), gui::Rect(x + 15, iy, x + width - 15, iy + itemH),
                gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Left, td::VAlignment::Center);
        }
    }

    void drawControlPanel() {
        gui::CoordType x = rightZoneLeft;
        gui::CoordType y = rightZoneTop;
        gui::CoordType w = rightZoneWidth;

        // Generate Floor Plan Button
        m_generateButtonRect = gui::Rect(x, y, x + w, y + 36);
        gui::Shape genBtn;
        genBtn.createRoundedRect(m_generateButtonRect, 6);
        genBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("Generate Floor Plan", 19, m_generateButtonRect,
            gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // Floor Plan Dropdown Selector
        y += 44;
        m_layoutDropdownRect = gui::Rect(x, y, x + w, y + 36);
        gui::Shape comboBtn;
        comboBtn.createRoundedRect(m_layoutDropdownRect, 6);
        comboBtn.drawFill(td::ColorID::Berry);
        comboBtn.drawWire(td::ColorID::White, 1.0f);

        const char* presetName = layoutName(m_layoutType);
        gui::DrawableString::draw(presetName, strlen(presetName),
            gui::Rect(x + 15, y, x + w - 30, y + 36),
            gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Left, td::VAlignment::Center);
        const char* arrow = m_layoutDropdownExpanded ? "^" : "v";
        gui::DrawableString::draw(arrow, 1,
            gui::Rect(x + w - 25, y, x + w - 10, y + 36),
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // Reset APs Button
        y += 44;
        m_randomizeButtonRect = gui::Rect(x, y, x + w, y + 34);
        gui::Shape randBtn;
        randBtn.createRoundedRect(m_randomizeButtonRect, 6);
        randBtn.drawFill(td::ColorID::Berry);
        randBtn.drawWire(td::ColorID::White, 0.8f);
        gui::DrawableString::draw("Randomize AP Positions", 22, m_randomizeButtonRect,
            gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // Power Budget Controls
        y += 42;
        gui::CoordType btnWidth = 35;

        m_budgetMinusRect = gui::Rect(x, y, x + btnWidth, y + 32);
        gui::Shape budgetMinusBtn;
        budgetMinusBtn.createRoundedRect(m_budgetMinusRect, 6);
        budgetMinusBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("-", 1, m_budgetMinusRect,
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        char budgetTotalBuf[32];
        snprintf(budgetTotalBuf, sizeof(budgetTotalBuf), "Power Budget: %.0fmW", m_powerBudget);
        gui::Rect budgetTotalLabelRect(x + btnWidth, y, x + w - btnWidth, y + 32);
        gui::DrawableString::draw(budgetTotalBuf, strlen(budgetTotalBuf), budgetTotalLabelRect,
            gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        m_budgetPlusRect = gui::Rect(x + w - btnWidth, y, x + w, y + 32);
        gui::Shape budgetPlusBtn;
        budgetPlusBtn.createRoundedRect(m_budgetPlusRect, 6);
        budgetPlusBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("+", 1, m_budgetPlusRect,
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // AP Count Controls
        y += 40;

        m_apCountMinusRect = gui::Rect(x, y, x + btnWidth, y + 32);
        gui::Shape minusBtn;
        minusBtn.createRoundedRect(m_apCountMinusRect, 6);
        minusBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("-", 1, m_apCountMinusRect,
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        char apBuf[32];
        snprintf(apBuf, sizeof(apBuf), "APs: %d", m_numAPs);
        gui::Rect apLabelRect(x + btnWidth, y, x + w - btnWidth, y + 32);
        gui::DrawableString::draw(apBuf, strlen(apBuf), apLabelRect,
            gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        m_apCountPlusRect = gui::Rect(x + w - btnWidth, y, x + w, y + 32);
        gui::Shape plusBtn;
        plusBtn.createRoundedRect(m_apCountPlusRect, 6);
        plusBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("+", 1, m_apCountPlusRect,
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // Run Optimization Button
        y += 48;
        m_runButtonRect = gui::Rect(x, y, x + w, y + 40);
        gui::Shape runBtn;
        runBtn.createRoundedRect(m_runButtonRect, 6);
        runBtn.drawFill(td::ColorID::Berry);
        gui::DrawableString::draw("Run Optimization", 16, m_runButtonRect,
            gui::Font::ID::SystemBold, td::ColorID::White, td::TextAlignment::Center, td::VAlignment::Center);

        // Optimization Results Card Panel
        y += 50;
        gui::CoordType cardHeight = 220;
        gui::Rect cardRect(x, y, x + w, y + cardHeight);

        gui::Shape cardBg;
        cardBg.createRoundedRect(cardRect, 8);
        cardBg.drawFill(td::ColorID::Berry);

        gui::Shape cardBorder;
        cardBorder.createRoundedRect(cardRect, 8);
        cardBorder.drawWire(td::ColorID::Berry, 1.5f);

        gui::CoordType lineY = y + 10;
        gui::CoordType marginX = x + 12;

        double powerPerAp = m_numAPs > 0 ? m_powerBudget / m_numAPs : 0.0;
        double effRange = m_objective
            ? m_objective->effectiveRangeForCount(static_cast<std::size_t>(m_numAPs))
            : m_apRange;

        char powerPerApBuf[64];
        snprintf(powerPerApBuf, sizeof(powerPerApBuf), "Power / AP: %.0f mW", powerPerAp);
        gui::DrawableString::draw(powerPerApBuf, strlen(powerPerApBuf),
            gui::Rect(marginX, lineY, x + w - 12, lineY + 16),
            gui::Font::ID::SystemSmaller, td::ColorID::LightGray, td::TextAlignment::Left, td::VAlignment::Top);
        lineY += 16;

        char effRangeBuf[64];
        snprintf(effRangeBuf, sizeof(effRangeBuf), "Effective Range: %.1f m", effRange);
        gui::DrawableString::draw(effRangeBuf, strlen(effRangeBuf),
            gui::Rect(marginX, lineY, x + w - 12, lineY + 18),
            gui::Font::ID::SystemSmaller, td::ColorID::LightGray, td::TextAlignment::Left, td::VAlignment::Top);
        lineY += 22;

        if (!m_hasResult) {
            const char* readyMsg = "Ready for optimization";
            gui::DrawableString::draw(readyMsg, strlen(readyMsg),
                gui::Rect(marginX, lineY, x + w - 12, lineY + 25),
                gui::Font::ID::SystemNormal, td::ColorID::White, td::TextAlignment::Left, td::VAlignment::Top);
            return;
        }

        double cs = m_plan ? m_plan->cellSize() : 1.0;
        double cellArea = cs * cs;

        std::size_t totalCells = m_objective ? m_objective->evalPoints().size() : 0;
        double coveredAreaM2 = m_lastResult.value * cellArea;
        double totalAreaM2 = static_cast<double>(totalCells) * cellArea;

        double coveragePct = (totalAreaM2 > 0.0) ? (coveredAreaM2 / totalAreaM2) * 100.0 : 0.0;

        char covBuf[96];
        snprintf(covBuf, sizeof(covBuf), "Coverage: %.2f / %.2f m² (%.1f%%)", coveredAreaM2, totalAreaM2, coveragePct);
        gui::DrawableString::draw(covBuf, strlen(covBuf),
            gui::Rect(marginX, lineY, x + w - 12, lineY + 22),
            gui::Font::ID::SystemBold, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);
        lineY += 22;

        char iterBuf[64];
        snprintf(iterBuf, sizeof(iterBuf), "Iterations: %d", m_lastResult.iterations);
        gui::DrawableString::draw(iterBuf, strlen(iterBuf),
            gui::Rect(marginX, lineY, x + w - 12, lineY + 18),
            gui::Font::ID::SystemNormal, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);
        lineY += 18;

        const char* statusText = "";
        td::ColorID statusColor = td::ColorID::Black;

        switch (m_lastResult.reason) {
            case apopt::ConvergenceReason::GradientNorm:
                statusText = "Converged: Local peak reached";
                statusColor = td::ColorID::Green;
                break;

            case apopt::ConvergenceReason::ObjectiveStall:
                statusText = "Stopped: Coverage gain plateaued";
                statusColor = td::ColorID::Yellow;
                break;

            case apopt::ConvergenceReason::LineSearchFailed:
                statusText = "Stopped: Step size vanished";
                statusColor = td::ColorID::Yellow;
                break;

            case apopt::ConvergenceReason::MaxIterations:
                statusText = "Stopped: Max iterations reached";
                statusColor = td::ColorID::Black;
                break;
        }

        gui::DrawableString::draw(statusText, strlen(statusText),
            gui::Rect(marginX, lineY, x + w - 12, lineY + 18),
            gui::Font::ID::SystemSmaller, statusColor, td::TextAlignment::Left, td::VAlignment::Top);
        lineY += 20;

        std::size_t numAPs = m_lastResult.u.size() / 2;
        for (std::size_t i = 0; i < numAPs && lineY + 14 <= cardRect.bottom - 6; ++i) {
            char apCoordsBuf[64];
            snprintf(apCoordsBuf, sizeof(apCoordsBuf), "AP %zu: (%.1f, %.1f)", i + 1, m_lastResult.u[2 * i], m_lastResult.u[2 * i + 1]);
            gui::DrawableString::draw(apCoordsBuf, strlen(apCoordsBuf),
                gui::Rect(marginX, lineY, x + w - 12, lineY + 14),
                gui::Font::ID::SystemSmaller, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);
            lineY += 14;
        }

        // Convergence Explanation Box
        y += cardHeight + 10;
        gui::CoordType expHeight = 150;
        gui::Rect expRect(x, y, x + w, y + expHeight);

        gui::Shape expBg;
        expBg.createRoundedRect(expRect, 8);
        expBg.drawFill(td::ColorID::DarkGray);

        gui::Shape expBorder;
        expBorder.createRoundedRect(expRect, 8);
        expBorder.drawWire(td::ColorID::Berry, 1.0f);

        gui::DrawableString::draw("Optimization Status Details:", strlen("Optimization Status Details:"),
            gui::Rect(x + 10, y + 6, x + w - 10, y + 22),
            gui::Font::ID::SystemSmallerBold, td::ColorID::Black, td::TextAlignment::Left, td::VAlignment::Top);

        std::string explanationText = "";

        switch (m_lastResult.reason) {
            case apopt::ConvergenceReason::GradientNorm:
                explanationText =
                    "The gradient norm fell below tolerance. The optimizer reached a "
                    "stationary point (local peak) where marginal shifts yield zero net gain "
                    "within signal range and obstacle constraints.";
                break;

            case apopt::ConvergenceReason::ObjectiveStall:
                explanationText =
                    "Objective improvement plateaued. APs settled into local basins "
                    "shaped by wall attenuation and power range limits.";
                break;

            case apopt::ConvergenceReason::LineSearchFailed:
                explanationText =
                    "Line search step size vanished (alpha -> 0). Backtracking failed because "
                    "wall attenuation or range cutoffs created sharp non-linear signal drops.";
                break;

            case apopt::ConvergenceReason::MaxIterations:
                explanationText =
                    "Reached maximum iteration cap before convergence criteria were met.";
                break;
        }

        drawWrappedString(explanationText, gui::Rect(x + 10, y + 24, x + w - 10, y + expHeight - 6),
                          gui::Font::ID::SystemSmaller, td::ColorID::Black, 14, 32);
    }

protected:
    void onDraw(const gui::Rect& rect) override {
        gui::Shape bg; bg.createRect(rect); bg.drawFill(td::ColorID::Black);
        drawFloorPlan();
        drawDualConvergencePlots(leftZoneLeft, plotTop, leftZoneWidth, plotHeight);
        drawControlPanel();
        drawLayoutDropdownOverlay(rightZoneLeft, rightZoneTop + 80, rightZoneWidth);
    }

    void onResize(const gui::Size& newSize) override {
        leftZoneLeft = static_cast<gui::CoordType>(newSize.width * 0.025);
        leftZoneTop = static_cast<gui::CoordType>(newSize.height * 0.025);
        rightZoneWidth = static_cast<gui::CoordType>(newSize.width * 0.26);
        gui::CoordType gap = static_cast<gui::CoordType>(newSize.width * 0.02);

        leftZoneWidth = newSize.width - leftZoneLeft - rightZoneWidth - gap - static_cast<gui::CoordType>(newSize.width * 0.025);

        plotHeight = std::max<gui::CoordType>(180, static_cast<gui::CoordType>(newSize.height * 0.28));
        leftZoneHeight = newSize.height - leftZoneTop * 2 - plotHeight - 15;
        plotTop = leftZoneTop + leftZoneHeight + 15;

        rightZoneLeft = leftZoneLeft + leftZoneWidth + gap;
        rightZoneTop = leftZoneTop;
        reDraw();
    }

    void onPrimaryButtonPressed(const gui::InputDevice& inputDevice) override {
        gui::Point click{inputDevice.getModelPoint()};

        if (m_layoutDropdownExpanded) {
            static const apopt::LayoutType kOptions[kLayoutCount]{
                apopt::LayoutType::OpenRoom, apopt::LayoutType::Corridor, 
                apopt::LayoutType::LShape, apopt::LayoutType::OfficeComplex
            };
            for (std::size_t i = 0; i < kLayoutCount; ++i) {
                if (m_layoutDropdownItemRects[i].contains(click)) {
                    m_layoutType = kOptions[i];
                    m_layoutDropdownExpanded = false;
                    generateFloorPlan();
                    return;
                }
            }
            m_layoutDropdownExpanded = false;
            reDraw();
            return;
        }

        if (m_layoutDropdownRect.contains(click)) {
            m_layoutDropdownExpanded = true;
            reDraw();
            return;
        }

        if (m_generateButtonRect.contains(click)) { 
            m_seedCounter++;
            generateFloorPlan(); 
            return; 
        }

        if (m_randomizeButtonRect.contains(click)) { 
            randomizeApPositions(); 
            return; 
        }

        if (m_apCountMinusRect.contains(click)) { 
            m_numAPs = std::max(1, m_numAPs - 1); 
            randomizeApPositions(); 
            return; 
        }
        if (m_apCountPlusRect.contains(click)) { 
            m_numAPs = std::min(kMaxAPs, m_numAPs + 1); 
            randomizeApPositions(); 
            return; 
        }

        if (m_budgetMinusRect.contains(click)) {
            m_powerBudget = std::max(50.0, m_powerBudget - 50.0);
            if (m_plan) {
                m_objective.emplace(*m_plan, currentSignalParams());
            }
            m_hasResult = false;
            m_lastResult = apopt::OptimizationResult{};
            reDraw();
            return;
        }
        if (m_budgetPlusRect.contains(click)) {
            m_powerBudget = std::min(2000.0, m_powerBudget + 50.0);
            if (m_plan) {
                m_objective.emplace(*m_plan, currentSignalParams());
            }
            m_hasResult = false;
            m_lastResult = apopt::OptimizationResult{};
            reDraw();
            return;
        }

        if (m_runButtonRect.contains(click) && m_plan.has_value()) { 
            runOptimization(); 
            return; 
        }
    }

public:
    WAPCanvas() : gui::Canvas({ gui::InputDevice::Event::PrimaryClicks }) {
        enableResizeEvent(true);
    }

    void init() { generateFloorPlan(); }
};