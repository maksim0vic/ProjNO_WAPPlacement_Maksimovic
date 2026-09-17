#include "Objective.h"
#include <algorithm>
#include <cmath>

namespace apopt {

Objective::Objective(const FloorPlan& plan, SignalParams params)
    : m_plan{plan}
    , m_params{params}
{
    for (int gy = 0; gy < plan.height(); ++gy) {
        for (int gx = 0; gx < plan.width(); ++gx) {
            if (plan.isFree(gx, gy)) {
                m_evalPoints.push_back(plan.gridToWorld(gx, gy));
            }
        }
    }
}

double Objective::effectiveRangeForCount(std::size_t n) const
{
    if (n == 0) return 0.0;
    
    double powerPerAp = m_params.totalPowerBudget / static_cast<double>(n);
    if (m_params.referencePower <= 0.0) return m_params.referenceRange;
    
    // Range scales with the square root of transmit power (mW) relative to reference power
    return m_params.referenceRange * std::sqrt(powerPerAp / m_params.referencePower);
}

double Objective::calculateSignal(double dist, double wallAttenuation, double effectiveRange) const
{
    if (effectiveRange <= 0.0) return 0.0;

    double k = 6.0 / effectiveRange;
    double baseSignal = 1.0 / (1.0 + std::exp(k * (dist - 0.75 * effectiveRange)));
    double attFactor = std::exp(-wallAttenuation);
    return baseSignal * attFactor;
}

double Objective::evaluate(const std::vector<double>& u) const
{
    std::vector<double> perCell = perCellCoverage(u);
    double total{0.0};
    for (double c : perCell) {
        total += c;
    }

    std::size_t numAPs = u.size() / 2;
    if (numAPs == 0) return 0.0;

    double effectiveRange = effectiveRangeForCount(numAPs);
    double repulsionPenalty = 0.0;
    double minSep = effectiveRange * 0.8;

    for (std::size_t i = 0; i < numAPs; ++i) {
        for (std::size_t j = i + 1; j < numAPs; ++j) {
            double dx = u[2 * i] - u[2 * j];
            double dy = u[2 * i + 1] - u[2 * j + 1];
            double dist = std::hypot(dx, dy);

            if (dist < minSep && dist > 1e-4) {
                // Penalize overlapping APs quadratically based on proximity
                repulsionPenalty += 2.0 * std::pow(minSep - dist, 2);
            }
        }
    }
    return total - repulsionPenalty;
}

std::vector<double> Objective::perCellCoverage(const std::vector<double>& u) const
{
    std::size_t n = u.size() / 2; // number of APs
    if (n == 0) {
        return std::vector<double>(m_evalPoints.size(), 0.0);
    }

    double effectiveRange = effectiveRangeForCount(n);

    std::vector<double> perCell;
    perCell.reserve(m_evalPoints.size());

    for (const Point2D& cell : m_evalPoints) {
        double missProbability = 1.0;

        for (std::size_t i = 0; i < n; ++i) {
            double apX = u[2 * i];
            double apY = u[2 * i + 1];

            double dist = std::hypot(cell.x - apX, cell.y - apY);
            double wallAtt = m_plan.lineOfSightAttenuation(apX, apY, cell.x, cell.y);

            double signal = calculateSignal(dist, wallAtt, effectiveRange);

            // Joint non-coverage probability
            missProbability *= (1.0 - signal);
        }
        // Cell is covered if at least one AP reaches it
        perCell.push_back(1.0 - missProbability);
    }

    return perCell;
}

} // namespace apopt