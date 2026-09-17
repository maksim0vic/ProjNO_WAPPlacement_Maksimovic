#pragma once

#include "FloorPlan.h"
#include <vector>

namespace apopt {

struct SignalParams {
    double totalPowerBudget{500.0}; // Total power in mW
    double referencePower{100.0};   // Reference power in mW
    double referenceRange{10.0};    // Nominal range in meters for referencePower (100 mW)
};

class Objective {
public:
    Objective(const FloorPlan& plan, SignalParams params = SignalParams{});

    double effectiveRangeForCount(std::size_t n) const;
    double calculateSignal(double dist, double wallAttenuation, double effectiveRange) const;

    double evaluate(const std::vector<double>& u) const;
    std::vector<double> perCellCoverage(const std::vector<double>& u) const;

    const std::vector<Point2D>& evalPoints() const { return m_evalPoints; }
    const SignalParams& params() const { return m_params; }

private:
    const FloorPlan& m_plan;
    SignalParams m_params;
    std::vector<Point2D> m_evalPoints;
};

} // namespace apopt