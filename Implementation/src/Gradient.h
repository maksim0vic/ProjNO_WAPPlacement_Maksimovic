#pragma once
#include "FloorPlan.h"
#include "Objective.h"
#include <vector>

namespace apopt {

struct GradientOptions {
    int maxIterations{200};

    // stop once ||grad f(u)|| drops below 
    double gradientNormTolerance{1e-4};

    //  stop once a step improves f(u) by less than this
    double objectiveImprovementTolerance{1e-6};

    // Backtracking line search (Armijo, on the ascent direction)
    double initialStepSize{1.0};
    double backtrackShrink{0.5};              
    double backtrackSufficientDecrease{1e-4}; 
    int maxBacktrackSteps{30};

    double finiteDifferenceEpsilon{1e-4};
};

enum class ConvergenceReason {
    MaxIterations,
    GradientNorm,
    ObjectiveStall,
    LineSearchFailed // alpha shrank to nothing without improving f(u)
};

struct OptimizationResult {
    std::vector<double> u;   // final AP positions 
    double value{0.0};       // f(u) at the returned point
    int iterations{0};       // completed outer iterations
    ConvergenceReason reason{ConvergenceReason::MaxIterations};
    std::vector<double> objectiveHistory; 
    std::vector<double> stepSizeHistory;
};

std::vector<double> numericGradient(const Objective& obj,
                                     const std::vector<double>& u,
                                     double epsilon = 1e-4);

void projectToFeasibleRegion(std::vector<double>& u, const FloorPlan& plan);

double backtrackingLineSearch(const Objective& obj,
                               const FloorPlan& plan,
                               const std::vector<double>& u,
                               const std::vector<double>& gradient,
                               double currentValue,
                               const GradientOptions& options,
                               std::vector<double>& outNextU,
                               double& outNextValue);


OptimizationResult optimize(const Objective& obj,
                             const FloorPlan& plan,
                             const std::vector<double>& initialU,
                             const GradientOptions& options = {});

} 