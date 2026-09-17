#include "Gradient.h"
#include <algorithm>
#include <cmath>

namespace apopt {

std::vector<double> numericGradient(const Objective& obj, const std::vector<double>& u, double epsilon)
{
    std::vector<double> grad(u.size(), 0.0);
    std::vector<double> uPlus{u};
    std::vector<double> uMinus{u};

    for (std::size_t i{0}; i < u.size(); ++i) {
        uPlus[i] = u[i] + epsilon;
        uMinus[i] = u[i] - epsilon;

        double fPlus{obj.evaluate(uPlus)};
        double fMinus{obj.evaluate(uMinus)};
        double diff{fPlus - fMinus};

        if (std::isfinite(diff)) {
            grad[i] = diff / (2.0 * epsilon);
        } else {
            grad[i] = 0.0;
        }

        uPlus[i] = u[i];
        uMinus[i] = u[i];
    }

    return grad;
}

void projectToFeasibleRegion(std::vector<double>& u, const FloorPlan& plan)
{
    double maxX{plan.width() * plan.cellSize()};
    double maxY{plan.height() * plan.cellSize()};

    std::size_t n{u.size() / 2};
    for (std::size_t i{0}; i < n; ++i) {
        double& x{u[2 * i]};
        double& y{u[2 * i + 1]};

        if (!std::isfinite(x)) { x = maxX * 0.5; }
        if (!std::isfinite(y)) { y = maxY * 0.5; }

        x = std::clamp(x, 0.0, std::nextafter(maxX, 0.0));
        y = std::clamp(y, 0.0, std::nextafter(maxY, 0.0));

        if (plan.isFreeWorld(x, y)) {
            continue;
        }

        int gx{0}, gy{0};
        plan.worldToGrid(Point2D{ x, y }, gx, gy);
        gx = std::clamp(gx, 0, plan.width() - 1);
        gy = std::clamp(gy, 0, plan.height() - 1);

        int bestGx{gx};
        int bestGy{gy};
        bool found{false};
        int maxRadius{std::max(plan.width(), plan.height())};

        for (int radius{1}; radius <= maxRadius && !found; ++radius) {
            for (int dy{-radius}; dy <= radius && !found; ++dy) {
                for (int dx{-radius}; dx <= radius && !found; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != radius) {
                        continue;
                    }
                    int nx{gx + dx};
                    int ny{gy + dy};
                    if (plan.isFree(nx, ny)) {
                        bestGx = nx;
                        bestGy = ny;
                        found = true;
                        break;
                    }
                }
            }
        }

        if (found) {
            Point2D snapped{plan.gridToWorld(bestGx, bestGy)};
            x = snapped.x;
            y = snapped.y;
        }
    }
}

double backtrackingLineSearch(const Objective& obj,
                               const FloorPlan& plan,
                               const std::vector<double>& u,
                               const std::vector<double>& gradient,
                               double currentValue,
                               const GradientOptions& options,
                               std::vector<double>& outNextU,
                               double& outNextValue)
{
    double gradNormSq{0.0};
    for (double g : gradient) {
        if (std::isfinite(g)) {
            gradNormSq += g * g;
        }
    }

    if (!std::isfinite(gradNormSq) || gradNormSq <= 0.0) {
        outNextU = u;
        outNextValue = currentValue;
        return 0.0;
    }

    double alpha{options.initialStepSize}; // reset to initial step size per iteration

    for (int step{0}; step < options.maxBacktrackSteps; ++step) {
        std::vector<double> candidate{u};
        for (std::size_t i{0}; i < u.size(); ++i) {
            candidate[i] += alpha * gradient[i];
        }
        projectToFeasibleRegion(candidate, plan);

        double candidateValue{obj.evaluate(candidate)};

        if (std::isfinite(candidateValue) &&
            candidateValue >= currentValue + options.backtrackSufficientDecrease * alpha * gradNormSq) {
            outNextU = candidate;
            outNextValue = candidateValue;
            return alpha;
        }

        alpha *= 0.7; 
    }

    outNextU = u;
    outNextValue = currentValue;
    return 0.0;
}

OptimizationResult optimize(const Objective& obj,
                             const FloorPlan& plan,
                             const std::vector<double>& initialU,
                             const GradientOptions& options)
{
    OptimizationResult result{};
    result.reason = ConvergenceReason::MaxIterations;

    std::vector<double> u{initialU};
    projectToFeasibleRegion(u, plan);

    double value{obj.evaluate(u)};
    result.objectiveHistory.push_back(value);

    int iter{0};
    for (; iter < options.maxIterations; ++iter) {
        std::vector<double> grad{numericGradient(obj, u, options.finiteDifferenceEpsilon)};

        double gradNormSq{0.0};
        for (double g : grad) {
            if (std::isfinite(g)) {
                gradNormSq += g * g;
            }
        }
        double gradNorm{std::sqrt(gradNormSq)};

        if (!std::isfinite(gradNorm) || gradNorm < options.gradientNormTolerance) {
            result.reason = ConvergenceReason::GradientNorm;
            break;
        }

        std::vector<double> nextU;
        double nextValue{0.0};
        double alpha{backtrackingLineSearch(obj, plan, u, grad, value, options, nextU, nextValue)};
        result.stepSizeHistory.push_back(alpha);

        if (alpha == 0.0) {
            result.reason = ConvergenceReason::LineSearchFailed;
            ++iter;
            break;
        }

        double improvement{nextValue - value};
        u = nextU;
        value = nextValue;
        result.objectiveHistory.push_back(value);

        if (improvement < options.objectiveImprovementTolerance) {
            result.reason = ConvergenceReason::ObjectiveStall;
            ++iter;
            break;
        }
    }

    result.u = u;
    result.value = value;
    result.iterations = std::min(iter, options.maxIterations);

    return result;
}

} 