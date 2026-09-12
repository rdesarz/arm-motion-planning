#include <Eigen/Core>
#include <algorithm>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/constraints/box-constraint.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/multibody-free-fwd.hpp>
#include <aligator/modelling/multibody/frame-translation.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/modelling/state-error.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/utils/rollout.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <system_error>
#include <utility>
#include <vector>

#include "amp/aligator_reach_planner.hpp"

namespace amp {

namespace {

constexpr std::array<const char*, 6> kJointNames = {"shoulder_pan", "shoulder_lift", "elbow_flex",
                                                    "wrist_flex",   "wrist_roll",    "gripper"};

constexpr std::size_t kArmJointCount = 5;
constexpr std::size_t kMaximumStepCount = 1000;
constexpr double kNominalTimeStepS = 0.02;
constexpr double kExperimentalVelocityLimitRadS = 4.0;
constexpr double kModelActuatorEffortLimitNm = 2.94;
constexpr double kFinalPositionToleranceM = 0.005;
constexpr double kFinalFrameSpeedToleranceMps = 0.02;
constexpr double kJointLimitToleranceRad = 1e-6;
constexpr double kEffortLimitToleranceNm = 1e-6;
constexpr double kDynamicsDefectTolerance = 1e-5;

PlanningError error(const PlanningErrorCode code, std::string message) {
  return PlanningError{.code = code, .message = std::move(message)};
}

double max_limit_violation(const Eigen::VectorXd& values, const Eigen::VectorXd& lower,
                           const Eigen::VectorXd& upper) {
  double violation = 0.0;
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    violation = std::max(violation, lower[index] - values[index]);
    violation = std::max(violation, values[index] - upper[index]);
  }
  return std::max(0.0, violation);
}

}  // namespace

class AligatorReachPlanner::Impl {
 public:
  explicit Impl(pinocchio::Model model) : model_(std::move(model)) {}

  pinocchio::Model model_;
};

AligatorReachPlanner::AligatorReachPlanner(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
AligatorReachPlanner::~AligatorReachPlanner() = default;
AligatorReachPlanner::AligatorReachPlanner(AligatorReachPlanner&&) noexcept = default;
AligatorReachPlanner& AligatorReachPlanner::operator=(AligatorReachPlanner&&) noexcept = default;

std::expected<JointTrajectory, PlanningError> AligatorReachPlanner::plan(
    const ReachRequest& request) const {
  if (!std::isfinite(request.duration_s) || request.duration_s <= 0.0) {
    return std::unexpected(error(PlanningErrorCode::invalid_request,
                                 "duration_s must be finite and strictly positive"));
  }
  if (!request.target_world_m.allFinite()) {
    return std::unexpected(
        error(PlanningErrorCode::invalid_request, "target_world_m must contain finite values"));
  }

  for (std::size_t index = 0; index < request.q_start.size(); ++index) {
    const double position = request.q_start[index];
    if (!std::isfinite(position)) {
      return std::unexpected(
          error(PlanningErrorCode::invalid_request, "q_start must contain finite values"));
    }
    if (position < impl_->model_.lowerPositionLimit[static_cast<Eigen::Index>(index)] ||
        position > impl_->model_.upperPositionLimit[static_cast<Eigen::Index>(index)]) {
      return std::unexpected(
          error(PlanningErrorCode::invalid_request,
                "q_start violates the limit for " + std::string(kJointNames[index])));
    }
  }

  const auto started_at = std::chrono::steady_clock::now();

  try {
    Eigen::VectorXd full_q(static_cast<Eigen::Index>(request.q_start.size()));
    for (std::size_t index = 0; index < request.q_start.size(); ++index) {
      full_q[static_cast<Eigen::Index>(index)] = request.q_start[index];
    }

    auto model_to_reduce = impl_->model_;
    // Pinocchio 4 rejects the MJCF's duplicate BODY/JOINT frame name while
    // reducing. Rename only the private model copy; the canonical joint name
    // and gripperframe contract remain unchanged.
    for (auto& frame : model_to_reduce.frames) {
      if (frame.name == kJointNames.back() && frame.type != pinocchio::JOINT) {
        frame.name = "gripper_body";
      }
    }
    const auto gripper_joint = model_to_reduce.getJointId(kJointNames.back());
    const std::vector<pinocchio::JointIndex> locked_joints{gripper_joint};
    pinocchio::Model arm_model =
        pinocchio::buildReducedModel(model_to_reduce, locked_joints, full_q);
    if (arm_model.nq != static_cast<int>(kArmJointCount) ||
        arm_model.nv != static_cast<int>(kArmJointCount)) {
      return std::unexpected(error(PlanningErrorCode::model_mismatch,
                                   "locking the gripper did not create a 5-DoF arm"));
    }
    if (!arm_model.existFrame("gripperframe")) {
      return std::unexpected(error(PlanningErrorCode::frame_missing,
                                   "gripperframe was lost while locking the gripper"));
    }

    using Scalar = double;
    using Space = aligator::MultibodyPhaseSpace<Scalar>;
    using Dynamics = aligator::dynamics::IntegratorSemiImplEulerTpl<Scalar>;
    using ContinuousDynamics = aligator::dynamics::MultibodyFreeFwdDynamicsTpl<Scalar>;
    using CostStack = aligator::CostStackTpl<Scalar>;
    using StageModel = aligator::StageModelTpl<Scalar>;
    using QuadraticStateCost = aligator::QuadraticStateCostTpl<Scalar>;
    using QuadraticControlCost = aligator::QuadraticControlCostTpl<Scalar>;
    using QuadraticResidualCost = aligator::QuadraticResidualCostTpl<Scalar>;
    using FrameTranslationResidual = aligator::FrameTranslationResidualTpl<Scalar>;
    using FrameVelocityResidual = aligator::FrameVelocityResidualTpl<Scalar>;
    using BoxConstraint = aligator::BoxConstraintTpl<Scalar>;
    using TrajOptProblem = aligator::TrajOptProblemTpl<Scalar>;
    using Solver = aligator::SolverProxDDPTpl<Scalar>;

    const Space state_space(arm_model);
    const int nq = arm_model.nq;
    const int nv = arm_model.nv;
    const int nu = nv;
    const int ndx = state_space.ndx();

    Eigen::VectorXd x0(nq + nv);
    x0.head(nq) = full_q.head(nq);
    x0.tail(nv).setZero();

    const double requested_steps = request.duration_s / kNominalTimeStepS;
    if (!std::isfinite(requested_steps) ||
        requested_steps > static_cast<double>(kMaximumStepCount)) {
      return std::unexpected(error(PlanningErrorCode::planning_failed,
                                   "requested duration exceeds the configured planning horizon"));
    }
    const auto rounded_steps = std::llround(requested_steps);
    const std::size_t step_count = static_cast<std::size_t>(std::max<long long>(1, rounded_steps));
    const double time_step = request.duration_s / static_cast<double>(step_count);

    const Eigen::MatrixXd actuation = Eigen::MatrixXd::Identity(nv, nu);
    const ContinuousDynamics continuous_dynamics(state_space, actuation);
    const Dynamics dynamics(continuous_dynamics, time_step);

    Eigen::MatrixXd running_state_weights = Eigen::MatrixXd::Zero(ndx, ndx);
    running_state_weights.diagonal().head(nv).array() = 1.0 * time_step;
    running_state_weights.diagonal().tail(nv).array() = 1e-2 * time_step;
    const Eigen::MatrixXd running_control_weights =
        1e-3 * time_step * Eigen::MatrixXd::Identity(nu, nu);
    const Eigen::VectorXd zero_control = Eigen::VectorXd::Zero(nu);

    CostStack running_cost(state_space, nu);
    running_cost.addCost("posture", QuadraticStateCost(state_space, nu, x0, running_state_weights));
    running_cost.addCost("effort",
                         QuadraticControlCost(state_space, zero_control, running_control_weights));

    StageModel stage(running_cost, dynamics);
    const Eigen::VectorXd control_lower =
        Eigen::VectorXd::Constant(nu, -kModelActuatorEffortLimitNm);
    const Eigen::VectorXd control_upper =
        Eigen::VectorXd::Constant(nu, kModelActuatorEffortLimitNm);
    stage.addConstraint(aligator::ControlErrorResidualTpl<Scalar>(ndx, zero_control),
                        BoxConstraint(control_lower, control_upper));

    Eigen::VectorXd state_lower(nq + nv);
    Eigen::VectorXd state_upper(nq + nv);
    state_lower.head(nq) = arm_model.lowerPositionLimit;
    state_upper.head(nq) = arm_model.upperPositionLimit;
    state_lower.tail(nv).setConstant(-kExperimentalVelocityLimitRadS);
    state_upper.tail(nv).setConstant(kExperimentalVelocityLimitRadS);
    stage.addConstraint(
        aligator::StateErrorResidualTpl<Scalar>(state_space, nu, state_space.neutral()),
        BoxConstraint(state_lower, state_upper));

    std::vector<xyz::polymorphic<StageModel>> stages(step_count, stage);

    CostStack terminal_cost(state_space, nu);
    const Eigen::MatrixXd terminal_state_weights = 1.0 * Eigen::MatrixXd::Identity(ndx, ndx);
    terminal_cost.addCost("posture",
                          QuadraticStateCost(state_space, nu, x0, terminal_state_weights));

    const auto frame_id = arm_model.getFrameId("gripperframe");
    const FrameTranslationResidual position_residual(ndx, nu, arm_model, request.target_world_m,
                                                     frame_id);
    terminal_cost.addCost("position", QuadraticResidualCost(state_space, position_residual,
                                                            1e5 * Eigen::Matrix3d::Identity()));

    const FrameVelocityResidual velocity_residual(ndx, nu, arm_model, pinocchio::Motion::Zero(),
                                                  frame_id, pinocchio::LOCAL);
    terminal_cost.addCost("velocity",
                          QuadraticResidualCost(state_space, velocity_residual,
                                                1e3 * Eigen::Matrix<double, 6, 6>::Identity()));

    const TrajOptProblem problem(x0, stages, terminal_cost);

    pinocchio::Data gravity_data(arm_model);
    const Eigen::VectorXd gravity_torque = pinocchio::rnea(
        arm_model, gravity_data, x0.head(nq), Eigen::VectorXd::Zero(nv), Eigen::VectorXd::Zero(nv));
    std::vector<Eigen::VectorXd> initial_controls(step_count, gravity_torque);
    const auto initial_states = aligator::rollout(dynamics, x0, initial_controls);

    Solver solver(1e-5, 1e-6, 300, aligator::QUIET);
    solver.rollout_type_ = aligator::RolloutType::NONLINEAR;
    solver.setup(problem);
    const bool converged = solver.run(problem, initial_states, initial_controls);
    const auto& results = solver.results_;
    if (!converged || !results.conv) {
      return std::unexpected(
          error(PlanningErrorCode::planning_failed,
                "Aligator did not converge (iterations=" + std::to_string(results.num_iters) +
                    ", primal_infeasibility=" + std::to_string(results.prim_infeas) +
                    ", dual_infeasibility=" + std::to_string(results.dual_infeas) + ")"));
    }
    if (results.xs.size() != step_count + 1 || results.us.size() != step_count) {
      return std::unexpected(error(PlanningErrorCode::validation_failed,
                                   "Aligator returned inconsistent trajectory dimensions"));
    }

    double maximum_joint_violation = 0.0;
    double maximum_velocity_violation = 0.0;
    const Eigen::VectorXd velocity_lower =
        Eigen::VectorXd::Constant(nv, -kExperimentalVelocityLimitRadS);
    const Eigen::VectorXd velocity_upper =
        Eigen::VectorXd::Constant(nv, kExperimentalVelocityLimitRadS);
    for (const auto& state : results.xs) {
      if (!state.allFinite() || state.size() != nq + nv) {
        return std::unexpected(
            error(PlanningErrorCode::validation_failed, "Aligator returned an invalid state"));
      }
      maximum_joint_violation = std::max(
          maximum_joint_violation, max_limit_violation(state.head(nq), arm_model.lowerPositionLimit,
                                                       arm_model.upperPositionLimit));
      maximum_velocity_violation =
          std::max(maximum_velocity_violation,
                   max_limit_violation(state.tail(nv), velocity_lower, velocity_upper));
    }

    double maximum_effort_violation = 0.0;
    for (const auto& control : results.us) {
      if (!control.allFinite() || control.size() != nu) {
        return std::unexpected(
            error(PlanningErrorCode::validation_failed, "Aligator returned an invalid control"));
      }
      maximum_effort_violation =
          std::max(maximum_effort_violation,
                   std::max(0.0, control.cwiseAbs().maxCoeff() - kModelActuatorEffortLimitNm));
    }

    double maximum_dynamics_defect = 0.0;
    auto dynamics_data = dynamics.createData();
    Eigen::VectorXd dynamics_difference(ndx);
    for (std::size_t step = 0; step < step_count; ++step) {
      dynamics.forward(results.xs[step], results.us[step], *dynamics_data);
      const auto* explicit_data =
          dynamic_cast<const aligator::ExplicitDynamicsDataTpl<Scalar>*>(dynamics_data.get());
      if (explicit_data == nullptr) {
        return std::unexpected(error(PlanningErrorCode::validation_failed,
                                     "Aligator dynamics data has an unexpected type"));
      }
      state_space.difference(explicit_data->xnext_, results.xs[step + 1], dynamics_difference);
      maximum_dynamics_defect =
          std::max(maximum_dynamics_defect, dynamics_difference.lpNorm<Eigen::Infinity>());
    }

    pinocchio::Data final_data(arm_model);
    const auto& final_state = results.xs.back();
    pinocchio::forwardKinematics(arm_model, final_data, final_state.head(nq), final_state.tail(nv));
    pinocchio::updateFramePlacements(arm_model, final_data);
    const double final_position_error =
        (final_data.oMf[frame_id].translation() - request.target_world_m).norm();
    const double final_frame_speed =
        pinocchio::getFrameVelocity(arm_model, final_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED)
            .linear()
            .norm();

    if (!std::isfinite(final_position_error) || !std::isfinite(final_frame_speed) ||
        maximum_joint_violation > kJointLimitToleranceRad || maximum_velocity_violation > 1e-6 ||
        maximum_effort_violation > kEffortLimitToleranceNm ||
        maximum_dynamics_defect > kDynamicsDefectTolerance ||
        final_position_error > kFinalPositionToleranceM ||
        final_frame_speed > kFinalFrameSpeedToleranceMps) {
      return std::unexpected(
          error(PlanningErrorCode::validation_failed,
                "Aligator result violated a postcondition (position_error=" +
                    std::to_string(final_position_error) +
                    ", frame_speed=" + std::to_string(final_frame_speed) +
                    ", joint_violation=" + std::to_string(maximum_joint_violation) +
                    ", velocity_violation=" + std::to_string(maximum_velocity_violation) +
                    ", effort_violation=" + std::to_string(maximum_effort_violation) +
                    ", dynamics_defect=" + std::to_string(maximum_dynamics_defect) + ")"));
    }

    JointTrajectory trajectory;
    trajectory.knots.reserve(results.xs.size());
    for (std::size_t knot_index = 0; knot_index < results.xs.size(); ++knot_index) {
      TrajectoryKnot knot{
          .time_s = knot_index == step_count ? request.duration_s
                                             : static_cast<double>(knot_index) * time_step,
          .q = {},
          .velocity = {},
      };
      for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
        knot.q[joint] = results.xs[knot_index][static_cast<Eigen::Index>(joint)];
        knot.velocity[joint] = results.xs[knot_index][nq + static_cast<Eigen::Index>(joint)];
      }
      knot.q.back() = request.q_start.back();
      knot.velocity.back() = 0.0;
      trajectory.knots.push_back(knot);
    }

    const double initial_configuration_error = [&] {
      double result = 0.0;
      for (std::size_t joint = 0; joint < request.q_start.size(); ++joint) {
        result =
            std::max(result, std::abs(trajectory.knots.front().q[joint] - request.q_start[joint]));
      }
      return result;
    }();
    if (initial_configuration_error > 1e-9) {
      return std::unexpected(error(PlanningErrorCode::validation_failed,
                                   "Aligator changed the requested initial configuration"));
    }

    trajectory.report = PlanningReport{
        .strategy = ReachPlannerKind::aligator,
        .final_position_error_m = final_position_error,
        .final_frame_speed_mps = final_frame_speed,
        .max_joint_limit_violation_rad = maximum_joint_violation,
        .computation_time_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count(),
    };
    return trajectory;
  } catch (const std::exception& planning_error) {
    return std::unexpected(
        error(PlanningErrorCode::planning_failed,
              "Aligator planning failed: " + std::string(planning_error.what())));
  }
}

std::expected<std::unique_ptr<ReachPlanner>, PlanningError> ReachPlannerFactory::create(
    const ReachPlannerKind kind, const std::filesystem::path& robot_mjcf) {
  if (kind != ReachPlannerKind::aligator) {
    return std::unexpected(
        error(PlanningErrorCode::strategy_unavailable, "requested reach strategy is unavailable"));
  }

  std::error_code filesystem_error;
  const auto absolute_path = std::filesystem::absolute(robot_mjcf, filesystem_error);
  if (filesystem_error || !std::filesystem::is_regular_file(absolute_path, filesystem_error)) {
    return std::unexpected(error(PlanningErrorCode::model_mismatch,
                                 "robot MJCF does not exist: " + robot_mjcf.string()));
  }

  pinocchio::Model model;
  try {
    pinocchio::mjcf::buildModel(absolute_path.string(), model, false);
  } catch (const std::exception& parse_error) {
    return std::unexpected(
        error(PlanningErrorCode::model_mismatch,
              "Pinocchio could not load the robot MJCF: " + std::string(parse_error.what())));
  }

  if (model.nq != static_cast<int>(kJointNames.size()) ||
      model.nv != static_cast<int>(kJointNames.size())) {
    return std::unexpected(
        error(PlanningErrorCode::model_mismatch, "robot model must have six scalar joints"));
  }
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    if (!model.existJointName(kJointNames[index])) {
      return std::unexpected(
          error(PlanningErrorCode::model_mismatch,
                "robot model is missing joint " + std::string(kJointNames[index])));
    }
    const auto joint = model.getJointId(kJointNames[index]);
    if (model.idx_qs[joint] != static_cast<int>(index) ||
        model.idx_vs[joint] != static_cast<int>(index)) {
      return std::unexpected(error(PlanningErrorCode::model_mismatch,
                                   "robot joint order does not match the trajectory contract"));
    }
  }
  if (!model.existFrame("gripperframe")) {
    return std::unexpected(
        error(PlanningErrorCode::frame_missing, "robot model has no gripperframe"));
  }

  auto concrete = std::unique_ptr<AligatorReachPlanner>(
      new AligatorReachPlanner(std::make_unique<AligatorReachPlanner::Impl>(std::move(model))));
  return std::unique_ptr<ReachPlanner>(std::move(concrete));
}

}  // namespace amp
