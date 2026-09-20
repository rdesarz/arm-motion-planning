#include "so101_traj_planner/core/aligator_reach_planner.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <aligator/core/function-abstract.hpp>
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
#include <chrono>
#include <cmath>
#include <optional>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <system_error>
#include <utility>
#include <vector>

#include "aligator_validation.hpp"
#include "so101_traj_planner/core/so101_model.hpp"

namespace so101_traj_planner {

namespace {

constexpr std::size_t kMaximumStepCount = 1000;
constexpr double kNominalTimeStepS = 0.02;

class JointAccelerationResidual final : public aligator::StageFunctionTpl<double> {
 public:
  using Scalar = double;
  using Base = aligator::StageFunctionTpl<Scalar>;
  using BaseData = aligator::StageFunctionDataTpl<Scalar>;
  using Dynamics = aligator::dynamics::MultibodyFreeFwdDynamicsTpl<Scalar>;
  using DynamicsData = aligator::dynamics::ContinuousDynamicsDataTpl<Scalar>;

  JointAccelerationResidual(const Dynamics& dynamics, const int velocity_dimension)
      : Base(dynamics.ndx(), dynamics.nu(), velocity_dimension),
        dynamics_(dynamics),
        velocity_dimension_(velocity_dimension) {}

  void evaluate(const Eigen::Ref<const Eigen::VectorXd>& state,
                const Eigen::Ref<const Eigen::VectorXd>& control,
                BaseData& base_data) const override {
    auto& data = static_cast<Data&>(base_data);
    dynamics_.forward(state, control, *data.dynamics_data);
    data.value_ = data.dynamics_data->xdot_.tail(velocity_dimension_);
  }

  void computeJacobians(const Eigen::Ref<const Eigen::VectorXd>& state,
                        const Eigen::Ref<const Eigen::VectorXd>& control,
                        BaseData& base_data) const override {
    auto& data = static_cast<Data&>(base_data);
    dynamics_.forward(state, control, *data.dynamics_data);
    dynamics_.dForward(state, control, *data.dynamics_data);
    data.Jx_ = data.dynamics_data->Jx_.bottomRows(velocity_dimension_);
    data.Ju_ = data.dynamics_data->Ju_.bottomRows(velocity_dimension_);
  }

  struct Data final : BaseData {
    explicit Data(const JointAccelerationResidual& residual)
        : BaseData(residual), dynamics_data(residual.dynamics_.createData()) {}

    std::shared_ptr<DynamicsData> dynamics_data;
  };

  std::shared_ptr<BaseData> createData() const override { return std::make_shared<Data>(*this); }

 private:
  Dynamics dynamics_;
  int velocity_dimension_;
};

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

std::optional<PlanningError> validate_request(const pinocchio::Model& model,
                                              const ReachRequest& request) {
  if (!std::isfinite(request.duration_s) || request.duration_s <= 0.0) {
    return error(PlanningErrorCode::invalid_request,
                 "duration_s must be finite and strictly positive");
  }
  if (!request.target_world_m.allFinite()) {
    return error(PlanningErrorCode::invalid_request, "target_world_m must contain finite values");
  }

  for (std::size_t index = 0; index < request.q_start.size(); ++index) {
    const double position = request.q_start[index];
    if (!std::isfinite(position)) {
      return error(PlanningErrorCode::invalid_request, "q_start must contain finite values");
    }
    if (position < model.lowerPositionLimit[static_cast<Eigen::Index>(index)] ||
        position > model.upperPositionLimit[static_cast<Eigen::Index>(index)]) {
      return error(PlanningErrorCode::invalid_request,
                   "q_start violates the limit for " + std::string(kSo101JointNames[index]));
    }
  }
  return std::nullopt;
}

std::expected<pinocchio::Model, PlanningError> build_arm_model(const pinocchio::Model& full_model,
                                                               const JointVector& q_start) {
  Eigen::Map<const Eigen::VectorXd> full_q(q_start.data(),
                                           static_cast<Eigen::Index>(q_start.size()));

  auto model_to_reduce = full_model;
  // Pinocchio 4 rejects the MJCF's duplicate BODY/JOINT frame name while
  // reducing. Rename only the private model copy; the canonical joint name
  // and gripperframe contract remain unchanged.
  for (auto& frame : model_to_reduce.frames) {
    if (frame.name == kSo101JointNames[kSo101GripperIndex] && frame.type != pinocchio::JOINT) {
      frame.name = "gripper_body";
    }
  }
  const auto gripper_joint =
      model_to_reduce.getJointId(kSo101JointNames[kSo101GripperIndex].data());
  const std::vector<pinocchio::JointIndex> locked_joints{gripper_joint};
  pinocchio::Model arm_model = pinocchio::buildReducedModel(model_to_reduce, locked_joints, full_q);
  if (arm_model.nq != static_cast<int>(kSo101ArmJointCount) ||
      arm_model.nv != static_cast<int>(kSo101ArmJointCount)) {
    return std::unexpected(
        error(PlanningErrorCode::model_mismatch, "locking the gripper did not create a 5-DoF arm"));
  }
  if (!arm_model.existFrame(kSo101EndEffectorFrame.data())) {
    return std::unexpected(
        error(PlanningErrorCode::frame_missing, "gripperframe was lost while locking the gripper"));
  }
  return arm_model;
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
  if (const auto request_error = validate_request(impl_->model_, request)) {
    return std::unexpected(*request_error);
  }

  const auto started_at = std::chrono::steady_clock::now();

  try {
    auto arm_model_result = build_arm_model(impl_->model_, request.q_start);
    if (!arm_model_result) {
      return std::unexpected(arm_model_result.error());
    }
    pinocchio::Model arm_model = std::move(*arm_model_result);

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
    for (std::size_t joint = 0; joint < kSo101ArmJointCount; ++joint) {
      x0[static_cast<Eigen::Index>(joint)] = request.q_start[joint];
    }
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

    const Eigen::MatrixXd running_acceleration_weights =
        time_step * Eigen::MatrixXd::Identity(nv, nv);
    const Eigen::MatrixXd running_control_weights =
        1e-3 * time_step * Eigen::MatrixXd::Identity(nu, nu);
    const Eigen::VectorXd zero_control = Eigen::VectorXd::Zero(nu);

    CostStack running_cost(state_space, nu);
    running_cost.addCost(
        "acceleration",
        QuadraticResidualCost(state_space, JointAccelerationResidual(continuous_dynamics, nv),
                              running_acceleration_weights));
    running_cost.addCost("effort",
                         QuadraticControlCost(state_space, zero_control, running_control_weights));

    StageModel stage(running_cost, dynamics);
    const Eigen::VectorXd control_lower =
        Eigen::VectorXd::Constant(nu, -kSo101ActuatorEffortLimitNm);
    const Eigen::VectorXd control_upper =
        Eigen::VectorXd::Constant(nu, kSo101ActuatorEffortLimitNm);
    stage.addConstraint(aligator::ControlErrorResidualTpl<Scalar>(ndx, zero_control),
                        BoxConstraint(control_lower, control_upper));

    Eigen::VectorXd state_lower(nq + nv);
    Eigen::VectorXd state_upper(nq + nv);
    state_lower.head(nq) = arm_model.lowerPositionLimit;
    state_upper.head(nq) = arm_model.upperPositionLimit;
    state_lower.tail(nv).setConstant(-kSo101ExperimentalVelocityLimitRadS);
    state_upper.tail(nv).setConstant(kSo101ExperimentalVelocityLimitRadS);
    stage.addConstraint(
        aligator::StateErrorResidualTpl<Scalar>(state_space, nu, state_space.neutral()),
        BoxConstraint(state_lower, state_upper));

    std::vector<xyz::polymorphic<StageModel>> stages(step_count, stage);

    CostStack terminal_cost(state_space, nu);
    Eigen::MatrixXd terminal_state_weights = Eigen::MatrixXd::Zero(ndx, ndx);
    terminal_state_weights.diagonal().tail(nv).setConstant(1e3);
    terminal_cost.addCost("joint_velocity",
                          QuadraticStateCost(state_space, nu, x0, terminal_state_weights));

    const auto frame_id = arm_model.getFrameId(kSo101EndEffectorFrame.data());
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
        Eigen::VectorXd::Constant(nv, -kSo101ExperimentalVelocityLimitRadS);
    const Eigen::VectorXd velocity_upper =
        Eigen::VectorXd::Constant(nv, kSo101ExperimentalVelocityLimitRadS);
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
                   std::max(0.0, control.cwiseAbs().maxCoeff() - kSo101ActuatorEffortLimitNm));
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
      if (!explicit_data->xnext_.allFinite()) {
        return std::unexpected(error(PlanningErrorCode::validation_failed,
                                     "Aligator returned a non-finite dynamics rollout"));
      }
      state_space.difference(explicit_data->xnext_, results.xs[step + 1], dynamics_difference);
      if (!dynamics_difference.allFinite()) {
        return std::unexpected(error(PlanningErrorCode::validation_failed,
                                     "Aligator returned a non-finite dynamics rollout"));
      }
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

    const detail::AligatorValidationMetrics metrics{
        .final_position_error_m = final_position_error,
        .final_frame_speed_mps = final_frame_speed,
        .max_joint_limit_violation_rad = maximum_joint_violation,
        .max_velocity_limit_violation_rad_s = maximum_velocity_violation,
        .max_effort_limit_violation_nm = maximum_effort_violation,
        .max_dynamics_defect = maximum_dynamics_defect,
    };
    if (const auto validation_error = detail::validate_aligator_metrics(metrics)) {
      return std::unexpected(error(PlanningErrorCode::validation_failed, *validation_error));
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
      for (std::size_t joint = 0; joint < kSo101ArmJointCount; ++joint) {
        knot.q[joint] = results.xs[knot_index][static_cast<Eigen::Index>(joint)];
        knot.velocity[joint] = results.xs[knot_index][nq + static_cast<Eigen::Index>(joint)];
      }
      knot.q[kSo101GripperIndex] = request.q_start[kSo101GripperIndex];
      knot.velocity[kSo101GripperIndex] = 0.0;
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

std::expected<AligatorReachPlanner, PlanningError> AligatorReachPlanner::load(
    const std::filesystem::path& robot_mjcf) {
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

  if (model.nq != static_cast<int>(kSo101JointCount) ||
      model.nv != static_cast<int>(kSo101JointCount)) {
    return std::unexpected(
        error(PlanningErrorCode::model_mismatch, "robot model must have six scalar joints"));
  }
  for (std::size_t index = 0; index < kSo101JointNames.size(); ++index) {
    if (!model.existJointName(kSo101JointNames[index].data())) {
      return std::unexpected(
          error(PlanningErrorCode::model_mismatch,
                "robot model is missing joint " + std::string(kSo101JointNames[index])));
    }
    const auto joint = model.getJointId(kSo101JointNames[index].data());
    if (model.idx_qs[joint] != static_cast<int>(index) ||
        model.idx_vs[joint] != static_cast<int>(index)) {
      return std::unexpected(error(PlanningErrorCode::model_mismatch,
                                   "robot joint order does not match the trajectory contract"));
    }
  }
  if (!model.existFrame(kSo101EndEffectorFrame.data())) {
    return std::unexpected(
        error(PlanningErrorCode::frame_missing, "robot model has no gripperframe"));
  }

  return AligatorReachPlanner(std::make_unique<Impl>(std::move(model)));
}

}  // namespace so101_traj_planner
