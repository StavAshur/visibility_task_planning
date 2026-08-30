#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>

#include "ValidityChecker.h"
#include "../common/Types.h"
namespace visual_planner {

class VisualIK {


public:

    /**
     * @brief Constructor
     */
    VisualIK(moveit::core::RobotModelConstPtr model, 
             std::shared_ptr<ValidityChecker> checker,
             const std::string& group_name = "manipulator",
             const std::string& ee_link = "tool0")
        : robot_model_(model), validity_checker_(checker), group_name_(group_name), ee_link_name_(ee_link)
    {
        robot_state_.reset(new moveit::core::RobotState(robot_model_));
        tool_params_ = {M_PI / 12.0, 1.0};
    }


    /**
     * @brief Standard IK solver with collision checking.
     * Solves for the exact pose provided without any visual-based local search.
     * @param pose The exact 6D pose (Position + Orientation) to solve for.
     * @param seed_joints Initial guess for the solver.
     * @param solution_out [Output] The valid joint configuration.
     */
    bool solveIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_joints, std::vector<double>& solution_out) {
        robot_state_->setJointGroupPositions(group_name_, seed_joints);
        // The pose is given in the model frame, and the solver reaches it through the
        // link transforms of the seed -- which matters when the IK group is a subgroup
        // mounted on joints it may not move.
        robot_state_->update();
        return checkIK(pose, solution_out);
    }

    void setGroupName(const std::string& group) {
        group_name_ = group;
    }

    void setEELinkName(const std::string& ee_link) {
        ee_link_name_ = ee_link;
    }

    /**
     * @brief Restricts the solver to a SUBGROUP of the configuration's joints.
     *
     * Two groups are in play. The state group (setGroupName) says what a configuration
     * vector means: it is the group every seed is expressed in, the group FK is read
     * from, and the group solutions are returned in. The IK group says which of those
     * joints the solver is allowed to move. They are the same group unless this is
     * called.
     *
     * The reason to separate them is the base-locality constraint: with the IK group
     * set to the arm alone, a solution cannot move the mobile base at all, so it
     * inherits the base position of its seed and can never leave the disk the seed was
     * drawn from. The alternative -- solving over the whole robot and rejecting
     * solutions that left the disk -- gives the solver no incentive to stay inside it,
     * and its random restarts place the base anywhere in a ten-metre range.
     *
     * The joints outside the IK group keep whatever values the seed gave them, so
     * inputs and outputs are unaffected: still full state-group vectors.
     *
     * @param group A group whose joints are a subset of the state group's. Passing the
     *              state group's own name is the same as clearIKGroupName().
     */
    void setIKGroupName(const std::string& group) {
        ik_group_name_ = group;
    }

    /// Returns the solver to using every joint of the state group.
    void clearIKGroupName() {
        ik_group_name_.clear();
    }

    /// The group the solver may move: the override if one is set, else the state group.
    const std::string& getIKGroupName() const {
        return ik_group_name_.empty() ? group_name_ : ik_group_name_;
    }

    /// The override alone, empty when none is set. Distinct from getIKGroupName() so a
    /// caller that restores a previous setting can tell "unset" from "set to the state
    /// group", which stop being the same thing once the state group changes.
    const std::string& getIKGroupOverride() const {
        return ik_group_name_;
    }

    // =========================================================================================
    // 1. Core Logic: Solve IK for a Specific Orientation
    // =========================================================================================

    /**
     * @brief Finds a solution that places the camera at a specific standoff distance from the centroid,
     * aligned with the provided orientation.
     * * @param current_joints Used as a seed for IK.
     * @param target_points The points to observe.
     * @param orientation The desired orientation (Rotation Matrix) of the camera.
     * @param solution_out [Output] Resulting joints.
     */
    /**
     * @brief Finds a solution that looks at the MES center with the given orientation.
     * Standoff distance is derived from the current robot position relative to the target.
     */
    bool solveVisualIK(const std::vector<double>& current_joints,
                       const Ball& target_mes,
                       const Eigen::Matrix3d& orientation,
                       std::vector<double>& solution_out) 
    {
        double h = tool_params_.beam_length;
        double theta = tool_params_.beam_angle;
        // Update Robot State to get current position for Standoff Calculation
        robot_state_->setJointGroupPositions(group_name_, current_joints);
        robot_state_->update();
        Eigen::Vector3d current_pos = robot_state_->getGlobalLinkTransform(ee_link_name_).translation();


        Eigen::Vector3d c = target_mes.center;
        double r = target_mes.radius;  

        // We assume the camera looks down its local +Z axis.
        Eigen::Vector3d z_axis = orientation.col(2);
        Eigen::Vector3d v = z_axis.normalized();

        // Compute valid segment [a, b] along v
        Eigen::Vector3d a = c - (h - r) * v;
        Eigen::Vector3d b = c - (r / std::tan(theta)) * v;
        double seg_length = (b - a).norm();

        // ROS_INFO("Before intersection: h=%f, theta=%f \n Segment is a=(%f,%f,%f) b=(%f,%f,%f)",
        //         h, theta,
        //         a.x(), a.y(), a.z(),
        //         b.x(), b.y(), b.z());


        // Define AABB parameters
        Eigen::Vector3d box_min(-5.0, -5.0, 0.0);
        Eigen::Vector3d box_max(5.0, 5.0, 1.4);

        // Vector logic: P(t) = a + t * u, for t in [0, 1]
        Eigen::Vector3d u = b - a;

        // Initialize t_start and t_end to represent the full original segment
        double t_start = 0.0;
        double t_end = 1.0;

        // Iterate over the X, Y, and Z axes
        for (int i = 0; i < 3; ++i) {
            // Check if the segment is parallel to the planes on this axis
            if (std::abs(u[i]) < 1e-8) {
                // If it's parallel and outside the box bounds, it doesn't intersect
                if (a[i] < box_min[i] || a[i] > box_max[i]) {
                    return false; 
                }
            } else {
                // Calculate intersection t-values with the two parallel planes
                double t1 = (box_min[i] - a[i]) / u[i];
                double t2 = (box_max[i] - a[i]) / u[i];

                // Ensure t1 is the entry point and t2 is the exit point
                if (t1 > t2) std::swap(t1, t2);

                // Narrow down the valid segment interval
                t_start = std::max(t_start, t1);
                t_end = std::min(t_end, t2);

                // If the valid interval becomes invalid/empty, there is no intersection
                if (t_start > t_end) return false;
            }
        }

        // If the valid interval is effectively empty or length is 0
        if (t_start >= t_end) return false;

        // Update points a and b to the intersection bounds
        // Store original 'a' to ensure 'b' is calculated correctly
        Eigen::Vector3d a_orig = a;
        a = a_orig + t_start * u;
        b = a_orig + t_end * u;

        ROS_INFO("After intersection: h=%f, theta=%f \n Segment is a=(%f,%f,%f) b=(%f,%f,%f)",
                 h, theta,
                 a.x(), a.y(), a.z(),
                 b.x(), b.y(), b.z());





        // 3. Final check: verify the segment hasn't been reduced to a single point
        if ((a - b).norm() <= 1e-6) return false;


        Eigen::Vector3d ab = b - a;
        double t_param = ab.dot(current_pos - a) / ab.squaredNorm();

        // clamping
        t_param = (t_param < 0.0) ? 0.0 : t_param;
        t_param = (t_param > 1.0) ? 1.0 : t_param;

        Eigen::Vector3d p_prime = a + t_param * ab;


        /// @todo THIS IS WRONG. DIR SHOULD BE EQUAL TO V. STEP SIZE SHOULD BE DIFFERENT FOR EACH DIRECTION.
        Eigen::Vector3d dir = v;
        double step_size_positive = seg_length * (1 - t_param)  / (2.0 * num_steps);
        double step_size_negative = seg_length * t_param / (2.0 * num_steps);

        // Construct Full Pose
        Eigen::Isometry3d test_pose = Eigen::Isometry3d::Identity();
        test_pose.translation() = p_prime;
        test_pose.linear() = orientation;

        // Solve IK with Local Search
        
        // Try ideal position
        // Note: robot_state_ is already set to current_joints from Step 1
        if (checkIK(test_pose, solution_out)) return true;
        
        /// @todo SHOULD BE FIXED! NOT THE RIGHT WAY!
        // Eigen::Vector3d flashlight_adjust = Eigen::Vector3d(0, 0, 0.2);
        // p_prime = p_prime - flashlight_adjust;
        

        for (size_t i = 1; i <= num_steps / 2.0; i++) {
            // Step in +dir (Closer to target)
            Eigen::Vector3d p_plus = p_prime + (i * step_size_positive * dir);
            test_pose.translation() = p_plus;

            if (validity_checker_->isPointInObstacle(p_plus[0], p_plus[1], p_plus[2]))
                continue;

            robot_state_->setJointGroupPositions(group_name_, current_joints); 
            if (checkIK(test_pose, solution_out)) return true;

            // Step in -dir (Further from target)
            Eigen::Vector3d p_minus = p_prime - (i * step_size_negative * dir);
            test_pose.translation() = p_minus;
            robot_state_->setJointGroupPositions(group_name_, current_joints);
            if (checkIK(test_pose, solution_out)) return true;
        }

        return false;
    }

    /**
     * @brief Wrapper: overload that takes vector of points, computes centroid, and calls MES version.
     */
    bool solveVisualIK(const std::vector<double>& current_joints,
                       const std::vector<geometry_msgs::Point>& target_points,
                       const Eigen::Matrix3d& orientation,
                       std::vector<double>& solution_out)
    {
        if (target_points.empty()) return false;

        // Calculate Centroid
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& p : target_points) {
            centroid += Eigen::Vector3d(p.x, p.y, p.z);
        }
        centroid /= target_points.size();

        Ball mes;
        mes.center = centroid;


        double r = 0.0;
        Eigen::Vector3d curr;
        for (const auto& p : target_points) {
            curr = Eigen::Vector3d(p.x, p.y, p.z);
            double d = (curr - centroid).norm();
            if (d > r) r = d;
        }

        mes.radius = r; 

        return solveVisualIK(current_joints, mes, orientation, solution_out);
    }

    // =========================================================================================
    // 2. Wrapper: Maintain Current Orientation
    // =========================================================================================

    /**
     * @brief Gets the current end-effector orientation via FK, then tries to find a valid 
     * observation pose preserving that orientation.
     */
    bool maintainOrientationVisualIK(const std::vector<double>& current_joints,
                                           const std::vector<geometry_msgs::Point>& target_points,
                                           std::vector<double>& solution_out)
    {
        // 1. Update Robot State
        robot_state_->setJointGroupPositions(group_name_, current_joints);
        robot_state_->update();

        // 2. Get Current Orientation
        const Eigen::Isometry3d& current_pose = robot_state_->getGlobalLinkTransform(ee_link_name_);
        Eigen::Matrix3d current_orientation = current_pose.linear();

        // 3. Delegate to Core Function
        return solveVisualIK(current_joints, target_points, current_orientation, solution_out);
    }

    // =========================================================================================
    // 3. Wrapper: Look-At Strategy (Original, Refactored)
    // =========================================================================================

    /**
     * @brief Calculates a new orientation that points the camera Z-axis at the targets,
     * then calls solveVisualIK.
     */
    bool greedyOrientationVisualIK(const std::vector<double>& current_joints,
                            const std::vector<geometry_msgs::Point>& target_points,
                            std::vector<double>& solution_out) 
    {
        if (target_points.empty()) return false;

        // 1. Calculate Centroid
        Eigen::Vector3d centroid(0, 0, 0);
        for (const auto& p : target_points) {
            centroid += Eigen::Vector3d(p.x, p.y, p.z);
        }
        centroid /= target_points.size();

        // 2. Determine Look-At Orientation
        // Get current position to determine "From Where" we are looking
        robot_state_->setJointGroupPositions(group_name_, current_joints);
        robot_state_->update();
        Eigen::Vector3d current_pos = robot_state_->getGlobalLinkTransform(ee_link_name_).translation();

        // Direction vector: From Camera TO Target
        Eigen::Vector3d direction = (centroid - current_pos).normalized();

        // Construct Rotation Matrix
        // Z points AT target
        Eigen::Vector3d z_axis = direction; 
        
        // Handle Arbitrary Up Vector for X/Y
        Eigen::Vector3d world_up(0, 0, 1);
        Eigen::Vector3d x_axis = world_up.cross(z_axis).normalized();
        
        // Singularity check (looking straight up/down)
        if (x_axis.norm() < 1e-3) { 
            x_axis = Eigen::Vector3d(1, 0, 0); 
        }
        
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3d look_at_rotation;
        look_at_rotation.col(0) = x_axis;
        look_at_rotation.col(1) = y_axis;
        look_at_rotation.col(2) = z_axis;

        // 3. Delegate to Core Function using the calculated rotation
        return solveVisualIK(current_joints, target_points, look_at_rotation, solution_out);
    }

    void setVisibilityToolParams(VisibilityToolParams params) {
        tool_params_=params;
    }

    void setVisibilityToolParams(double beam_angle_rad, double beam_length_m) {
        tool_params_.beam_angle = beam_angle_rad;
        tool_params_.beam_length = beam_length_m;
    }

private:
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<ValidityChecker> validity_checker_;
    VisibilityToolParams tool_params_;
    std::string group_name_;
    std::string ee_link_name_; // End Effector Link (Camera frame)
    /// Empty means the solver may move every joint of the state group. See
    /// setIKGroupName().
    std::string ik_group_name_;

    size_t num_steps = 20;


    /**
     * @brief Helper function to check IK with collision detection
     *
     * Solves over the IK group but seeds from, validates against, and returns the
     * state group, so restricting the solver never changes the shape of a
     * configuration vector.
     */
    bool checkIK(const Eigen::Isometry3d& target_pose, std::vector<double>& solution_out) {
        const moveit::core::JointModelGroup* state_jmg = robot_model_->getJointModelGroup(group_name_);
        const moveit::core::JointModelGroup* ik_jmg    = robot_model_->getJointModelGroup(getIKGroupName());

        if (!state_jmg) throw std::runtime_error("VisualIK: no joint model group named '" + group_name_ + "'.");
        if (!ik_jmg)    throw std::runtime_error("VisualIK: no joint model group named '" + getIKGroupName() + "' to solve IK over.");

        double timeout = 0.1;

        // The candidate carries values for the IK group only, so it is applied to the
        // state first and the full state-group configuration read back out: the joints
        // the solver may not move keep their seed values, and the collision check sees
        // the whole robot either way.
        moveit::core::GroupStateValidityCallbackFn callback =
            [this, state_jmg](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* values) {
                state->setJointGroupPositions(group, values);
                state->update();
                std::vector<double> q;
                state->copyJointGroupPositions(state_jmg, q);
                return this->validity_checker_->isValid(q);
            };

        // setFromIK uses the current state of robot_state_ as the seed
        bool found_ik = robot_state_->setFromIK(ik_jmg, target_pose, ee_link_name_, timeout, callback);

        if (found_ik) {
            robot_state_->copyJointGroupPositions(state_jmg, solution_out);
            return true;
        }
        return false;
    }
};

}// namespace visual_planner
