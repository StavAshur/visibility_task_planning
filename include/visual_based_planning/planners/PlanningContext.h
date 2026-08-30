#pragma once

// Standard Library
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <string>
#include <stdexcept>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

// Internal Components
#include "../data_structures/VisibilityOracle.h"
#include "../data_structures/VisibilityIntegrity.h"
#include "../data_structures/VisibilityRoadmap.h"
#include "../components/Sampler.h"
#include "../components/ValidityChecker.h"
#include "../components/VisualIK.h"
#include "../components/PathSmoother.h"

#include "../common/Types.h"

namespace visual_planner {

/**
 * @brief The world a planner plans in: robot, scene, and visibility structures.
 *
 * Everything here is expensive to construct and independent of which algorithm runs
 * or which query is being answered -- the planning scene and robot state, the
 * sampler, validity checker, VisualIK and path smoother, the visibility oracle, and
 * the two visibility structures (the VI-tree and the VIR roadmap).
 *
 * It is held by shared_ptr so that many planners can share one context. That is what
 * makes it cheap to construct a planner: switching algorithms, or running a local
 * planner once per base location in the VisTSP pipeline, reuses the same VI-tree
 * instead of rebuilding it. Nothing that varies per planning query belongs here --
 * the graph, the start configuration, the target and the resulting path are all
 * per-run state owned by the planner.
 */
class PlanningContext {
public:
    // --- Configuration ---
    double resolution_;
    double visibility_threshold_;
    BoundingBox workspace_bounds_;
    bool is_initialized_ = false;
    VisibilityIntegrityParams visibility_integrity_params_;
    VisibilityToolParams visibility_tool_params_;

    /// Whether initialize_visibility() builds a visibility structure at all. Building
    /// one is the expensive part of setting up a context, so a planner that will never
    /// query it can turn this off. Distinct from the planner's own flag for whether to
    /// *use* visibility-guided sampling, though the node drives both from one ROS
    /// parameter.
    bool use_visibility_structure_ = true;

    /// Selects which visibility structure backs sampleVisibilityRegion(): the VIR
    /// roadmap when true, the VI-tree otherwise. The VI-tree is the default; the
    /// roadmap is the VIR baseline. The context owns both, so the choice lives with
    /// them rather than with any one planner.
    bool use_visibility_roadmap_ = false;

    // --- Components ---
    std::shared_ptr<Sampler> sampler_;
    std::shared_ptr<ValidityChecker> validity_checker_;
    std::shared_ptr<VisualIK> vis_ik_;
    std::shared_ptr<PathSmoother> path_smoother_;

    // --- Visibility Components ---
    std::shared_ptr<VisibilityOracle> vis_oracle_;
    std::shared_ptr<VisibilityIntegrity> vis_integrity_;
    std::shared_ptr<VisibilityRoadmap> vis_roadmap_;

    planning_scene::PlanningScenePtr planning_scene_;

    // Robot State Tracking
    moveit::core::RobotStatePtr robot_state_;
    std::string group_name_;
    std::string ee_link_name_;

    PlanningContext(const planning_scene::PlanningScenePtr& scene,
                    double resolution = 0.05)
        : planning_scene_(scene),
          resolution_(resolution),
          visibility_threshold_(0.8),
        //   group_name_("manipulator"),
          group_name_("whole_robot"),
          ee_link_name_("tool0")
    {
        // 1. Initialize Validity Checker first
        validity_checker_ = std::make_shared<ValidityChecker>(scene, resolution);
        
        // 2. Initialize VisualIK
        vis_ik_ = std::make_shared<VisualIK>(
            scene->getRobotModel(), 
            validity_checker_, 
            group_name_, 
            ee_link_name_ 
        );

        // 3. Initialize Visibility Oracle
        vis_oracle_ = std::make_shared<VisibilityOracle>();
        

        // 4. Initialize Sampler
        sampler_ = std::make_shared<Sampler>(scene->getRobotModel());
        sampler_->setVisualComponents(vis_ik_, vis_oracle_);
        sampler_->setVisibilityThreshold(visibility_threshold_);
        sampler_->setValidityChecker(validity_checker_);

        // 5. Initialize Smoother
        path_smoother_ = std::make_shared<PathSmoother>(validity_checker_);
        
        // 6. Internal Robot State & FK Setup
        robot_state_ = std::make_shared<moveit::core::RobotState>(scene->getRobotModel());
        robot_state_->setToDefaultValues();

        vis_oracle_->setFKSolver([this](const std::vector<double>& joints){
            robot_state_->setJointGroupPositions(group_name_, joints);
            robot_state_->update();
            return robot_state_->getGlobalLinkTransform(ee_link_name_);
        });

        workspace_bounds_ = {-5.0, 5.0, -5.0, 5.0, 0.0, 2.5};
        sampler_->setWorkspaceBounds(workspace_bounds_);
        validity_checker_->setWorkspaceBounds(workspace_bounds_);


        // 8. Initialize visibility integrity tool
        vis_integrity_ = std::make_shared<VisibilityIntegrity>();
        vis_integrity_->setSampler(sampler_);
        vis_integrity_->setVisibilityOracle(vis_oracle_);
        vis_integrity_->setWorkspaceBounds(workspace_bounds_);

        vis_roadmap_ = std::make_shared<VisibilityRoadmap>();
        vis_roadmap_->setSampler(sampler_);
        vis_roadmap_->setVisibilityOracle(vis_oracle_);
        vis_roadmap_->setWorkspaceBounds(workspace_bounds_); 

        // 9. Populate the Visibility Oracle with obstacles. The visibility structure is
        // deliberately NOT built here: its parameters (beam geometry, VI settings) are
        // loaded after construction, and building it now would use the defaults.
        initialize_visibility(false);
    }

    /**
     * @brief Extracts collision objects from the PlanningScene and adds them to VisibilityOracle.
     *
     * @param build_structure Whether to also build the active visibility structure.
     *        Pass false during construction, before the visibility parameters are
     *        loaded; the structure is built on the first setPlanningScene() instead.
     */
    void initialize_visibility(bool build_structure = true) {
        if (!planning_scene_) return;

        // Get list of collision objects in the world
        // Note: This gets attached bodies and world objects
        std::vector<std::string> object_ids = planning_scene_->getWorld()->getObjectIds();
        
        ROS_INFO("PlanningContext: Extracting %lu obstacles from PlanningScene...", object_ids.size());

        for (const std::string& id : object_ids) {
            // Get the object (CollisionObject)
            const collision_detection::World::ObjectConstPtr& obj = planning_scene_->getWorld()->getObject(id);
            if (!obj) continue;

            // Iterate through shapes in the object (could be mesh, box, etc.)
            for (size_t i = 0; i < obj->shapes_.size(); ++i) {
                // Compute AABB for the shape
                // We need to transform the shape pose to world frame
                Eigen::Isometry3d shape_pose = obj->shape_poses_[i]; 
                
                double x_min, y_min, z_min, x_max, y_max, z_max;
                
                if (obj->shapes_[i]->type == shapes::BOX) {
                    const shapes::Box* box = static_cast<const shapes::Box*>(obj->shapes_[i].get());
                    // Half extents
                    double hx = box->size[0]/2.0;
                    double hy = box->size[1]/2.0;
                    double hz = box->size[2]/2.0;
                    
                    // Transform the 8 corners and find min/max
                    // (Simplified: assuming axis aligned for now, but really should rotate)
                    // If we assume the objects are somewhat axis aligned or we take a loose bound:
                    Eigen::Vector3d center = shape_pose.translation();

                    x_min = center.x() - hx; x_max = center.x() + hx;
                    y_min = center.y() - hy; y_max = center.y() + hy;
                    z_min = center.z() - hz; z_max = center.z() + hz;
                    
                    vis_oracle_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                    validity_checker_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                    sampler_->addObstacle(x_min, y_min, z_min, x_max, y_max, z_max);
                } 
                else if (obj->shapes_[i]->type == shapes::SPHERE) {
                    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(obj->shapes_[i].get());
                    Eigen::Vector3d center = shape_pose.translation();
                    double r = sphere->radius;
                    vis_oracle_->addObstacle(center.x()-r, center.y()-r, center.z()-r, 
                                             center.x()+r, center.y()+r, center.z()+r);
                    validity_checker_->addObstacle(center.x()-r, center.y()-r, center.z()-r,
                                                   center.x()+r, center.y()+r, center.z()+r);
                }
                // Add MESH support if needed later
            }
        }
        
        if(build_structure && use_visibility_structure_) {
            setVisibilityIntegrityParams(visibility_integrity_params_);
            setVisibilityToolParams(visibility_tool_params_);

            if(use_visibility_roadmap_)
                vis_roadmap_->build();
            else
                vis_integrity_->build();
            
        }

    }
    void setWorkspaceBounds(const BoundingBox& bounds) {
        workspace_bounds_ = bounds;
        sampler_->setWorkspaceBounds(bounds);
        validity_checker_->setWorkspaceBounds(bounds);
        vis_integrity_->setWorkspaceBounds(bounds);
        vis_roadmap_->setWorkspaceBounds(bounds);
    }

    void setVisibilityToolParams(const VisibilityToolParams& params) {
        visibility_tool_params_ = params;
        vis_oracle_->setVisibilityToolParams(params);
        vis_ik_->setVisibilityToolParams(params);
        vis_integrity_->setVTParams(params);
        vis_roadmap_->setVTParams(params);

    }

    void setVisibilityThreshold(double t) {
        visibility_threshold_ = t;
        if (sampler_) sampler_->setVisibilityThreshold(t);
    }

    void setUseVisibilityRoadmap(bool enable) {
        use_visibility_roadmap_ = enable;
    }

    void setVisibilityIntegrityParams(const VisibilityIntegrityParams& params) {
        visibility_integrity_params_ = params;
        vis_integrity_->setVIParams(params);
        vis_roadmap_->setVIParams(params);
    }

    void setResolution(double res) { 
        resolution_ = res;
        validity_checker_->setResolution(res);
    }

    void setGroupName(const std::string& group) {
        group_name_ = group; 
        vis_ik_->setGroupName(group);
        sampler_->setGroupName(group);
        validity_checker_->setGroupName(group);
    }
    
    void setEELinkName(const std::string& ee_link) {
        ee_link_name_ = ee_link;
        vis_ik_->setEELinkName(ee_link);
    }
    
    std::string getGroupName() const { return group_name_; }
    std::string getEELinkName() const { return ee_link_name_; }

    bool isInitialized() const { return is_initialized_; }

    // ========================================================================
    // 1. Core Primitives
    Eigen::Isometry3d solveFK(const std::vector<double>& joints){
        robot_state_->setJointGroupPositions(group_name_, joints);
        robot_state_->update();
        return robot_state_->getGlobalLinkTransform(ee_link_name_);
    }
    VisualIK& getVisualIK() { return *vis_ik_; }
    PathSmoother& getSmoother() { return *path_smoother_; }
    ValidityChecker& getValidityChecker() { return *validity_checker_; }
    Sampler& getSampler() {return *sampler_;}

    /**
     * @brief Swaps in a fresh planning scene and rebuilds the visibility structures.
     *
     * Note this invalidates anything holding VI-tree node pointers, since the tree is
     * rebuilt from scratch.
     */
    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        planning_scene_ = scene;
        initialize_visibility();
        validity_checker_->setPlanningScene(scene);
        is_initialized_ = true;
    }

    void setUseVisibilityStructure(bool enable) { use_visibility_structure_ = enable; }

    /**
     * @brief The robot's current joint values for the planning group.
     *
     * Used to seed a planner's start configuration when none was set explicitly. The
     * context supplies it because it owns the scene and the group name; the start
     * configuration itself is per-run state and belongs to the planner.
     */
    std::vector<double> getCurrentJoints() const {
        std::vector<double> joints;
        if (!planning_scene_) return joints;
        planning_scene_->getCurrentState().copyJointGroupPositions(group_name_, joints);
        return joints;
    }

    /**
     * @brief The mobile base's (x, y) for a configuration of the planning group.
     *
     * Both target robots -- the simulated mobile UR5, whose base is two prismatic
     * joints, and the Husky + UR5 -- carry the base translation in the first two
     * variables of the planning group, so it is read directly rather than resolved by
     * joint name.
     *
     * This is the one place that assumption lives. Should the real Husky report its
     * base from localization rather than from a joint value, this method becomes the
     * service call and no caller changes.
     *
     * Throws on a configuration too short to hold a base position: that is a
     * programming error, and answering (0,0) would silently place the base at the
     * centre of the environment -- inside the locality disk in most scenes, so the
     * constraint would look enforced while enforcing nothing.
     */
    Eigen::Vector2d basePosition(const std::vector<double>& q) const {
        if (q.size() < 2) {
            throw std::runtime_error("PlanningContext::basePosition: configuration has no base variables.");
        }
        return Eigen::Vector2d(q[0], q[1]);
    }

    /**
     * @brief Samples a workspace point that sees `target`, using whichever visibility
     *        structure is active.
     */
    bool sampleVisibilityRegion(const Ball& target, Eigen::Vector3d& sample_pos) {
        if (use_visibility_roadmap_) {
            return vis_roadmap_->SampleFromVisibilityRegion(target, sample_pos, visibility_threshold_);
        }
        return vis_integrity_->SampleFromVisibilityRegion(target, sample_pos, visibility_threshold_);
    }

    /**
     * @brief Asks the VI-tree which regions see enough of each of several targets.
     *
     * Only the VI-tree answers this; the VIR roadmap has no multi-target query. A
     * caller that gets false has no region information and must not read that as "no
     * region sees these targets".
     *
     * @return false when the VI-tree is not the active structure, or was never built.
     */
    bool queryMultiTarget(const std::vector<Ball>& targets, MultiTargetQueryResult& out,
                          double threshold) {
        if (use_visibility_roadmap_ || !use_visibility_structure_) return false;
        if (!vis_integrity_ || !vis_integrity_->isBuilt()) return false;

        vis_integrity_->query(targets, out, threshold);
        return true;
    }
};

} // namespace visual_planner
