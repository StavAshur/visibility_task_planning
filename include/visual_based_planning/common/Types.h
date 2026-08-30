#pragma once

#include <string>

#include <Eigen/Core>

namespace visual_planner {

    // Defined here so it is accessible to VisualIK, Sampler, and Planner
    struct Ball {
        Eigen::Vector3d center;
        double radius;
    };

    enum class EdgeCheckMode {
        LINEAR,
        BINARY_SEARCH
    };

    struct BoundingBox {
        double x_min, x_max;
        double y_min, y_max;
        double z_min, z_max;
    };

    struct RRTParams {
        double goal_bias = std::pow(0.5, 7);         
        double max_extension = 0.1;      
        int max_iterations = 5000;       
    };

    struct PRMParams {
        int num_neighbors = 10;          
        int num_samples = 50;  
        int max_goals = 10;
        EdgeCheckMode edge_validation_method = EdgeCheckMode::BINARY_SEARCH;
        int max_size = 10000;
    };

    struct VisibilityIntegrityParams {
        int num_samples = 1000;
        int num_roadmap_samples = 12000;
        double vi_threshold = 0.7;
        int k_neighbors = 15;
        int face_samples = -1;
        double limit_diameter_factor = 1000.0;
    };
    

    struct VisibilityToolParams {
        double beam_angle = M_PI / 12.0;  // Half-angle in radians
        double beam_length = 3.0;
    };

    /**
     * @brief Optional cap on how far the mobile base may travel during one query.
     *
     * A disk of radius `delta` around `center`, which is the base position of the
     * query's root configuration. Used by the multi-target coverage queries so the
     * same planner can serve as the LOCAL planner of the VisTSP pipeline, where the
     * base has to stay near one location while the arm does the looking.
     *
     * A disk rather than a box because the constraint it expresses is a distance, and
     * because convexity is what makes it cheap to enforce: the base joints are
     * prismatic, so the straight line between two in-disk configurations keeps the
     * base in the disk. Checking the endpoints of an edge is therefore enough -- no
     * constraint checking is needed inside edge validation.
     *
     * Where the base is for a given configuration is asked through
     * PlanningContext::basePosition().
     */
    struct BaseLocality {
        bool enabled = false;
        double delta = 0.0;                     ///< Radius in metres.
        Eigen::Vector2d center =
            Eigen::Vector2d::Zero();            ///< Base (x,y) of the root configuration.

        /// The joint group VisualIK is restricted to while the constraint is active:
        /// the arm alone, so a solution cannot move the base out of the disk. See
        /// VisualIK::setIKGroupName().
        std::string ik_group = "manipulator";
    };
} // namespace visual_planner



