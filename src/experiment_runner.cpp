#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visual_based_planning/PlanVisibilityPath.h>
#include <random>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>


#include <yaml-cpp/yaml.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

// Struct for representing bounding boxes
struct AABB {
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
};

// Structure to hold a pre-generated target problem
struct TargetProblem {
    int id;
    float cx, cy, cz;
    float radius;
    std::vector<float> points_flat; 
};

struct ExperimentConfig {
    std::string name;
    std::string planner_mode;        
    bool snap_fov_on_insert;
    bool use_visibility_integrity;
    bool use_visibility_roadmap;
};

struct Stats {
    double avg = 0.0;
    double median = 0.0;
    double min = 0.0;
    double max = 0.0;
    double std_dev = 0.0;
    int count = 0;
};

struct ExperimentResult {
    std::string config_name;
    double first_run_time;                 
    
    // Raw Data (Runs 1-99, excluding init)
    std::vector<double> all_times; 
    std::vector<double> successful_times;

    // Success tracking
    int success_count = 0;
    int fail_count = 0;
    std::vector<int> outcomes;             

    // Computed Stats
    Stats stats_all;
    Stats stats_success;
};

class ExperimentRunner {
public:
    ExperimentRunner(ros::NodeHandle& nh) : nh_(nh) {
        client_ = nh_.serviceClient<visual_based_planning::PlanVisibilityPath>("plan_visibility_path");
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    void runExperiments() {
        ROS_INFO("Waiting for planning service...");
        client_.waitForExistence();
        ROS_INFO("Service available.");

        ROS_INFO("Pre-generating 200 target problems...");
        std::string yaml_path;
        nh_.param<std::string>("sampling_regions_file", yaml_path, "/home/roblab22/catkin_ws/src/visual_based_planning/config/sampling_regions_double_room.yaml");
        generateTargetProblemsFromRegions(yaml_path);

        std::vector<ExperimentConfig> configs = {
            {"VisRRT",  "VisRRT", true,  false, false},
             {"RRT", "VisRRT", false, false, false},
             {"VisPRM",  "VisPRM", false, true, false},
             {"VIR",  "VisPRM", false, true, true},
             {"PRM", "VisPRM", false, false, false}
        };

        std::vector<ExperimentResult> all_results;

        for (const auto& cfg : configs) {
            ROS_INFO("------------------------------------------------");
            ROS_INFO("Starting Configuration: %s", cfg.name.c_str());
            
            nh_.setParam("/planner/mode", cfg.planner_mode);
            nh_.setParam("/planner/snap_fov_on_insert", cfg.snap_fov_on_insert);
            // No experiment here scans a roadmap for FOV snapping, and pinning it keeps
            // a run from inheriting whatever the previous one left on the param server.
            nh_.setParam("/planner/snap_fov_on_scan", false);
            nh_.setParam("/planner/visibility_integrity/enabled", cfg.use_visibility_integrity);
            nh_.setParam("/planner/use_visibility_integrity", cfg.use_visibility_integrity);
            nh_.setParam("/planner/use_visibility_roadmap", cfg.use_visibility_roadmap);

            ros::Duration(0.5).sleep(); 

            ExperimentResult result;
            result.config_name = cfg.name;

            int num_trials = 200;
            for (int i = 0; i < num_trials; ++i) {
                
                nh_.setParam("/target_points", problems_[i].points_flat);

                visual_based_planning::PlanVisibilityPath srv;
                const std::vector<float>& pts = problems_[i].points_flat;
                for (size_t k = 0; k + 2 < pts.size(); k += 3) {
                    geometry_msgs::Point p;
                    p.x = pts[k];
                    p.y = pts[k+1];
                    p.z = pts[k+2];
                    srv.request.task.target_points.push_back(p);
                }

                ros::WallTime start_time = ros::WallTime::now();
                bool call_success = client_.call(srv);
                ros::WallTime end_time = ros::WallTime::now();
                
                double duration = (end_time - start_time).toSec();
                bool run_success = (call_success && srv.response.success);

                // Track Outcomes
                if (run_success) {
                    result.success_count++;
                    result.outcomes.push_back(1);
                } else {
                    result.fail_count++;
                    result.outcomes.push_back(0);
                    if (!call_success) ROS_WARN("Service call failed (Comm Error) on trial %d", i);
                }

                // Store Data
                if (i == 0) {
                    result.first_run_time = duration;
                    ROS_INFO("  [Init]: %.4f sec | Result: %s", duration, run_success ? "SUCCESS" : "FAIL");
                }
                result.all_times.push_back(duration);
                if (run_success) {
                    result.successful_times.push_back(duration);
                }
 
                if (i % 10 == 0 && i > 0) ROS_INFO("  Progress: %d/%d", i, num_trials);
            }

            // Compute Stats for both All and Successful sets
            result.stats_all = computeStats(result.all_times);
            result.stats_success = computeStats(result.successful_times);
            
            printStats(result);
            all_results.push_back(result);
        }

        saveToFile(all_results);
        ROS_INFO("Experiments completed. Data saved to 'planner_experiments_results.txt'.");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    std::mt19937 rng_;
    std::vector<TargetProblem> problems_;
    int door_room_counter_=0;
    int windows_room_counter_=0;
    bool is_door_;
    std::vector<bool> room_flags_;

    void generateTargetProblemsFromRegions(const std::string& yaml_file_path) {
        problems_.clear();
        problems_.reserve(200);
        room_flags_.clear();
        room_flags_.reserve(200);

        // 1. Read Sampling Regions from YAML
        std::vector<AABB> regions;
        try {
            YAML::Node config = YAML::LoadFile(yaml_file_path);
            for (const auto& node : config["sampling_regions"]) {
                AABB region;
                region.min_x = node["min"][0].as<float>();
                region.min_y = node["min"][1].as<float>();
                region.min_z = node["min"][2].as<float>();
                region.max_x = node["max"][0].as<float>();
                region.max_y = node["max"][1].as<float>();
                region.max_z = node["max"][2].as<float>();
                regions.push_back(region);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load sampling regions from %s. Error: %s", yaml_file_path.c_str(), e.what());
            return;
        }

        if (regions.empty()) {
            ROS_ERROR("No sampling regions found. Aborting generation.");
            return;
        }

        // 2. Extract Obstacles (AABBs) from MoveIt Planning Scene
        moveit::planning_interface::PlanningSceneInterface psi;
        std::map<std::string, moveit_msgs::CollisionObject> objects = psi.getObjects();
        std::vector<AABB> obstacles;

        for (const auto& kv : objects) {
            const auto& obj = kv.second;
            for (size_t i = 0; i < obj.primitives.size(); ++i) {
                if (obj.primitives[i].type == shape_msgs::SolidPrimitive::BOX) {
                    float dx = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X];
                    float dy = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                    float dz = obj.primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

                    // MoveIt populates primitive_poses to correspond with primitives
                    float cx = obj.primitive_poses[i].position.x;
                    float cy = obj.primitive_poses[i].position.y;
                    float cz = obj.primitive_poses[i].position.z;

                    AABB obs;
                    obs.min_x = cx - dx / 2.0f;
                    obs.max_x = cx + dx / 2.0f;
                    obs.min_y = cy - dy / 2.0f;
                    obs.max_y = cy + dy / 2.0f;
                    obs.min_z = cz - dz / 2.0f;
                    obs.max_z = cz + dz / 2.0f;
                    obstacles.push_back(obs);
                }
            }
        }

        ROS_INFO("Extracted %zu obstacles from the planning scene.", obstacles.size());

        // 3. Generate Target Problems
        std::uniform_int_distribution<int> dist_region(0, regions.size() - 1);
        std::normal_distribution<float> dist_norm(0.0, 1.0);
        std::uniform_real_distribution<float> dist_u(0.0, 1.0);

        for (int i = 0; i < 200; ++i) {
            TargetProblem prob;
            prob.id = i;
            
            bool valid = false;
            while (!valid) {
                // Select a region uniformly at random
                const AABB& reg = regions[dist_region(rng_)];
                
                // Sample center point inside the selected AABB
                std::uniform_real_distribution<float> dist_x(reg.min_x, reg.max_x);
                std::uniform_real_distribution<float> dist_y(reg.min_y, reg.max_y);
                std::uniform_real_distribution<float> dist_z(reg.min_z, reg.max_z);
                
                prob.cx = dist_x(rng_);
                prob.cy = dist_y(rng_);
                prob.cz = dist_z(rng_);
                
                // Compute max radius contained within the AABB (shortest distance to bounds)
                float r_x = std::min(prob.cx - reg.min_x, reg.max_x - prob.cx);
                float r_y = std::min(prob.cy - reg.min_y, reg.max_y - prob.cy);
                float r_z = std::min(prob.cz - reg.min_z, reg.max_z - prob.cz);
                prob.radius = std::min({r_x, r_y, r_z});
                
                if (prob.radius <= 0.0f) continue; // Skip degenerate bounds

                // Verify intersection with scene obstacles
                bool intersection_found = false;
                for (const auto& obs : obstacles) {
                    if (checkSphereAABBIntersection(prob.cx, prob.cy, prob.cz, prob.radius, obs)) {
                        intersection_found = true;
                        ROS_INFO(" target sphere with center (%.4f,  %.4f, %.4f) is in collision", prob.cx, prob.cy, prob.cz);
                        break;
                    }
                }
                
                if (!intersection_found) {
                    valid = true;
                    bool room_flag = (prob.cx > 0.0);
                    room_flags_[i] = room_flag;
                }

            }
            


            // Sample 100 points inside the valid sphere
            prob.points_flat.reserve(100 * 3);
            for (int j = 0; j < 100; ++j) {
                float x = dist_norm(rng_);
                float y = dist_norm(rng_);
                float z = dist_norm(rng_);
                float norm = std::sqrt(x*x + y*y + z*z);
                x /= norm; y /= norm; z /= norm;
                
                float r_scale = prob.radius * std::cbrt(dist_u(rng_));
                prob.points_flat.push_back(prob.cx + x * r_scale);
                prob.points_flat.push_back(prob.cy + y * r_scale);
                prob.points_flat.push_back(prob.cz + z * r_scale);
            }
            problems_.push_back(prob);
        }
        ROS_INFO("Successfully generated 200 non-intersecting targets.");
    }

    Stats computeStats(const std::vector<double>& data) {
        Stats s;
        s.count = data.size();
        if (data.empty()) return s;
        
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        s.avg = sum / data.size();

        std::vector<double> sorted = data;
        std::sort(sorted.begin(), sorted.end());
        s.min = sorted.front();
        s.max = sorted.back();
        
        if (sorted.size() % 2 == 0)
            s.median = (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]) / 2.0;
        else
            s.median = sorted[sorted.size()/2];

        double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
        double variance = (sq_sum / data.size()) - (s.avg * s.avg);
        s.std_dev = std::sqrt(std::max(0.0, variance)); // max with 0 to avoid tiny negative from float precision
        return s;
    }

    void printStats(const ExperimentResult& res) {
        std::cout << "\nResults for " << res.config_name << ":\n";
        std::cout << "  Success: " << res.success_count << " | Fail: " << res.fail_count << "\n";
        std::cout << "  Init Time: " << res.first_run_time << " s\n";
        
        std::cout << "  -- All Runs --\n";
        std::cout << "    Avg: " << res.stats_all.avg << ", Med: " << res.stats_all.median << "\n";
        
        std::cout << "  -- Successful Runs Only --\n";
        if (res.stats_success.count > 0) {
            std::cout << "    Avg: " << res.stats_success.avg << ", Med: " << res.stats_success.median << "\n";
            std::cout << "    Min: " << res.stats_success.min << ", Max: " << res.stats_success.max << "\n";
        } else {
            std::cout << "    (No successful runs to analyze)\n";
        }
        std::cout << "\n";
    }

    void saveToFile(const std::vector<ExperimentResult>& results) {
        std::ofstream file("planner_experiments_results.txt");
        if (!file.is_open()) {
            ROS_ERROR("Unable to open file for writing results.");
            return;
        }

        file << "Experiment Results\n";
        file << "==================\n";

        for (const auto& res : results) {
            file << "Configuration: " << res.config_name << "\n";
            file << "Total Successes: " << res.success_count << "\n";
            file << "Total Failures:  " << res.fail_count << "\n";
            file << "Initialization Time: " << res.first_run_time << "\n";
            
            file << "\n--- Stats: ALL Runs ---\n";
            file << "  Count:   " << res.stats_all.count << "\n";
            file << "  Average: " << res.stats_all.avg << "\n";
            file << "  Median:  " << res.stats_all.median << "\n";
            file << "  Min:     " << res.stats_all.min << "\n";
            file << "  Max:     " << res.stats_all.max << "\n";
            file << "  StdDev:  " << res.stats_all.std_dev << "\n";

            file << "\n--- Stats: SUCCESSFUL Runs Only ---\n";
            file << "  Count:   " << res.stats_success.count << "\n";
            if (res.stats_success.count > 0) {
                file << "  Average: " << res.stats_success.avg << "\n";
                file << "  Median:  " << res.stats_success.median << "\n";
                file << "  Min:     " << res.stats_success.min << "\n";
                file << "  Max:     " << res.stats_success.max << "\n";
                file << "  StdDev:  " << res.stats_success.std_dev << "\n";
            } else {
                file << "  (N/A - No Successes)\n";
            }
            
            file << "\nRun Outcomes (1=Success, 0=Fail): ";
            for (size_t i = 0; i < res.outcomes.size(); ++i) {
                file << res.outcomes[i] << (i < res.outcomes.size() - 1 ? ", " : "");
            }
            file << "\n";
            file << "\nTarget room (1=windows (right), 0=door (left)): ";
            for (size_t i = 0; i < res.outcomes.size(); ++i) {
                file << room_flags_[i] << (i < res.outcomes.size() - 1 ? ", " : "");
            }
            file << "\n";

            file << "Raw Times (Run 0 to 99): ";
            for (double t : res.all_times) {
                file << ", " << t;
            }
            file << "\n\n";
        }
        file.close();
    }

    bool checkSphereAABBIntersection(float cx, float cy, float cz, float radius, const AABB& box) {
        float sqDist = 0.0f;

        // Calculate squared distance from the sphere center to the box
        float v = cx;
        if (v < box.min_x) sqDist += (box.min_x - v) * (box.min_x - v);
        if (v > box.max_x) sqDist += (v - box.max_x) * (v - box.max_x);

        v = cy;
        if (v < box.min_y) sqDist += (box.min_y - v) * (box.min_y - v);
        if (v > box.max_y) sqDist += (v - box.max_y) * (v - box.max_y);

        v = cz;
        if (v < box.min_z) sqDist += (box.min_z - v) * (box.min_z - v);
        if (v > box.max_z) sqDist += (v - box.max_z) * (v - box.max_z);

        return sqDist <= (radius * radius);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "experiment_runner");
    ros::NodeHandle nh;
    ExperimentRunner runner(nh);
    runner.runExperiments();
    return 0;
}
