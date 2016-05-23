/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "QuadCopterVFFAvoidance.hh"

#include <cerrno>
#include <tuple>
#include <chrono>
#include "common/math.hh"

#define MIN_THETA 0.01
#define MIN_PHI 0.01
#define COAV_CALC_PERIOD_MS 100
#define COAV_TARGET_SEC_MARGIN_M 1.5

double get_closest_obst_dist(const std::vector<Obstacle> &obstacles)
{

    // Find the distance from the closest obstacle
    double closest_obst_dist = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < obstacles.size(); ++i) {
        Obstacle obst_pos = obstacles[i];
        double length = glm::length(obst_pos.center);
        if (length < closest_obst_dist) {
            closest_obst_dist = length;
        }
    }

    return closest_obst_dist;
}

PolarVector get_critical_point(const Obstacle& obstacle)
{
    PolarVector cp;
    cp.theta = atan(obstacle.center.x / obstacle.center.z);
    cp.phi = atan(obstacle.center.y / obstacle.center.z);
    return cp;
}

void QuadCopterVFFAvoidance::avoid(const std::vector<Obstacle> &obstacles, std::shared_ptr<QuadCopter> vehicle)
{
    /* TODO: WORK IN PROGRESS */

    /* Get Target relative coordinates */
    Pose target_pose = vehicle->target_pose();
    Pose vehicle_pose = vehicle->vehicle_pose();
    Pose relative_pose = target_pose - vehicle_pose;

    PolarVector target = cartesian_to_spherical(
        relative_pose.pos.x, relative_pose.pos.y, relative_pose.pos.z);

    /* Target parameters and variables */
    float kg = 1.0;
    float ko = 1.0;
    float c1 = 1.0;
    float c2 = 1.0;

    float theta_g = target.theta;
    float phi_g = target.phi;
    float dg = target.len;

    float attract_x = kg * theta_g * (exp(-c1 * dg) + c2);
    float attract_y = kg * phi_g * (exp(-c1 * dg) + c2);

    /* Obstacle parameters and variables */
    float repulse_x = 0.0;
    float repulse_y = 0.0;
    float theta_o = 0.0;
    float phi_o = 0.0;
    float d_o = 0.0;
    float t1 = 1.0;
    float t2 = 1.0;
    float s1 = 1.0;
    float s2 = 1.0;
    float c3 = 1.0;
    float c4 = 1.0;
    PolarVector critical_point;

    for (unsigned int i = 0; i < obstacles.size(); ++i) {
        critical_point = get_critical_point(obstacles[i]);
        theta_o = critical_point.theta;
        phi_o = critical_point.phi;
        d_o = critical_point.len;

        /* If the angles are below a specific threshold, consider them as equal
         * to -/+ threshold. */
        if (fabs(theta_o) < MIN_THETA)
            theta_o = copysign(MIN_THETA, theta_o);
        if (fabs(phi_o) < MIN_PHI)
            phi_o = copysign(MIN_PHI, phi_o);

        repulse_x += -ko * sign(theta_o) *
                     sigmoid(s1 * (1.0 - fabs(phi_o) / s2)) * exp(-c3 * d_o) *
                     exp(-c4 * fabs(theta_o));
        repulse_y += -ko * sign(phi_o) *
                     sigmoid(t1 * (1.0 - fabs(theta_o) / t2)) * exp(-c3 * d_o) *
                     exp(-c4 * fabs(phi_o));
    }

    // std::cout << "[QuadCopterVFFAvoidance] "
              // << "Rates: " << (attract_x + repulse_x) << ", "
              // << (attract_y + repulse_y) << std::endl;

    std::tuple<double, double> rates(attract_x + repulse_x, attract_y + repulse_y);
    double dist = get_closest_obst_dist(obstacles);

    // std::cout << "Num obstacles, dist: " << obstacles.size() << dist << std::endl;

    // This is the code for drone control
    using namespace std::chrono;

    // Check if its time to calculate the coav waypoint
    time_point<system_clock> curr_time = system_clock::now();
    if (duration_cast<milliseconds>(curr_time - last_calc_time).count() <
        COAV_CALC_PERIOD_MS) {
        return;
    }

    // Get Vehicle and Target Pose
    PolarVector target_polar_pos = cartesian_to_spherical(
        relative_pose.pos.x, relative_pose.pos.y, relative_pose.pos.z);

    glm::dvec3 coav_wp = calculate_coav_wp(vehicle_pose,
        std::get<0>(rates), std::get<1>(rates), dist, target_polar_pos.len);

    // Set new Target Position According to COAV
    target_pose = Pose{coav_wp, glm::dquat(0, 0, 0, 0)};

    vehicle->set_target_pose(target_pose);

    last_calc_time = curr_time;
}

glm::dvec3
QuadCopterVFFAvoidance::calculate_coav_wp(Pose pose, double turn_rate,
                                   double climb_rate, double closest_obst_dist,
                                   double target_dist)
{
    // The new waypoint is positioned on the suface of an imaginary sphere of
    // radius R centender in the Drone's current position. Phi and theta are
    // calculated according to the turn rate and the climb rate. R is given by
    // min(closest_obst_dist, target_dist).
    double R = 0;

    if (closest_obst_dist < target_dist) {
        R = closest_obst_dist - COAV_TARGET_SEC_MARGIN_M;
    } else {
        R = target_dist;
    }

    double theta = turn_rate;

    // Considering that the look-at vector can be represented
    // as the axis X rotated around Z by the YAW angle, then.
    glm::dvec3 view_dir(
        R * cos(glm::yaw(pose.rot)),
        R * sin(glm::yaw(pose.rot)),
        0
    );

    glm::dvec3 new_waypoint_dir(
        view_dir.x * cos(theta) - view_dir.y * sin(theta),
        view_dir.x * sin(theta) + view_dir.y * cos(theta),
        0
    );

    return pose.pos + fabs(R) * glm::normalize(new_waypoint_dir);
}

