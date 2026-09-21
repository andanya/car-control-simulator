#include "world.h"
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <tuple>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std;

World world;
double a, df;
mutex mtx;
bool pressed;

const double crosswalk_width = 40;
const double max_ped_speed = 40;
const int road_end = World::ROAD_LENGTH + 1;
const double ROAD_WIDTH = 4 * World::LANE_WIDTH;


double process_peds(const vector<DetectedPedestrian>& peds,
                    const Car& car,
                    double closestX,
                    double dang_ped_dist) {
    for (const auto& p : peds) {
        // Take only those that can get to the crosswalk and be able to collide
        // cerr << closestX - dang_ped_dist - car.x << "  " << closestX + crosswalk_width + dang_ped_dist - car.x << endl;
        if (p.x >= closestX - dang_ped_dist && p.x <= closestX + crosswalk_width + dang_ped_dist) {
            if (p.y < car.y + Car::LENGTH / 2 * sin(car.psi) - Car::WIDTH / 2 - World::LANE_WIDTH && sin(p.yaw) < -0.5)
                continue;
            if (p.y > car.y + Car::LENGTH / 2 * sin(car.psi) + Car::WIDTH / 2 + World::LANE_WIDTH && sin(p.yaw) > 0.5)
                continue;
            if (p.x < closestX  && cos(p.yaw) < -0.5)
                continue;
            if (p.x > closestX + crosswalk_width && cos(p.yaw) > 0.5)
                continue;
            // if (car.x + Car::LENGTH / 2 + s * 0.5 > p.x)
            //     continue;

            return -100;
        }
    }
    return 100;
}


double next_obstacle_x = 0;
double next_next_obstacle_x = 0;
double third_obstacle_x = 0;
Obstacle next_obstacle{road_end, 60, 2};
double next_traj_score = 0;
double new_car_y = 45;

double getAcceleration_without_peds(const Car& car) {
//     if (car.v < 10) {
//         return 100;
//     }

//     if (abs(new_car_y - car.y) / (1e-6 + abs(next_obstacle_x - car.x)) > 0.2 && car.v > 10) {
//         return -100;
//     }

//     if (abs(car.psi) > 0.7 && car.v > 50) {
//         return -100;
//     }

//     // const double max_brake = 50;
//     // const double s = car.v * car.v / 2 / max_brake;


// ////////////////
//     if (car.psi < -0.025 && car.y < 1.5 * Car::WIDTH && car.v > 5) { // was beta before
//         return -100;
//     }

//     if (car.psi > 0.025 && car.y > ROAD_WIDTH - 1.5 * Car::WIDTH && car.v > 5) { // was beta before
//         return -100;
//     }

    if (abs(car.beta) > 0.05 && car.v > 10) {
        // cerr << "slow down" << endl;
        return -100;
    }

    if (abs(car.psi) > 0.29 && car.v > 13) { //15
        return -100;
    }

    // if (abs(car.beta) > 0.02 && car.v > 10) {
    //     return 0;
    // }



    if (next_traj_score > 0.15 && next_obstacle_x < car.x + 4 * Car::LENGTH && car.v > 35) { // 5 * Car::Length
        // cerr << ":::::::::::::::: next_traj_score!" << endl;
        return -100;
    }

//     if (car.v > 100 && next_traj_score > 0.10 && next_next_obstacle_x < car.x + 20 * Car::LENGTH) {
//         return -100;
//     }

//     if (car.v > 100 && next_traj_score > 0.10 && third_obstacle_x < car.x + 25 * Car::LENGTH) {
//         return -100;
//     }


    return 100;
}

double getAcceleration(
    const vector<DetectedPedestrian>& peds,
    const vector<Crosswalk>& crosswalks,
    const vector<Obstacle>& obstacles,
    const Car& car) {

    // return 0;

    const double ped_fear_rad = 28;

    const double max_acc = 50;
    const double max_brake = 50;

    const double s = car.v * car.v / 2 / max_brake;

    // if (car.v > 100) {
    //     return -100;
    // } else if (car.v < 10) {
    //     return 100;
    // }
    // cerr << "car.v = " << car.v << " car.beta = " << car.beta << " car.psi" << car.psi << endl;


    double upper_edge = car.y + s * sin(car.psi) + Car::LENGTH * sin(car.psi) / 2 - Car::WIDTH * cos(car.psi)/ 2 - 3;
    double lower_edge = car.y + s * sin(car.psi) + Car::LENGTH * sin(car.psi) / 2 + Car::WIDTH * cos(car.psi)/ 2 + 3;
    // cerr << "upper_edge = " << upper_edge << " lower_edge = " << lower_edge << endl;

    if ((upper_edge < 0 || lower_edge > ROAD_WIDTH) && car.v > 1) {
        // cerr << "Edge of the road!" << endl;
        return -100;
    } 

    upper_edge = car.y + (next_obstacle_x - car.x - Car::LENGTH) * tan(car.psi) + Car::LENGTH * sin(car.psi) / 2 - Car::WIDTH * cos(car.psi)/ 2 - 3;
    lower_edge = car.y + (next_obstacle_x - car.x - Car::LENGTH) * tan(car.psi) + Car::LENGTH * sin(car.psi) / 2 + Car::WIDTH * cos(car.psi)/ 2 + 3;

    if (upper_edge < next_obstacle.y + next_obstacle.r && lower_edge > next_obstacle.y - next_obstacle.r && car.v > 2) {
        if (next_obstacle_x < car.x + s + Car::LENGTH / 2 + 2) {
            // cerr << "Can crash obstacle" << endl;
            return -100;
        }
    } 

    int closestX = road_end;
    for (const auto& cr : crosswalks)
        if (cr.lx > car.x && cr.lx < closestX) {
            closestX = cr.lx;
        }


    // we should be able to stop before any of the next crosswalks

    double factor_ = 1.1;
    if (next_traj_score > 0.15) {
        factor_ = 2.3;
    }
    for (const auto& cr : crosswalks) {
        if (cr.lx > closestX && cr.lx - factor_ * Car::LENGTH < car.x + s && car.v > 1) {
            // cerr << "further crosswalks danger" << endl;
            return -100; 
        }
    }

    // we should be able to stop before any of the next obstacles
    for (const auto& o : obstacles) {
        if (o.x - o.r > next_obstacle_x && o.x - o.r - Car::LENGTH / 2 - 2  < car.x + s && car.v > 1) {
            // cerr  << "further obstacles danger" << endl;
            return -100; 
        }
    }


    // if we are standing on a crosswalk, drive away


   for (const auto& cr: crosswalks) {
        if (car.x + Car::LENGTH / 2 + ped_fear_rad > cr.lx && car.x < cr.rx && car.v < 20) {// was car.v < 50
            if (abs(car.psi) < 0.015 || car.v < 20) {
                return 20;
            }
        }
    }

    // evaluating stopping distance
        // const double t = car.v / max_brake;
        // const double s = car.v * t + (-max_brake) * t * t / 2;


    // if closest crosswalk is farther then stopping length, accelerate
    if (closestX - 1.7 * Car::LENGTH > car.x + s) {
        return getAcceleration_without_peds(car);
    }
    // if car is on the crosswalk, accelerate
    // if (car.x + (1.5 * Car::LENGTH) > closestX) {
    //    return 100;
    // }

    // pedestrians processings
    
    const double v_reg = 0.00001;

    const double cr_pass_distance = max(closestX + crosswalk_width + ped_fear_rad - car.x, 0.);
    const double dang_ped_dist_v = cr_pass_distance / (car.v + v_reg) * max_ped_speed;
    const double dang_ped_dist_a = 1.8 * sqrt(cr_pass_distance * 2 / max_acc) * max_ped_speed;
    // const double dang_ped_dist_a = sqrt(cr_pass_distance * 2 / max_acc) * max_ped_speed;

    const double dang_ped_dist = min(dang_ped_dist_v, dang_ped_dist_a);


    if (process_peds(peds, car, closestX, dang_ped_dist) == -100) {
        // cerr << "peds make stop" << endl;
        return -100;
    }


    // if next crosswalk is too close, slow down

    double next_closestX = road_end;

    for (const auto& cr : crosswalks) {
        if (cr.lx > closestX && cr.lx < closestX + 3 * Car::WIDTH) {
            next_closestX = cr.lx;
        }
    }



    if (next_closestX != road_end) {
        // if (next_closestX <= car.x + 1.5 * Car::LENGTH + s) {
        //     return -100;
        //     cerr << "Next crosswalk is close -- stopping length" << endl;
        // }
        if (next_closestX - car.x < 5 * Car::LENGTH && process_peds(peds, car, next_closestX, dang_ped_dist) == -100) {
            // cerr << "Next crosswalk is close -- peds" << endl;
            return -100;
        }
    }
    return getAcceleration_without_peds(car);
}




vector<Obstacle>& closest_obstacles(const vector<Obstacle>& obstacles,
                                    const Car& car,
                                    vector<Obstacle>& three_obstacles) {
    three_obstacles.clear();
    // Obstacle closest_first{road_end, 0, 0};
    // Obstacle closest_second{road_end, 0, 0};
    // Obstacle closest_third{road_end, 0, 0};

    // for (auto& obstacle : obstacles) {
    //     double left_edge = obstacle.x - obstacle.r;
    //     if (left_edge > car.x + Car::WIDTH / 2 && left_edge < closest_first.x - closest_first.r) {
    //         closest_third = closest_second;
    //         closest_second = closest_first;
    //         closest_first = obstacle;
    //     }
    // }
    
    // three_obstacles.push_back(closest_first);
    // three_obstacles.push_back(closest_second);
    // three_obstacles.push_back(closest_third);
    three_obstacles.clear();
    for (vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); ++it) {
        double left_edge = it->x - it->r;
        if (left_edge > car.x + Car::WIDTH / 2) {
            three_obstacles.push_back(*it);
            if (it + 1 != obstacles.end()) {
                three_obstacles.push_back(*(it + 1));
            }
            if (it + 2 != obstacles.end()) {
                three_obstacles.push_back(*(it + 2));
            }
            break;
        }
    }
    return three_obstacles;
}

void print_point(const vector<double> vec) {
    cerr << "(" << vec[0] << ", " << vec[1] << ")" << endl;
}

void print_trajectory(const vector<vector<double>>& traj) {
    // cerr << "Traj size = " << traj.size() << endl;
    for (auto& point : traj) {
        // cerr << "point size " << point.size() << endl;
        cerr << "(" << point[0] << ", " << point[1] << ")" << "  ->  ";
    }
    cerr << endl;
}

double max_traj_slope (const vector<vector<double>>& traj, const Car& car) {
    double max_slope = abs((traj[0][1] - car.y) / (1e-6 + abs(traj[0][0] - car.x)));
    // cerr << "max_slope = " << max_slope << " ";
    double slope = 0;
    for (vector<vector<double>>::const_iterator it = traj.begin(); it != traj.end(); ++it) {
        if (it + 1 != traj.end()) {
            slope = abs(((*(it + 1))[1] - (*it)[1]) / (1e-6 + abs((*(it + 1))[0] - (*it)[0])));
            if (slope > max_slope) {
                max_slope = slope;
            } 
        }
    }
    return max_slope; 
}

void allowed_next(const vector<double> initial,
                  const Obstacle& obstacle,
                  vector<vector<double>>& allowed
                 ) {
    allowed.clear();
    // cerr << "allowed_next:: initial" << "(" << initial[0] << ", " << initial[1] << ") o.x = " << obstacle.x << " o.y = " << obstacle.y << endl;  
    // print_point(initial);

    if (initial.size() != 2) {
        // cerr << "allowed_next:: Bad initial" << endl;
        return;
    }



    if (initial[1] - Car::WIDTH / 3. - 5. > obstacle.y + obstacle.r || initial[1] + Car::WIDTH / 2 + 3. < obstacle.y - obstacle.r) {
        vector<double> straight{double(obstacle.x), initial[1]};
        // print_point(straight);
        allowed.push_back(straight);
    }

    double upper_gap = obstacle.y - obstacle.r - 10; // -10
    double lower_gap = ROAD_WIDTH - (obstacle.y + obstacle.r) - 10; // -10
    // cerr << obstacle.x << " " << obstacle.y << " " << obstacle.r << " " << ROAD_WIDTH << endl;
    // cerr << upper_gap << " " << lower_gap << endl;

    if (upper_gap > Car::WIDTH) {
        // cerr << "upper!" << endl;
        vector<double> upper{double(obstacle.x), upper_gap / 2};
        // print_point(upper);
        allowed.push_back(upper);
    }
    if (lower_gap > Car::WIDTH) {
        // cerr << "lower!" << endl;
        vector<double> lower{double(obstacle.x), obstacle.y + obstacle.r + lower_gap / 2 + 5}; // + 5
        // print_point(lower);
        allowed.push_back(lower);
    }
    // cerr << "allowed next: " << endl;
    // for (auto& p : allowed) {
    //     print_point(p);
    // }
}



void make_route(const vector<Obstacle>& obstacles, const Car& car) {
    // cerr << "obst size = " << obstacles.size() << endl;

    static vector<Obstacle> three_obstacles{};
    closest_obstacles(obstacles, car, three_obstacles);

    Obstacle ob{int(road_end), 0, 1};
    Obstacle ob_{int(road_end + 1), road_end, 1};
    Obstacle ob__{int(road_end + 2), 0, 1};

    if (three_obstacles.size() < 3) {
        three_obstacles.push_back(ob);
    }
    if (three_obstacles.size() < 3) {
        three_obstacles.push_back(ob_);
    }
    if (three_obstacles.size() < 3) {
        three_obstacles.push_back(ob__);
    }

    next_obstacle_x = three_obstacles[0].x - three_obstacles[0].r;
    next_obstacle = three_obstacles[0];

    next_next_obstacle_x = three_obstacles[1].x - three_obstacles[1].r;
    third_obstacle_x = three_obstacles[2].x - three_obstacles[2].r;
    // for (auto& o : three_obstacles){
    //     cerr << "Nearest " << o.x << " " << o.y << endl;
    // }

    vector<vector<double>> allowed{};
    vector<vector<double>> new_allowed{};

    vector<double> initial{car.x, car.y};
    allowed_next(initial, three_obstacles[0], allowed);
    if (allowed.size() == 0) {
        new_car_y = car.y;
        return;
    }
    // cerr << "::first" << endl;
    // cerr << "before first for " << endl;
    // for (auto& o : allowed){
    //     cerr << "allowed 1: " << o[0] << " " << o[1] << endl;
    // }

    vector<vector<double>> trajectory{};
    vector<vector<vector<double>>> pre_trajectories{};
    vector<vector<vector<double>>> trajectories{};


    for (auto first_goal : allowed) {
        // cerr << "First goal = " << endl;
        // print_point(first_goal);
        // cerr << endl;

        allowed_next(first_goal, three_obstacles[1], new_allowed);
        if (new_allowed.size() == 0) {
            trajectory.clear();
            trajectory.push_back(first_goal);
            pre_trajectories.push_back(trajectory);
            continue;
        }
        // for (auto& o : new_allowed){
        //     cerr << "allowed 2: " << o[0] << " " << o[1] << endl;
        // }
        for (auto next : new_allowed) {
            trajectory.clear();
            trajectory.push_back(first_goal);
            trajectory.push_back(next);
            // print_trajectory(trajectory);
            pre_trajectories.push_back(trajectory);
        }
    }
    allowed = new_allowed;

    // cerr << "::second" << endl;
    for (auto prev_traj : pre_trajectories) {
        allowed_next(prev_traj.back(), three_obstacles[2], new_allowed);
        // for (auto& o : allowed){
        //     cerr << "allowed 3: " << o[0] << " " << o[1] << endl;
        // }
        if (new_allowed.size() == 0) {
            trajectory.clear();
            trajectory = prev_traj;
            trajectories.push_back(trajectory);
            continue;
        }
        for (auto next : new_allowed) {
            trajectory.clear();
            trajectory = prev_traj;
            trajectory.push_back(next);
            // print_trajectory(trajectory);
            trajectories.push_back(trajectory);
        }
    }
    // cerr << "::third" << endl;
    trajectory.clear();
    double best_traj_slope = 1e6;
    double slope = 0;
    // cerr << "Car position (" << car.x << ", " << car.y << ")" << endl;
    for (auto& traj : trajectories) {
        // print_trajectory(traj);
        slope = max_traj_slope(traj, car);
        // cerr << "score = " << slope << endl;
        if (slope < best_traj_slope) {
            best_traj_slope = slope;
            trajectory = traj;
        }
    }
    if (trajectory.size() == 0) {
        new_car_y = car.y;
        return;
    }

    new_car_y = trajectory[0][1];

    // cerr << "Best traj" << endl;
    // print_trajectory(trajectory);
    // cerr << "new_car_y = " << new_car_y << endl;

    trajectory.pop_back();
    next_traj_score =  max_traj_slope(trajectory, car);
    // cerr << "new obstacle between " << three_obstacles[0].y - three_obstacles[0].r << " and " << three_obstacles[0].y + three_obstacles[0].r;
    // cerr << "next_traj_score = " << next_traj_score << endl;
    //cerr << endl;
}

double getSteer(const vector<Obstacle>& obstacles, const Car& car) {
    if (car.v == 0) {
        return 0;
    }

    if (car.x < 5) {
        next_obstacle_x = 0;
        next_next_obstacle_x = 0;
        next_traj_score = 0;
        new_car_y = 45;
    }

    // int direction = 0;
    if (car.x + Car::WIDTH / 2 > next_obstacle_x) {
        make_route(obstacles, car);
    }

    // return 10 * (-car.psi - 8 * car.beta) - 0.0023 * (car.y - new_car_y) * abs(car.y - new_car_y);
    // cerr << atan((new_car_y - car.y) / (next_obstacle_x - car.x)) << endl;
    double factor = 1;
    // if (abs(car.y - new_car_y) > 2.5 * Car::WIDTH) {
    //     factor = 2;
    // } 
    // if (abs(car.x - next_obstacle_x) < 2 * Car::LENGTH) {
    //     factor = 0;
    // }
    double regularizer = 5;
    if (abs(new_car_y - car.y) < 10) {
        regularizer = Car::LENGTH / 2;
    }


    // cerr << "new_car_y = " << new_car_y << " next_obstacle_x = " << next_obstacle_x << endl;
    double steer = -10 * car.beta - 4 * (car.psi - atan(factor * (new_car_y - (car.y + Car::LENGTH * sin(car.psi) / 2)) / (regularizer + next_obstacle_x - car.x))) - 0.01 * (car.y - new_car_y);
    if (car.v > 101) {
        steer -= 10 * car.beta;
    }
    // if (car.y + Car::LENGTH * sin(car.psi) / 2 - Car::WIDTH * cos(car.psi)/ 2 < Car::WIDTH || car.y + Car::LENGTH * sin(car.psi) / 2 + Car::WIDTH * cos(car.psi)/ 2 > ROAD_WIDTH - Car::WIDTH) {
    //     steer -= 3 * car.psi;
    // }
    // cerr << "beta = " << car.beta << "steer = " << steer << endl;
    return steer;
    // - 0.01 * (car.y - new_car_y)

    // if (car.x > closest_obstacle_x) {
    //     make_route()
    // }
    


    // cerr << "closest obstacle y = " << closest_obstacle.y << " r = " << closest_obstacle.r << " car.y = " << car.y << endl;
    // if (car.y - Car::WIDTH / 2 > closest_obstacle.y + closest_obstacle.r || car.y + Car::WIDTH / 2 < closest_obstacle.y - closest_obstacle.r) {
    //     // return -car.psi - 0.3 * car.beta * car.v / (Car::WIDTH / 2);
    //     return -car.psi - 5 * car.beta;
    // }



    // int direction = 0;

    // if (upper_score > lower_score) {
    //     new_car_y = upper_gap_center;
    //     direction = -1;
    // } else {
    //     new_car_y = lower_gap_center;
    //     direction = 1;
    // }
    // new_car_middle_y = (car.y + new_car_y ) / 2;
    // car_start_y = car.y;

    // cerr << "Decided to turn, new_car_y = " << new_car_y << endl;
    // turning_mode = direction * int(25 * 100 / (car.v + 1e-5) * abs(new_car_y - car.y) / (2.4 * World::LANE_WIDTH));
    // cerr << "turning_mode = " << turning_mode << endl;
    // abort();
    // cerr << "new y = " << new_car_y << " new_middle" << new_car_middle_y << endl;


    // if (new_car_y < car.y) {
    //     return -0.5;
    // } else {
    //     return 0.5;
    // }
}

void solve() {
    while (true) {
        mtx.lock();
        world.updatePedestrians();

        const Car ego = world.getMyCar();

        // cerr << "frontAngle = " << ego.frontAngle << " beta = " << ego.beta << " psi = " << ego.psi << endl;
        // comment this line to run simulator in manual mode
        double df = getSteer(world.getObstacles(), ego);
        double a = getAcceleration(world.getPedestrians(), world.getCrosswalks(), world.getObstacles(), ego);


        world.makeStep(a, df);

        if (!pressed) {
            a = 0, df = 0;
        }
        pressed = false;

        if (world.checkCollisions()) {
            cerr << "collision in " << world.getTimeSinceStart() << " sec.\n";
            abort();
            world.init();
        }
        if (world.gameOver()) {
            cerr << world.getTimeSinceStart() << " sec.\n";
            break;
        }
        mtx.unlock();

        this_thread::sleep_for(chrono::milliseconds(10));
    }
}


int main(int /*argc*/, char ** /*argv*/) {
    srand(1991);
    // srand(1993);

    cout << "Started solution" << endl;

    v.setSize(W, H);

    v.setOnKeyPress([&](const QKeyEvent& ev) {
        pressed = true;
        if (ev.key() == Qt::Key_W) a = 100;
        if (ev.key() == Qt::Key_S) a = -100;
        if (ev.key() == Qt::Key_A) df = -0.8;
        if (ev.key() == Qt::Key_D) df = 0.8;
    });

    world.init();
    thread solveThread(solve);

    World world_copy;
    while (true) {
        RenderCycle r(v);

        {
            mtx.lock();
            world_copy = world;
            mtx.unlock();
        }
        world_copy.draw();
    }
}
