// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <fstream>
#include <vector>

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
    double e, Kh, h, Pout, Kp, delta, fuel_mass, lander_mass, scalar_distance_square, Gravitational_force_scalar;

    h = position.abs() - MARS_RADIUS;
    fuel_mass = fuel * FUEL_CAPACITY * FUEL_DENSITY;
    lander_mass = fuel_mass + UNLOADED_LANDER_MASS;
    scalar_distance_square = position.abs2();
    Gravitational_force_scalar = -(GRAVITY * MARS_MASS * lander_mass / scalar_distance_square);

    Kh = 0.0170;
    Kp = 2;

    e = -(0.5 + Kh * h + velocity * position.norm());
    Pout = Kp * e;
    delta = Gravitational_force_scalar / MAX_THRUST;

    if (Pout <= -delta) { 
        throttle = 0;
    } 
    else if (Pout >= (1 - delta)) {
        throttle = 1;
    }
    else {
        throttle = delta + Pout;
    }

    ofstream fout;
    fout.open("C:/Users/scofieldluo/Desktop/trajectories kh0.0170.txt", ios_base::app);
    if (fout){
        //file opened successfully
        fout << h << ' ' << velocity * position.norm() << endl;
    }
    else {
        //file did not open successfully
        cout << "Could not open file for writing" << endl;
    }

}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
    double fuel_mass, lander_mass, scalar_distance_square, lander_area, para_area;
    vector3d position_norm, Gravitational_force, drag_force, thrust, drag_lander, drag_para, force_total, a, new_position;
    static vector3d previous_position;

    fuel_mass = fuel * FUEL_CAPACITY * FUEL_DENSITY;
    lander_mass = fuel_mass + UNLOADED_LANDER_MASS;
    scalar_distance_square = position.abs2();
    position_norm = position.norm();
    Gravitational_force = -(GRAVITY * MARS_MASS * lander_mass / scalar_distance_square) * position_norm;

    thrust = thrust_wrt_world();

    lander_area = M_PI * LANDER_SIZE * LANDER_SIZE;
    para_area = 5 * (2.0 * LANDER_SIZE) * (2.0 * LANDER_SIZE);
    drag_lander = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * lander_area * velocity.abs2() * velocity.norm();
    drag_para = -0.5 * DRAG_COEF_CHUTE * atmospheric_density(position) * para_area * velocity.abs2() * velocity.norm();

    if (parachute_status == DEPLOYED) {
        drag_force = drag_lander + drag_para;
    } else {
        drag_force = drag_lander;
    }

    force_total = thrust + Gravitational_force + drag_force;  
    a = force_total / lander_mass;

    if (simulation_time == 0) {
        velocity += a * delta_t;
        new_position = position + velocity * delta_t;
    }
    else {
        new_position = 2 * position - previous_position + delta_t * delta_t * a;
        velocity = (new_position - position) / delta_t;
    }

    previous_position = position;
    position = new_position;

    /* Euler
    a = force_total / lander_mass;
    velocity += a * delta_t;
    position += velocity * delta_t;
    */




  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
