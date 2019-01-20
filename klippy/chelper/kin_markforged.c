 // Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // move_get_coord
#include "pyhelper.h" // errorf

/*

B motor is for Y
A1, A2 is for each X

void MarkForgedSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS] + cartesian_mm[Y_AXIS];
    actuator_mm[BETA_STEPPER] = cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}
void HBotSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS] + cartesian_mm[Y_AXIS];
    actuator_mm[BETA_STEPPER ] = cartesian_mm[X_AXIS] - cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void MarkForgedSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const 
{
    cartesian_mm[X_AXIS] = actuator_mm[ALPHA_STEPPER] - actuator_mm[BETA_STEPPER];
    cartesian_mm[Y_AXIS] = actuator_mm[BETA_STEPPER];
    cartesian_mm[Z_AXIS] = actuator_mm[GAMMA_STEPPER];
}
void HBotSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    cartesian_mm[X_AXIS] = 0.5F * (actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER]);
    cartesian_mm[Y_AXIS] = 0.5F * (actuator_mm[ALPHA_STEPPER] - actuator_mm[BETA_STEPPER]);
    cartesian_mm[Z_AXIS] = actuator_mm[GAMMA_STEPPER];
}
*/


static double
markforged_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x + c.y;
}

static double
markforged_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

struct stepper_kinematics * __visible
markforged_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'a')
        sk->calc_position = markforged_stepper_a_calc_position;
    else if (axis == 'b')
        sk->calc_position = markforged_stepper_b_calc_position;
    return sk;
}
