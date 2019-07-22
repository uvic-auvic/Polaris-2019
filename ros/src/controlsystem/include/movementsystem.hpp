//
// Created by avlec on 22/07/19.
//

#ifndef POLARIS_MOVEMENTSYSTEM_HPP
#define POLARIS_MOVEMENTSYSTEM_HPP

/*
 * This class is responsible for controlling the movement of the AUV.
 *
 */
class MovementSystem {
public:
	// Base function for commanding movement.
	void move(double x, double y, double z);
	// Convenience macros.
	void move_horiz(double vel_x, double vel_y) { move(vel_x, vel_y, 0); }
	void move_forward(double vel)               { move(vel, 0, 0); }
	void move_lateral(double vel)               { move(0, vel, 0); }
	void move_vert(double vel)                  { move(0, 0, vel); }
};

#endif //POLARIS_MOVEMENTSYSTEM_HPP
