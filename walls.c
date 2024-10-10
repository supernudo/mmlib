
#include "walls.h"


/** Calibration constants for sensors */
#define SENSOR_SIDE_LEFT_A 2.806
#define SENSOR_SIDE_LEFT_B 0.287
#define SENSOR_SIDE_RIGHT_A 2.327
#define SENSOR_SIDE_RIGHT_B 0.231


#define SENSOR_FRONT_LEFT_A 1500.462
#define SENSOR_FRONT_LEFT_B 138.777
#define SENSOR_FRONT_RIGHT_A 1378.603
#define SENSOR_FRONT_RIGHT_B 124.503

static volatile float distance[SENSORS_WALL_MAX];
static volatile float calibration_factor[SENSORS_WALL_MAX];

/* XXX: Keep sync with sensor.h */
const float sensors_calibration_a[SENSORS_WALL_MAX] = {
	SENSOR_FRONT_LEFT_A, SENSOR_FRONT_RIGHT_A,
    SENSOR_SIDE_LEFT_A, SENSOR_SIDE_RIGHT_A};
const float sensors_calibration_b[SENSORS_WALL_MAX] = {
	SENSOR_FRONT_LEFT_B, SENSOR_FRONT_RIGHT_B,
    SENSOR_SIDE_LEFT_B, SENSOR_SIDE_RIGHT_B};

/* Distance thresholds */
#define SIDE_WALL_DETECTION (CELL_DIMENSION * 0.90)
#define FRONT_WALL_DETECTION (CELL_DIMENSION * 1.5)
#define SIDE_CALIBRATION_READINGS 20
#define DIAGONAL_MIN_DISTANCE 0.24

/**
 * @brief Calculate and update the distance from each sensor.
 *
 * @note The distances are calculated from the center of the robot.
 */
void update_distance_readings(void)
{
	uint8_t i = 0;
	uint16_t on[SENSORS_WALL_MAX], off[SENSORS_WALL_MAX];

	for (i = 0; i < SENSORS_WALL_MAX; i++) {
		on[i] = sensor_adc_get_value_on(i);
		off[i] = sensor_adc_get_value_off(i);
		distance[i] =
		    (sensors_calibration_a[i] / sensor_raw_log(on[i], off[i]) -
		     sensors_calibration_b[i]);
		if ((i == SENSOR_SIDE_LEFT) || (i == SENSOR_SIDE_RIGHT))
			distance[i] -= calibration_factor[i];
	}
}

/**
 * @brief Get distance value from front left sensor.
 */
float get_front_left_distance(void)
{
	return distance[SENSOR_FRONT_LEFT];
}

/**
 * @brief Get distance value from front right sensor.
 */
float get_front_right_distance(void)
{
	return distance[SENSOR_FRONT_RIGHT];
}

/**
 * @brief Get distance value from side left sensor.
 */
float get_side_left_distance(void)
{
	return distance[SENSOR_SIDE_LEFT];
}

/**
 * @brief Get distance value from side right sensor.
 */
float get_side_right_distance(void)
{
	return distance[SENSOR_SIDE_RIGHT];
}

/**
 * @brief Calculate and return the side sensors error when object is too close.
 *
 * Taking into account that the walls are parallel to the robot, this function
 * returns the distance that the robot is moved from the center of the
 * corridor.
 */
float get_side_sensors_close_error(void)
{
	float left_error;
	float right_error;

	left_error = distance[SENSOR_SIDE_LEFT] - MIDDLE_MAZE_DISTANCE;
	right_error = distance[SENSOR_SIDE_RIGHT] - MIDDLE_MAZE_DISTANCE;

	if ((left_error > 0.) && (right_error < 0.))
		return right_error;
	if ((right_error > 0.) && (left_error < 0.))
		return -left_error;
	return 0.;
}

/**
 * @brief Calculate and return the side sensors error when object is too far.
 *
 * This is useful when the robot is too far away from a lateral wall on one
 * side but there is no wall on the other side.
 */
float get_side_sensors_far_error(void)
{
	float left_error;
	float right_error;

	left_error = distance[SENSOR_SIDE_LEFT] - MIDDLE_MAZE_DISTANCE;
	right_error = distance[SENSOR_SIDE_RIGHT] - MIDDLE_MAZE_DISTANCE;

	if ((left_error > 0.1) && (right_error < 0.04))
		return right_error;
	if ((right_error > 0.1) && (left_error < 0.04))
		return -left_error;
	return 0.;
}

/**
 * @brief Calculate and return the front sensors error.
 *
 * Taking into account that robot is approaching to a perpendicular wall, this
 * function returns the difference between the front sensors distances.
 *
 * If there is no front wall detected, it returns 0.
 */
float get_front_sensors_error(void)
{
	if (!front_wall_detection())
		return 0.;
	return distance[SENSOR_FRONT_LEFT] - distance[SENSOR_FRONT_RIGHT];
}

/**
 * @brief Calculate and return the diagonal sensors error.
 *
 * This function returns an error if the robot is too close to a pillar. The
 * error is calculated as the difference between the sensed distance and a
 * minimum distance threshold.
 */
float get_diagonal_sensors_error(void)
{
	float left_error;
	float right_error;

	left_error = distance[SENSOR_FRONT_LEFT] - DIAGONAL_MIN_DISTANCE;
	right_error = distance[SENSOR_FRONT_RIGHT] - DIAGONAL_MIN_DISTANCE;

	if (right_error < 0.)
		return right_error;
	if (left_error < 0.)
		return -left_error;
	return 0.;
}

/**
 * @brief Return the front wall distance, in meters.
 */
float get_front_wall_distance(void)
{
	return (distance[SENSOR_FRONT_LEFT] +
		distance[SENSOR_FRONT_RIGHT]) /
	       2.;
}

/**
 * @brief Detect the existance or absence of the left wall.
 */
bool left_wall_detection(void)
{
	return (distance[SENSOR_SIDE_LEFT] < SIDE_WALL_DETECTION) ? true
								     : false;
}

/**
 * @brief Detect the existance or absence of the right wall.
 */
bool right_wall_detection(void)
{
	return (distance[SENSOR_SIDE_RIGHT] < SIDE_WALL_DETECTION) ? true
								      : false;
}

/**
 * @brief Detect the existance or absence of the front wall.
 */
bool front_wall_detection(void)
{
	return ((distance[SENSOR_FRONT_LEFT] < FRONT_WALL_DETECTION) &&
		(distance[SENSOR_FRONT_RIGHT] < FRONT_WALL_DETECTION))
		   ? true
		   : false;
}

/**
 * @brief Return left, front and right walls detection readings.
 */
struct walls_around read_walls(void)
{
	struct walls_around walls_readings;

	walls_readings.left = left_wall_detection();
	walls_readings.front = front_wall_detection();
	walls_readings.right = right_wall_detection();
	return walls_readings;
}

/**
 * @brief Calibration for side sensors.
 */
void side_sensors_calibration(void)
{
	float right_temp = 0;
	float left_temp = 0;
	int i;

	for (i = 0; i < SIDE_CALIBRATION_READINGS; i++) {
		left_temp += distance[SENSOR_SIDE_LEFT];
		right_temp += distance[SENSOR_SIDE_RIGHT];
		time_wait_ms(SENSORS_PERIOD_us);
	}
	calibration_factor[SENSOR_SIDE_LEFT] +=
	    (left_temp / SIDE_CALIBRATION_READINGS) - MIDDLE_MAZE_DISTANCE;
	calibration_factor[SENSOR_SIDE_RIGHT] +=
	    (right_temp / SIDE_CALIBRATION_READINGS) - MIDDLE_MAZE_DISTANCE;
}
