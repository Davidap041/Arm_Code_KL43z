/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _Kalman_h_
#define _Kalman_h_
typedef struct {
	float Q_angle; // Process noise variance for the accelerometer
	float Q_bias; // Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
} Kalman_data;
float getAngle(Kalman_data *kalman, float newAngle, float newRate, float dt);

void setAngle(Kalman_data *kalman, float angle); // Used to set angle, this should be set as the starting angle
float getRate(Kalman_data *kalman); // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(Kalman_data *kalman,float Q_angle);
/**
 * setQbias(float Q_bias)
 * Default value (0.003f) is in Kalman.cpp.
 * Raise this to follow input more closely,
 * lower this to smooth result of kalman filter.
 */
void setQbias(Kalman_data *kalman,float Q_bias);
void setRmeasure(Kalman_data *kalman,float R_measure);

float getQangle(Kalman_data *kalman);
float getQbias(Kalman_data *kalman);
float getRmeasure(Kalman_data *kalman);

/* Kalman filter variables */

#endif
