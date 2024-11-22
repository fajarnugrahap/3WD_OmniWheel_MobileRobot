/*
 * user.h
 *
 *  Created on: Jul 12, 2024
 *      Author: USER
 */

#ifndef INC_USER_H_
#define INC_USER_H_

void remote_mode1(void);

void set_speeds(float x_speed,float y_speed,float w_speed);

void set_target (int max_speed, int x_target, int y_target, int theta_target);

void Mleft(int rpm1,int dir1);

void Mback(int rpm3,int dir3);

void Mright(int rpm2,int dir2);

#endif /* INC_USER_H_ */
