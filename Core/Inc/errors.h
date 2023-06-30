/*
 * errors.h
 *
 *  Created on: 18 окт. 2022 г.
 *      Author: serad
 */

#ifndef INC_ERRORS_H_
#define INC_ERRORS_H_

typedef enum
{

	OK,
	ID_Err,
	I2C_Mem_Read_Err,
	I2C_Mem_Write_Err,

} ErrorStatusTypeDef;

#endif /* INC_ERRORS_H_ */
