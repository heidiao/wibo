#ifndef	__STEPPER_MOTOR_H
#define	__STEPPER_MOTOR_H

#include	"def.h"

#define	FREQ								1000000	// 1Mhz

#define	DEV_ADDR							50

#define	SENDER_UART							0x01
#define SENDER_USB							0x02
#define	SENDER_SPI							0x03

#define	CMD_PING							0x01
#define CMD_READ_PRES_STRING				0x02
#define	CMD_NACK							0x03
#define CMD_CHECK_MODE_SUPPORT				0x04
#define CMD_START_DATA_STREAMING			0x08
#define CMD_STOP_DATA_STREAMING				0x09

#define	CMD_MOTOR_DRIVE						0x10
#define	CMD_MOTOR_STOP						0x11
#define	CMD_MOTOR_GET_STATE					0x12
#define	CMD_MOTOR_REPORT_SENSOR				0x18
#define	CMD_MOTOR_TEST						0x1f

#define	CMD_REPLY_ADD						0x80

#define	PRESENTATION_STRING					"MEMS shield demo"

typedef enum {
	ID_HORI,
	ID_VERT,
	ID_MAX
} ident_t;

typedef enum {
	CCW = -1,	// counter clockwise
	NA = 0,		// not avaliable (last state)
	CW = 1		// clockwise
} dir_t;

#pragma pack(push)
#pragma pack(1)
#define	DEFINE_CMD_START	typedef struct __attribute__((__packed__)) {
#define	DEFINE_CMD_END(x)	} x
							
DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
DEFINE_CMD_END(cmd_short_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	uint32_t	sensors_enabled;
	uint32_t	data_tx_period;	
DEFINE_CMD_END(cmd_start_data_streaming_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	ident_t		id:8;
	dir_t		direction:8;
	uint16_t	steps;
	uint16_t	period_start;		// in us
	uint16_t	period_end;			// in us
	uint16_t	period_accel;		// in us
DEFINE_CMD_END(cmd_motor_drive_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	ident_t		id:8;
	uint8_t		pls;
	uint8_t		dir;
	uint8_t		awo;
	uint8_t		cs;
DEFINE_CMD_END(cmd_motor_test_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	ident_t		id:8;
	uint16_t	period_stop;
	uint16_t	period_accel;
DEFINE_CMD_END(cmd_motor_stop_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	ident_t		id:8;
DEFINE_CMD_END(cmd_motor_get_state_t);

#pragma pack(pop)

uint32_t cmd_motor_drive_time( cmd_motor_drive_t *cmd );

typedef struct {
	uint8_t		id;
	uint32_t	freq;
	double		spd;		// steps per degree
} motor_t;

extern motor_t pseudo_hori, pseudo_vert;
extern motor_t wibo_hori, wibo_vert;

uint16_t motor_deg2step( motor_t *motor, int degree );
double motor_step2deg( motor_t *motor, uint16_t step );
uint16_t motor_rpm2period( motor_t *motor, double rpm );
cmd_motor_drive_t* motor_gen_cmd_motor_drive( motor_t *motor, cmd_motor_drive_t *cmd, dir_t direction, uint16_t steps, uint16_t period_start, uint16_t period_end, uint16_t period_accel );

// angle in degree
// accel_time in milli-second
cmd_motor_drive_t* motor_gen_cmd_motor_drive_rpm( motor_t *motor, cmd_motor_drive_t *cmd, dir_t direction, int degree, double rpm_start, double rpm_end, int accel_time );

#endif
