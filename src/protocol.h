/*
  protocol.h - Stuff related to the client/controller binary protocol.
  Part of flexo-controller

  Copyright (c) 2019 Phil Desrosiers

  Flexo is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Flexo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Flexo.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "flexo.h"

#define PROTOCOL_VERSION_MAJOR 0 // Major version difference indicates incompatible.
#define PROTOCOL_VERSION_MINOR 1 // All minor versions should be backward compatible.

enum msg_type_t
{
    GET_STATUS,
    RESET,
    MOVE,
    SET
};

typedef struct header_t
{
    uint16_t message_length; //length of the entire message, in bytes. (max 65K)
    uint64_t message_id;
    uint8_t protocol_version_major;
    uint8_t protocol_version_minor;
    msg_type_t msg_type_id;
} header_t;

typedef struct response_header_t
{
    struct header_t;
    bool success;
} response_header_t;

typedef struct footer_t //same footer used for both request and response.
{
    uint32_t crc32; //CRC for message body.
} footer_t;

/**
 * Joint move message. Attempting to move a joint outside of its min/max extents will 
 * fail unless override is set to true. When a movement command fails in this way, 
 * no movement of the robot will occur. That is, submitting a move command for multiple 
 * joints will fail transactionally if any one of the subcommands would fail. The robot 
 * should move as commanded, or not move at all.
 **/
typedef struct req_move_t
{
    double theta[MOTOR_COUNT]; //angle in radians.
    movementMode_t mode;       //absolute or relative.
    bool override;             //override min/max extents
} req_move_t;

typedef struct req_set_t
{
    int index; // which joint are we configuring?
    jointConfig_t jointConfig;
    // motorConfig_t motorConfig;
} req_set_t;

typedef struct res_pose_t
{
    system_state_t systemMode;
    move_state_t moveMode;
    double theta[MOTOR_COUNT];
} res_pose_t;

#endif // __PROTOCOL_H__