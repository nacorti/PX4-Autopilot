/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessCompoundVTOL.hpp
 *
 * Actuator effectiveness for Compound VTOL
 *
 * @author Nick Corti <nacorti@gmail.com>
 */

#include "ActuatorEffectivenessCompoundVTOL.hpp"

ActuatorEffectivenessCompoundVTOL::ActuatorEffectivenessCompoundVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

bool
ActuatorEffectivenessCompoundVTOL::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
	if (!_updated) {
		return false;
	}
	float tilt_main = 0.0f;
	float tilt_pusher = 0.0f;

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			tilt_main = 0.0f;
			tilt_pusher = 0.0f;
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			tilt_main = 0.2f;
			tilt_pusher = 1.5f;
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF: {
			tilt_main = 0.1f;
			tilt_pusher = 0.8f;
			break;
		}
	}

	// Trim: half throttle, tilted motors
	_trim(0) = 0.4f;
	_trim(1) = 0.4f;
	_trim(2) = 0.6f;
	_trim(3) = tilt_main;
	_trim(4) = tilt_main;
	_trim(5) = tilt_pusher;

	const float l_x_front = 8.f;
	const float l_x_rear = 8.f;
	const float l_y_front = 2.0f;
	const float c_t_front = 10.f;
	const float c_t_rear = 20.f;

	// Effectiveness
	const float compound_vtol[NUM_AXES][NUM_ACTUATORS] = {
		{ c_t_front *l_y_front * cosf(_trim(3)), -c_t_front *l_y_front * cosf(_trim(4)),                         0.f,                             0.f,                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ c_t_front *l_x_front * cosf(_trim(3)),  c_t_front *l_x_front * cosf(_trim(4)),  -c_t_rear *l_x_rear * 1.0f,                             0.f,                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{                                   0.f,                                    0.f,                         0.f, c_t_front *l_y_front * _trim(0),-c_t_front *l_y_front * _trim(1), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{                                   0.f,                                    0.f,                         0.f,                             0.f,                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{           -c_t_front * cosf(_trim(3)),            -c_t_front * cosf(_trim(4)),                   -c_t_rear,                             0.f,                             0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	};
	matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(compound_vtol);

	_updated = false;
	return true;
}

void
ActuatorEffectivenessCompoundVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);
	_updated = true;

}
