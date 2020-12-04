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
 * @file ActuatorEffectivenessStandardVTOL.hpp
 *
 * Actuator effectiveness for standard VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessStandardVTOL.hpp"

ActuatorEffectivenessStandardVTOL::ActuatorEffectivenessStandardVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

bool
ActuatorEffectivenessStandardVTOL::update()
{
	if (_updated) {
		_updated = false;
		return true;
	}

	return false;
}

void
ActuatorEffectivenessStandardVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
<<<<<<< HEAD
				{ -0.5f,   0.5f,   0.5f,  -0.5f, 0.f, 0.0f, 0.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.5f,  -0.5f,   0.5f,  -0.5f, 0.f,  0.f,  0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.25f,  0.25f, -0.25f, -0.25f, 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{   0.f,    0.f,    0.f,    0.f, 1.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{   0.f,    0.f,    0.f,    0.f, 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{-0.25f, -0.25f, -0.25f, -0.25f, 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
=======
				{-0.5f,  0.5f,  0.5f, -0.5f, 0.f, 0.0f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.5f, -0.5f,  0.5f, -0.5f, 0.f, 0.f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.25f,  0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{-0.25f, -0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
>>>>>>> 66b066d57d8680e8e530c3d4b610c6f2326174c9
			};
			_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{ 0.f, 0.f, 0.f, 0.f, 0.f, -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}

	case FlightPhase::TRANSITION_HF_TO_FF:
	case FlightPhase::TRANSITION_FF_TO_HF: {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{ -0.5f,  0.5f,  0.5f, -0.5f, 0.f, -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.5f, -0.5f,  0.5f, -0.5f, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.25f,  0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f,  0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{-0.25f, -0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			_effectiveness = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}
	}
}
