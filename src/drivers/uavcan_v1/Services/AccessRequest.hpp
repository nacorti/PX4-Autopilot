/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file Access.hpp
 *
 * Defines a Access Service invoker and process Access responses
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include <uavcan/_register/Access_1_0.h>

#include "../Publishers/Publisher.hpp"

class UavcanAccessServiceRequest
{
public:
	UavcanAccessServiceRequest(CanardInstance &ins, UavcanParamManager &pmgr) :
		_canard_instance(ins), _param_manager(pmgr) { };

	bool setPortId(CanardNodeID node_id, uavcan_register_Name_1_0 &name)
	{
		int result {0};

		uavcan_register_Access_Request_1_0 request_msg;
		request_msg.value.natural16.value.count = 1;
		uavcan_register_Value_1_0_select_natural16_(&request_msg.value); // Set to natural16 so that ParamManager casts type

		//FIXME ParamManager only has notion of being either sub/pub have to find a portable way to address trhis
		name.name.elements[7] = 's'; //HACK Change pub into sub

		if (_param_manager.GetParamByName(name, request_msg.value)) {
			name.name.elements[7] = 'p'; //HACK Change sub into pub
			memcpy(&request_msg.name, &name, sizeof(request_msg.name));

			uint8_t request_payload_buffer[uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindRequest,
				.port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_,                // This is the subject-ID.
				.remote_node_id = node_id,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = access_request_transfer_id,
				.payload_size   = uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &request_payload_buffer,
			};

			result = uavcan_register_Access_Request_1_0_serialize_(&request_msg, request_payload_buffer, &transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++access_request_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}

			return result > 0;

		} else {
			return false;
		}
	};

private:
	CanardInstance &_canard_instance;
	CanardTransferID access_request_transfer_id = 0;
	UavcanParamManager &_param_manager;

};
