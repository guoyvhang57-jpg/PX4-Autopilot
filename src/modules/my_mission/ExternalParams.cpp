/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 ****************************************************************************/

/**
 * @file ExternalParams.cpp
 */

#include "ExternalParams.hpp"

void ExternalParams::refresh()
{
	for (auto &mirror : _mirrors) {
		if (mirror.handle == PARAM_INVALID) {
			mirror.handle = param_find(mirror.name);
		}

		float value = mirror.value;

		if ((mirror.handle != PARAM_INVALID) && (param_get(mirror.handle, &value) == PX4_OK)) {
			mirror.value = value;
		}
	}
}
