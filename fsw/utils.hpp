#pragma once

#include "pico/stdlib.h"


namespace utils
{

uint8_t
crc8(const uint8_t* buf, size_t len);

}
