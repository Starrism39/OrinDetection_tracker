/*************************************************************************************************************************
 * Copyright 2024 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the “Software”), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *************************************************************************************************************************/
#include "utils/timestamp.h"

std::shared_ptr<unsigned char> getTimestamp()
{
    auto epoch = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch();
    long long unix_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();
    unsigned char *timestamp_buf = new unsigned char[8];

    timestamp_buf[0] = static_cast<unsigned char>(unix_timestamp >> 56 & 0xff);
    timestamp_buf[1] = static_cast<unsigned char>(unix_timestamp >> 48 & 0xff);
    timestamp_buf[2] = static_cast<unsigned char>(unix_timestamp >> 40 & 0xff);
    timestamp_buf[3] = static_cast<unsigned char>(unix_timestamp >> 32 & 0xff);
    timestamp_buf[4] = static_cast<unsigned char>(unix_timestamp >> 24 & 0xff);
    timestamp_buf[5] = static_cast<unsigned char>(unix_timestamp >> 16 & 0xff);
    timestamp_buf[6] = static_cast<unsigned char>(unix_timestamp >> 8 & 0xff);
    timestamp_buf[7] = static_cast<unsigned char>(unix_timestamp & 0xff);

    return std::shared_ptr<unsigned char>(timestamp_buf, [](unsigned char *ptr)
                                          { delete[] ptr; });
}

long long Stream2Timestamp(const unsigned char *data)
{
    return (static_cast<long long>(data[0]) << 56) |
           (static_cast<long long>(data[1]) << 48) |
           (static_cast<long long>(data[2]) << 40) |
           (static_cast<long long>(data[3]) << 32) |
           (static_cast<long long>(data[4]) << 24) |
           (static_cast<long long>(data[5]) << 16) |
           (static_cast<long long>(data[6]) << 8) | data[7];
}