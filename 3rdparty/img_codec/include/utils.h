/*************************************************************************************************************************
 * Copyright 2024 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#pragma once

#include <cstring>
#include <fstream>

bool wirte_file(const std::string &path, void *data, size_t len)
{
    FILE *file = fopen(path.c_str(), "wb");
    if (file == nullptr)
    {
        fprintf(stderr, "can't open file: %s\n", path.c_str());
        return false;
    }
    fwrite(data, sizeof(unsigned char), len, file);
    fclose(file);
    return true;
}

bool read_file(const std::string &path, void **data, int &len)
{
    FILE *file = fopen(path.c_str(), "rb");
    if (file == nullptr)
    {
        fprintf(stderr, "can't open file: %s\n", path.c_str());
        return false;
    }
    fseek(file, 0, SEEK_END);
    len = ftell(file);
    fseek(file, 0, SEEK_SET);

    *data = realloc(*data, len);
    fread(*data, len, 1, file);
    fclose(file);
    return true;
}

unsigned short fromBigEndian(const unsigned char *data)
{
    return (static_cast<unsigned short>(data[0]) << 8) | data[1];
}

unsigned char cal_sum(const unsigned char *data, size_t len)
{
    unsigned char sum = 0;
    for (int i = 0; i < len; ++i)
    {
        sum = (sum + data[i]) & 0xff;
    }
    return sum;
}
