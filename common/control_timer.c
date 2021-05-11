/**
 * @file control_timer.h
 * @brief timer library for control cycle
 * @author RT Corporation
 * @date 2019-2021
 * @copyright License: Apache License, Version 2.0
 */
// Copyright 2019 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//// ヘッダーファイル
#include <time.h>


//時間管理用変数
static struct timespec start_time = {0}, end_time = {0}, sleep_time = {0}, duration_time = {0};
static long loop_time_nsec = 0;

void initControlTimer(int control_cycle)
{
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    loop_time_nsec = (long)((1.0 / control_cycle)*1e9);
}

void sleepForControlCycle(void)
{
    //周期がloop_time_nsec [ns]となるようにnanosleepで調整する処理
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    if (end_time.tv_nsec < start_time.tv_nsec)
    {
        duration_time.tv_nsec = end_time.tv_nsec + 1000000000 - start_time.tv_nsec;
    }
    else
    {
        duration_time.tv_nsec = end_time.tv_nsec - start_time.tv_nsec;
    }
    //loop_time_nsec より短かったらnanosleepで時間調整
    if (duration_time.tv_nsec < loop_time_nsec)
    {
        sleep_time.tv_nsec = loop_time_nsec - duration_time.tv_nsec;
        //printf("sleep time %ld.%09ld\n", sleep_time.tv_sec, sleep_time.tv_nsec);
        nanosleep(&sleep_time, NULL);
    }
        start_time.tv_nsec = start_time.tv_nsec + loop_time_nsec;
    if (start_time.tv_nsec > 1000000000)
    {
        start_time.tv_nsec = start_time.tv_nsec - 1000000000;
    }
}