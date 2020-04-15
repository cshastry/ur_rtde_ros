/*
 * Copyright (c) 2020 Kim Lindberg Schwaner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*!
 * \file
 */

#include "monotonic.h"

static constexpr timespec make_timespec(const monotonic_clock::duration& dur)
{
    auto s = std::chrono::duration_cast<std::chrono::seconds>(dur);
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(dur - s);
    return timespec{
        static_cast<std::time_t>(s.count()),
        static_cast<long>(ns.count())
    };
}

monotonic_clock::time_point monotonic_clock::now() noexcept
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return time_point(duration(std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec)));
}

monotonic_rate::monotonic_rate(double frequency)
    : period_(std::chrono::duration_cast<monotonic_clock::duration>(std::chrono::duration<double>(1.0 / frequency)))
{
    reset();
}

void monotonic_rate::reset()
{
    actual_cycle_time_ = monotonic_clock::duration::zero();
    t_wakeup_ = monotonic_clock::now();
    t_end_expected_ = t_wakeup_ + period_;
}

bool monotonic_rate::sleep()
{
    auto t_now = monotonic_clock::now();

    // Don't sleep if we've taken too much time; instead reset
    if (t_now >= t_end_expected_) {
        actual_cycle_time_ = t_now - t_wakeup_;
        t_wakeup_ = t_now;
        t_end_expected_ = t_now + period_;
        return false;
    }

    // Sleep
    auto ts = make_timespec(t_end_expected_.time_since_epoch());
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);

    t_now = monotonic_clock::now();

    // Actual duration since last wake-up
    actual_cycle_time_ = t_now - t_wakeup_;

    // Actual time at which this wake-up occured
    t_wakeup_ = t_now;

    // Set next expected wake-up time
    t_end_expected_ += period_;

    return true;
}

monotonic_clock::duration monotonic_rate::actual_cycle_time() const
{
    return actual_cycle_time_;
}

monotonic_clock::duration monotonic_rate::expected_cycle_time() const
{
    return period_;
}

monotonic_clock::duration monotonic_rate::remaining_cycle_time() const
{
    auto t_now = monotonic_clock::now();

    if (t_now >= t_end_expected_) {
        return monotonic_clock::duration::zero();
    } else {
        return t_end_expected_ - t_now;
    }
}

monotonic_clock::time_point monotonic_rate::wakeup_stamp() const
{
    return t_wakeup_;
}
