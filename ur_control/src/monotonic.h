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

#pragma once

#include <chrono>

#if !defined(_GLIBCXX_USE_CLOCK_MONOTONIC)
#error Need clock_gettime monotonic clock support
#endif

/*!
 * \brief A monotonic clock.
 *
 * Time returned has the property of only increasing at a uniform rate.
 *
 * This monotonic clock is likely to be the same as \c std::chrono::steady_clock
 * on POSIX systems, but we explicitly define it here to be sure to be able to
 * use \c clock_nanosleep() with the clock.
 */
struct monotonic_clock
{
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<monotonic_clock, duration>;

    static constexpr bool is_steady = true;

    static time_point now() noexcept;
};

/*!
 * \brief Steady loop rate.
 *
 * Implements a steady loop rate using monotonic clock and absolute sleep intervals.
 *
 * Works similarly to ROS' Rate.
 */
class monotonic_rate
{
public:
    explicit monotonic_rate(double frequency);

    template <typename Rep, typename Period>
    explicit monotonic_rate(const std::chrono::duration<Rep, Period>& period)
        : period_(std::chrono::duration_cast<monotonic_clock::duration>(period))
    {
        reset();
    }

    void reset();

    /**
     * \brief Sleep for the time remaining in a cycle.
     *
     * \return True if the desired loop rate was met, false otherwise.
     */
    bool sleep();

    /*!
     * \brief Actual duration between the time of the most current wake-up and
     *        the time of the wake-up before that.
     *
     * \return The duration between wake-ups.
     */
    monotonic_clock::duration actual_cycle_time() const;

    monotonic_clock::duration expected_cycle_time() const;
    monotonic_clock::duration remaining_cycle_time() const;
    monotonic_clock::time_point wakeup_stamp() const;

private:
    monotonic_clock::duration period_;
    monotonic_clock::duration actual_cycle_time_;
    monotonic_clock::time_point t_wakeup_;
    monotonic_clock::time_point t_end_expected_;
};
