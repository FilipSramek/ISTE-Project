#pragma once

#include "utils/stdlib/stdint.h"

namespace math {

/**
 * @brief Calendar date-time utility (UTC-like, no timezone handling).
 *
 * Supported range is limited to [1970-01-01 00:00:00, 2099-12-31 23:59:59].
 * This keeps arithmetic deterministic and bounded for embedded use.
 */
class DateTime {
public:
    int32_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // ========== Constructors ==========

    /**
     * @brief Default constructor - Unix epoch start.
     */
    constexpr DateTime()
        : year(1970), month(1U), day(1U), hour(0U), minute(0U), second(0U) {}

    /**
     * @brief Constructor from date-time components.
     */
    constexpr DateTime(int32_t year_,
                       uint8_t month_,
                       uint8_t day_,
                       uint8_t hour_,
                       uint8_t minute_,
                       uint8_t second_)
        : year(year_),
          month(month_),
          day(day_),
          hour(hour_),
          minute(minute_),
          second(second_) {}

    // ========== Validation ==========

    /**
     * @brief Check whether year is leap year in Gregorian calendar.
     */
    static bool is_leap_year(int32_t y) {
        if ((y % 400) == 0) {
            return true;
        }

        if ((y % 100) == 0) {
            return false;
        }

        return ((y % 4) == 0);
    }

    /**
     * @brief Get number of days in given month.
     * @param y Year
     * @param m Month [1..12]
     * @param days_out Output number of days in the month
     * @return true on success
     */
    static bool days_in_month(int32_t y, uint8_t m, uint8_t& days_out) {
        if ((m < 1U) || (m > 12U)) {
            return false;
        }

        static constexpr uint8_t DAYS_TABLE[12] = {
            31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
        };

        uint8_t days = DAYS_TABLE[static_cast<uint8_t>(m - 1U)];
        if ((m == 2U) && is_leap_year(y)) {
            days = 29U;
        }

        days_out = days;
        return true;
    }

    /**
     * @brief Check if this date-time is inside supported range and valid.
     */
    bool is_valid() const {
        if ((year < MIN_YEAR) || (year > MAX_YEAR)) {
            return false;
        }

        if ((month < 1U) || (month > 12U)) {
            return false;
        }

        uint8_t month_days = 0U;
        if (!days_in_month(year, month, month_days)) {
            return false;
        }

        if ((day < 1U) || (day > month_days)) {
            return false;
        }

        if (hour > 23U) {
            return false;
        }

        if (minute > 59U) {
            return false;
        }

        if (second > 59U) {
            return false;
        }

        return true;
    }

    // ========== Conversion ==========

    /**
     * @brief Convert date-time to unix seconds from 1970-01-01.
     * @param unix_seconds Output seconds from epoch
     * @return true on success
     */
    bool to_unix_seconds(int64_t& unix_seconds) const {
        if (!is_valid()) {
            return false;
        }

        static constexpr uint16_t DAYS_BEFORE_MONTH[12] = {
            0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U
        };

        const int32_t years_since_epoch = year - MIN_YEAR;
        const int32_t leap_before_this_year = leap_count_until_(year - 1) - leap_count_until_(MIN_YEAR - 1);
        const int32_t days_before_year = (years_since_epoch * 365) + leap_before_this_year;

        int32_t days_before_month = static_cast<int32_t>(DAYS_BEFORE_MONTH[static_cast<uint8_t>(month - 1U)]);
        if ((month > 2U) && is_leap_year(year)) {
            days_before_month += 1;
        }

        const int32_t days_this_month = static_cast<int32_t>(day - 1U);
        const int32_t total_days = days_before_year + days_before_month + days_this_month;

        const int32_t seconds_of_day =
            (static_cast<int32_t>(hour) * 3600) +
            (static_cast<int32_t>(minute) * 60) +
            static_cast<int32_t>(second);

        unix_seconds = (static_cast<int64_t>(total_days) * SECONDS_PER_DAY) +
                       static_cast<int64_t>(seconds_of_day);
        return true;
    }

    /**
     * @brief Build date-time from unix seconds.
     * @param unix_seconds Seconds from epoch
     * @param result Output date-time
     * @return true on success
     */
    static bool from_unix_seconds(int64_t unix_seconds, DateTime& result) {
        if ((unix_seconds < 0) || (unix_seconds > max_unix_seconds_())) {
            return false;
        }

        int64_t remaining_days = unix_seconds / SECONDS_PER_DAY;
        int64_t seconds_of_day = unix_seconds % SECONDS_PER_DAY;

        int32_t y = MIN_YEAR;
        for (uint32_t i = 0U; i < MAX_YEAR_SCAN; ++i) {
            const int32_t year_days = is_leap_year(y) ? 366 : 365;
            if (remaining_days < static_cast<int64_t>(year_days)) {
                break;
            }

            remaining_days -= static_cast<int64_t>(year_days);
            y += 1;
        }

        if ((y < MIN_YEAR) || (y > MAX_YEAR)) {
            return false;
        }

        uint8_t m = 1U;
        for (uint8_t i = 0U; i < 12U; ++i) {
            uint8_t month_days = 0U;
            if (!days_in_month(y, m, month_days)) {
                return false;
            }

            if (remaining_days < static_cast<int64_t>(month_days)) {
                break;
            }

            remaining_days -= static_cast<int64_t>(month_days);
            m = static_cast<uint8_t>(m + 1U);
        }

        if ((m < 1U) || (m > 12U)) {
            return false;
        }

        const uint8_t d = static_cast<uint8_t>(remaining_days + 1);
        const uint8_t h = static_cast<uint8_t>(seconds_of_day / 3600);
        seconds_of_day %= 3600;
        const uint8_t min = static_cast<uint8_t>(seconds_of_day / 60);
        const uint8_t sec = static_cast<uint8_t>(seconds_of_day % 60);

        result.year = y;
        result.month = m;
        result.day = d;
        result.hour = h;
        result.minute = min;
        result.second = sec;

        return result.is_valid();
    }

    /**
     * @brief Add signed seconds and return normalized date-time.
     * @param delta_seconds Signed seconds to add
     * @param result Output date-time
     * @return true on success
     */
    bool add_seconds(int32_t delta_seconds, DateTime& result) const {
        int64_t base_seconds = 0;
        if (!to_unix_seconds(base_seconds)) {
            return false;
        }

        const int64_t sum = base_seconds + static_cast<int64_t>(delta_seconds);
        return from_unix_seconds(sum, result);
    }

    /**
     * @brief Compute signed difference in seconds (other - this).
     */
    bool difference_seconds(const DateTime& other, int64_t& diff_seconds) const {
        int64_t a = 0;
        int64_t b = 0;

        if (!to_unix_seconds(a)) {
            return false;
        }

        if (!other.to_unix_seconds(b)) {
            return false;
        }

        diff_seconds = b - a;
        return true;
    }

    // ========== Comparison Operators ==========

    bool operator==(const DateTime& rhs) const {
        return (year == rhs.year) &&
               (month == rhs.month) &&
               (day == rhs.day) &&
               (hour == rhs.hour) &&
               (minute == rhs.minute) &&
               (second == rhs.second);
    }

    bool operator!=(const DateTime& rhs) const {
        return !(*this == rhs);
    }

    bool operator<(const DateTime& rhs) const {
        if (year != rhs.year) { return year < rhs.year; }
        if (month != rhs.month) { return month < rhs.month; }
        if (day != rhs.day) { return day < rhs.day; }
        if (hour != rhs.hour) { return hour < rhs.hour; }
        if (minute != rhs.minute) { return minute < rhs.minute; }
        return second < rhs.second;
    }

    bool operator>(const DateTime& rhs) const {
        return rhs < *this;
    }

    bool operator<=(const DateTime& rhs) const {
        return !(*this > rhs);
    }

    bool operator>=(const DateTime& rhs) const {
        return !(*this < rhs);
    }

private:
    static constexpr int32_t MIN_YEAR = 1970;
    static constexpr int32_t MAX_YEAR = 2099;
    static constexpr int64_t SECONDS_PER_DAY = 86400LL;
    static constexpr uint32_t MAX_YEAR_SCAN = 130U;

    static int32_t leap_count_until_(int32_t y) {
        return (y / 4) - (y / 100) + (y / 400);
    }

    static int64_t max_unix_seconds_() {
        DateTime max_dt(MAX_YEAR, 12U, 31U, 23U, 59U, 59U);
        int64_t value = 0;
        const bool ok = max_dt.to_unix_seconds(value);
        if (!ok) {
            return 0;
        }

        return value;
    }
};

} // namespace math
