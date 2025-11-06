#pragma once

#include <cstdint>
#include <map>
#include <cmath>

namespace IGVC
{
    /**
     * @enum LimiterKey
     */
    enum LimiterKey : uint8_t
    {
        MotorInput,
        MotorFeedback,
        GPSData,
    };

    /**
     * @brief Class representing a limiter for broadcast values.
     */
    class Limiter
    {
    public:
        ~Limiter() = default;

        /**
         * @brief Set the limit for a specific key.
         * @param key The limiter key.
         * @param limit The limit value (e.g. how many times per second can it be used)
         */
        Limiter &setLimit(LimiterKey key, double limit)
        {
            mLimits[key] = limit;
            return *this;
        }

        /**
         * @brief Check if the current value for a key will exceed its limit if increased.
         * @param key The limiter key.
         * @return True if the limit will be exceeded, false otherwise.
         */
        bool willExceed(LimiterKey key) const
        {
            auto limitIt = mLimits.find(key);
            if (limitIt != mLimits.end())
            {
                auto currentIt = mCurrentValues.find(key);
                int32_t currentValue = (currentIt != mCurrentValues.end()) ? currentIt->second : 0;
                return (currentValue + 1) > std::ceil(limitIt->second);
            }

            return false;
        }

        /**
         * @brief Increment the current value for a specific key.
         * @param key The limiter key.
         */
        void useKey(LimiterKey key)
        {
            mCurrentValues[key]++;
        }

        /**
         * @brief Reset the current values for all keys. Should be called once per second.
         */
        void tick()
        {
            for (auto &pair : mLimits)
            {
                mCurrentValues[pair.first] = 0;
            }
        }
    
    private:
        // Map to hold the limits for different keys
        std::map<LimiterKey, double> mLimits;

        // Map to hold the current values for different keys
        std::map<LimiterKey, int32_t> mCurrentValues;
    };
}