#pragma once

#include <cstdint>
#include <type_traits>
#include "igvc/json.hpp"

namespace IGVC
{
    namespace Units
    {
        class Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in meters
             */
            explicit Unit(double value) : mValue(value) {}

            /**
             * Get value in meters
             * @return Value in meters
             */
            double toMeters() const
            {
                return mValue;
            }

            double toCentimeters() const
            {
                return mValue * 100.0;
            }

            double toMillimeters() const
            {
                return mValue * 1000.0;
            }

            double toFeet() const
            {
                return mValue / 0.3048;
            }

            double toYards() const
            {
                return mValue / 0.9144;
            }

            double toInches() const
            {
                return mValue / 0.0254;
            }

            double toMiles() const
            {
                return mValue / 1609.34;
            }

        private:
            /**
             * Value in  meters
             */
            double mValue;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(Unit, mValue)
        };

        class Meters : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in meters
             */
            explicit Meters(double value) : Unit(value) {}

            static Meters from(double value)
            {
                return Meters(value);
            }
        };

        class Centimeters : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in centimeters
             */
            explicit Centimeters(double value) : Unit(value / 100.0) {}

            static Centimeters from(double value)
            {
                return Centimeters(value);
            }
        };

        class Millimeters : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in millimeters
             */
            explicit Millimeters(double value) : Unit(value / 1000.0) {}

            static Millimeters from(double value)
            {
                return Millimeters(value);
            }
        };

        class Feet : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in feet
             */
            explicit Feet(double value) : Unit(value * 0.3048) {}

            static Feet from(double value)
            {
                return Feet(value);
            }
        };

        class Yards : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in yards
             */
            explicit Yards(double value) : Unit(value * 0.9144) {}

            static Yards from(double value)
            {
                return Yards(value);
            }
        };

        class Inches : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in inches
             */
            explicit Inches(double value) : Unit(value * 0.0254) {}

            static Inches from(double value)
            {
                return Inches(value);
            }
        };

        class Miles : public Unit
        {
        public:
            /**
             * Constructor
             * @param value Value in miles
             */
            explicit Miles(double value) : Unit(value * 1609.34) {}

            static Miles from(double value)
            {
                return Miles(value);
            }
        };

        // Time Enum
        enum class TimeUnit : uint8_t
        {
            Milliseconds,
            Seconds,
            Minutes,
            Hours
        };

        // Time
        class Time
        {
        public:
            /**
             * Constructor
             * @param value Value in milliseconds
             * @param timeUnit Time unit
             */
            explicit Time(double value, TimeUnit timeUnit = TimeUnit::Milliseconds) : mValue(value), mTimeUnit(timeUnit) {}

            /**
             * Get value in milliseconds
             * @return Value in milliseconds
             */
            double toMilliseconds() const
            {
                return mValue;
            }

            /**
             * Get value in seconds
             * @return Value in seconds
             */
            double toSeconds() const
            {
                return mValue / 1000.0;
            }

            /**
             * Get value in minutes
             * @return Value in minutes
             */
            double toMinutes() const
            {
                return mValue / 60000.0;
            }

            /**
             * Get value in hours
             * @return Value in hours
             */
            double toHours() const
            {
                return mValue / 3600000.0;
            }

        private:
            /**
             * Value in milliseconds
             */
            double mValue;

            /**
             * Time unit
             */
            TimeUnit mTimeUnit;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(Time, mValue, mTimeUnit)
        };

        class Seconds : public Time
        {
        public:
            /**
             * Constructor
             * @param value Value in seconds
             */
            explicit Seconds(double value) : Time(value * 1000.0, TimeUnit::Seconds) {}

            static Seconds from(double value)
            {
                return Seconds(value);
            }
        };

        class Minutes : public Time
        {
        public:
            /**
             * Constructor
             * @param value Value in minutes
             */
            explicit Minutes(double value) : Time(value * 60000.0, TimeUnit::Minutes) {}

            static Minutes from(double value)
            {
                return Minutes(value);
            }
        };

        class Hours : public Time
        {
        public:
            /**
             * Constructor
             * @param value Value in hours
             */
            explicit Hours(double value) : Time(value * 3600000.0, TimeUnit::Hours) {}
            static Hours from(double value)
            {
                return Hours(value);
            }
        };

        // TimeRange
        class TimeRange
        {
        public:
            /**
             * Constructor
             * @param start Start time
             * @param end End time
             */
            TimeRange(const Time &start, const Time &end) : mStart(start), mEnd(end) {}

            /**
             * Get the duration of the time range in seconds
             * @return Duration in seconds
             */
            double duration(TimeUnit unit = TimeUnit::Seconds) const
            {
                double durationInMilliseconds = mEnd.toMilliseconds() - mStart.toMilliseconds();
                Time t = Time(durationInMilliseconds, TimeUnit::Milliseconds);
                switch (unit)
                {
                case TimeUnit::Milliseconds:
                    return t.toMilliseconds();
                case TimeUnit::Seconds:
                    return t.toSeconds();
                case TimeUnit::Minutes:
                    return t.toMinutes();
                case TimeUnit::Hours:
                    return t.toHours();
                default:
                    return t.toSeconds();
                }
            }

        private:
            /**
             * Start time
             */
            Time mStart;

            /**
             * End time
             */
            Time mEnd;
        };

        // LinearVelocity

        class LinearVelocity
        {
        public:
            /**
             * Constructor
             * @param unit Distance unit
             * @param time Time in seconds
             */
            LinearVelocity(const Unit &unit, TimeUnit timeUnit) : mUnit(unit), mTimeUnit(timeUnit) {}

            /**
             * Get the velocity per TimeUnit
             * @return Velocity in specified time unit
             */
            double get() const
            {
                double timeFactor = 1.0;
                switch (mTimeUnit)
                {
                case TimeUnit::Milliseconds:
                    timeFactor = 1000.0;
                    break;
                case TimeUnit::Seconds:
                    timeFactor = 1.0;
                    break;
                case TimeUnit::Minutes:
                    timeFactor = 1.0 / 60.0;
                    break;
                case TimeUnit::Hours:
                    timeFactor = 1.0 / 3600.0;
                    break;
                default:
                    timeFactor = 1.0;
                    break;
                }
                return mUnit.toMeters() * timeFactor;
            }

            double toMetersPerSecond() const
            {
                return get();
            }

            double toMilesPerHour() const
            {
                return get() * 2.23694;
            }

            double toFeetPerSecond() const
            {
                return get() * 3.28084;
            }

        private:
            /**
             * The distance unit
             */
            Unit mUnit;

            /**
             * The time unit
             */
            TimeUnit mTimeUnit;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(LinearVelocity, mUnit, mTimeUnit)
        };

        class MetersPerSecond : public LinearVelocity
        {
        public:
            /**
             * Constructor
             * @param distance Distance in meters
             * @param time Time in seconds
             */
            MetersPerSecond(const Meters &distance) : LinearVelocity(distance, TimeUnit::Seconds) {}

            static MetersPerSecond from(const double &distance)
            {
                return MetersPerSecond(Meters(distance));
            }
        };

        class FeetPerSecond : public LinearVelocity
        {
        public:
            /**
             * Constructor
             * @param distance Distance in feet
             * @param time Time in seconds
             */
            FeetPerSecond(const Feet &distance) : LinearVelocity(distance, TimeUnit::Seconds) {}

            static FeetPerSecond from(const double &distance)
            {
                return FeetPerSecond(Feet(distance));
            }
        };

        class MilesPerHour : public LinearVelocity
        {
        public:
            /**
             * Constructor
             * @param distance Distance in miles
             * @param time Time in hours
             */
            MilesPerHour(const Miles &distance) : LinearVelocity(distance, TimeUnit::Hours) {}

            static MilesPerHour from(const double &distance)
            {
                return MilesPerHour(Miles(distance));
            }
        };

        // AngularVelocity
        class AngularVelocity
        {
        public:
            /**
             * Constructor
             * @param degrees Degrees per second
             */
            explicit AngularVelocity(double degrees) : mDegreesPerSecond(degrees) {}
            /**
             * Get the angular velocity in degrees per second
             * @return Degrees per second
             */
            double toDegreesPerSecond() const
            {
                return mDegreesPerSecond;
            }

            /**
             * Get the angular velocity in radians per second
             * @return Radians per second
             */
            double toRadiansPerSecond() const
            {
                return mDegreesPerSecond * (3.14159265358979323846 / 180.0);
            }

        private:
            double mDegreesPerSecond;

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(AngularVelocity, mDegreesPerSecond)
        };

        class DegreesPerSecond : public AngularVelocity
        {
        public:
            /**
             * Constructor
             * @param degrees Degrees per second
             */
            explicit DegreesPerSecond(double degrees) : AngularVelocity(degrees) {}

            static DegreesPerSecond from(double degrees)
            {
                return DegreesPerSecond(degrees);
            }
        };

        class RadiansPerSecond : public AngularVelocity
        {
        public:
            /**
             * Constructor
             * @param radians Radians per second
             */
            explicit RadiansPerSecond(double radians) : AngularVelocity(radians * (180.0 / 3.14159265358979323846)) {}

            static RadiansPerSecond from(double radians)
            {
                return RadiansPerSecond(radians);
            }
        };

        // Color
        class Color
        {
        public:
            /**
             * Constructor
             * @param r Red component (0-255)
             * @param g Green component (0-255)
             * @param b Blue component (0-255)
             */
            Color(uint8_t r, uint8_t g, uint8_t b) : mR(r), mG(g), mB(b) {}
            uint8_t r() const { return mR; }
            uint8_t g() const { return mG; }
            uint8_t b() const { return mB; }

        public:
            static Color Red()
            {
                return Color(255, 0, 0);
            }

            static Color Green()
            {
                return Color(0, 255, 0);
            }

            static Color Blue()
            {
                return Color(0, 0, 255);
            }

            static Color White()
            {
                return Color(255, 255, 255);
            }

        private:
            uint8_t mR;
            uint8_t mG;
            uint8_t mB;
        };
    }
}