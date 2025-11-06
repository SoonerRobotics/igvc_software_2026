#pragma once

#include <type_traits>

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
            explicit Miles(double value) : Unit(value * 1609.34) {}
        };

        static Miles from(double value)
        {
            return Miles(value);
        }
    };

    namespace Velocity
    {
        template <typename UnitType, typename = std::enable_if_t<std::is_base_of<Units::Unit, UnitType>::value>>
        class LinearVelocity
        {
        public:
            /**
             * Constructor
             * @param distance Distance unit
             * @param time Time in seconds
             */
            LinearVelocity(const UnitType& distance, double time) : mDistance(distance), mTime(time) {}

        private:
            UnitType mDistance;
            double mTime;
        };
    }
}