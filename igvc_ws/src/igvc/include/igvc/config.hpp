#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "igvc/json.hpp"
#include "igvc/units.hpp"

using UpdateFn = void (*)();
namespace IGVC
{
    class GlobalConfig
    {
    public:
        GlobalConfig(UpdateFn updateCallback = nullptr) : mUpdateCallback(updateCallback) {}
        ~GlobalConfig() = default;

    public:
        uint32_t getMapResolution() const
        {
            return mMapResolution;
        }

        void setMapResolution(uint32_t resolution)
        {
            mMapResolution = resolution;
            mUpdateCallback();
        }

    private:
        uint32_t mMapResolution = 80;
        UpdateFn mUpdateCallback = nullptr;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(GlobalConfig, mMapResolution)
    };

    class VisionConfig
    {
    public:
        VisionConfig(UpdateFn updateCallback = nullptr) : mUpdateCallback(updateCallback) {}
        ~VisionConfig() = default;

    public:
        void setLaneHsvLower(uint32_t h, uint32_t s, uint32_t v)
        {
            mLaneHsvLower[0] = h;
            mLaneHsvLower[1] = s;
            mLaneHsvLower[2] = v;
            mUpdateCallback();
        }

        void setLaneHsvUpper(uint32_t h, uint32_t s, uint32_t v)
        {
            mLaneHsvUpper[0] = h;
            mLaneHsvUpper[1] = s;
            mLaneHsvUpper[2] = v;
            mUpdateCallback();
        }

        const cv::Scalar getLaneHsvLower() const
        {
            return cv::Scalar(
                mLaneHsvLower[0],
                mLaneHsvLower[1],
                mLaneHsvLower[2]
            );
        }

        const cv::Scalar getLaneHsvUpper() const
        {
            return cv::Scalar(
                mLaneHsvUpper[0],
                mLaneHsvUpper[1],
                mLaneHsvUpper[2]
            );
        }

        uint32_t getGaussianBlurKernelSize() const
        {
            return mGaussianBlurKernelSize;
        }

        void setGaussianBlurKernelSize(uint32_t size)
        {
            mGaussianBlurKernelSize = size;
            mUpdateCallback();
        }

        float getGaussianBlurSigma() const
        {
            return mGaussianBlurSigma;
        }

        void setGaussianBlurSigma(float sigma)
        {
            mGaussianBlurSigma = sigma;
            mUpdateCallback();
        }

    private:
        // hsv thresholds
        uint32_t mLaneHsvLower[3] = {0, 0, 0};
        uint32_t mLaneHsvUpper[3] = {85, 255, 255};
        
        // blurring
        uint32_t mGaussianBlurKernelSize = 5;
        float mGaussianBlurSigma = 1.5f;

        UpdateFn mUpdateCallback = nullptr;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(VisionConfig, mLaneHsvLower, mLaneHsvUpper, mGaussianBlurKernelSize, mGaussianBlurSigma)
    };

    class ManualControlConfig
    {
    public:
        ManualControlConfig(UpdateFn updateCallback = nullptr) : mUpdateCallback(updateCallback) {}
        ~ManualControlConfig() = default;

        IGVC::Units::LinearVelocity getForwardSpeed() const
        {
            return mForwardSpeed;
        }

        void setForwardSpeed(const IGVC::Units::LinearVelocity &speed)
        {
            mForwardSpeed = speed;
            mUpdateCallback();
        }

        IGVC::Units::AngularVelocity getTurnSpeed() const
        {
            return mTurnSpeed;
        }

        void setTurnSpeed(const IGVC::Units::AngularVelocity &speed)
        {
            mTurnSpeed = speed;
            mUpdateCallback();
        }

        void setSidewaysSpeed(const IGVC::Units::LinearVelocity &speed)
        {
            mSidewaysSpeed = speed;
            mUpdateCallback();
        }

        IGVC::Units::LinearVelocity getSidewaysSpeed() const
        {
            return mSidewaysSpeed;
        }

    private:
        IGVC::Units::LinearVelocity mForwardSpeed = IGVC::Units::MilesPerHour::from(4.0);
        IGVC::Units::LinearVelocity mSidewaysSpeed = IGVC::Units::MilesPerHour::from(4.0);
        IGVC::Units::AngularVelocity mTurnSpeed = IGVC::Units::DegreesPerSecond::from(45.0);

        UpdateFn mUpdateCallback = nullptr;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(ManualControlConfig, mForwardSpeed, mTurnSpeed, mSidewaysSpeed)
    };

    class MotorControlConfig
    {
    public:
        MotorControlConfig(UpdateFn updateCallback = nullptr) : mUpdateCallback(updateCallback) {}
        ~MotorControlConfig() = default;

        IGVC::Units::LinearVelocity getMaxForwardSpeed() const
        {
            return mMaxForwardSpeed;
        }

        void setMaxForwardSpeed(const IGVC::Units::LinearVelocity &speed)
        {
            mMaxForwardSpeed = speed;
            mUpdateCallback();
        }

        IGVC::Units::LinearVelocity getMaxSidewaysSpeed() const
        {
            return mMaxSidewaysSpeed;
        }

        void setMaxSidewaysSpeed(const IGVC::Units::LinearVelocity &speed)
        {
            mMaxSidewaysSpeed = speed;
            mUpdateCallback();
        }

        IGVC::Units::AngularVelocity getMaxTurnSpeed() const
        {
            return mMaxTurnSpeed;
        }

        void setMaxTurnSpeed(const IGVC::Units::AngularVelocity &speed)
        {
            mMaxTurnSpeed = speed;
            mUpdateCallback();
        }

    private:
        IGVC::Units::LinearVelocity mMaxForwardSpeed = IGVC::Units::MilesPerHour::from(5.5);
        IGVC::Units::LinearVelocity mMaxSidewaysSpeed = IGVC::Units::MilesPerHour::from(5.5);
        IGVC::Units::AngularVelocity mMaxTurnSpeed = IGVC::Units::DegreesPerSecond::from(120.0);

        UpdateFn mUpdateCallback = nullptr;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(MotorControlConfig, mMaxForwardSpeed, mMaxSidewaysSpeed, mMaxTurnSpeed)
    };

    class Config
    {
    public:
        // take in a binded update function to call on updates
        // eg we would call: Config::getInstance(std::bind(&IGVC::Node::onLocalConfigUpdated, this));
        static Config &getInstance(std::function<void()> updateCallback = nullptr)
        {
            auto realUpdateCallback = [updateCallback]() {
                if (updateCallback)
                {
                    updateCallback();
                }
            };

            static Config instance(realUpdateCallback);
            return instance;
        }


        void update()
        {
            if (mUpdateCallback)
            {
                mUpdateCallback();
            }
        }

        void loadFromJson(const std::string &jsonStr)
        {
            nlohmann::json j = nlohmann::json::parse(jsonStr);
            j.get_to(*this);
        }
        
        GlobalConfig globalConfig;
        VisionConfig visionConfig;
        MotorControlConfig motorControlConfig;
        ManualControlConfig manualControlConfig;
    private:
        Config(std::function<void()> outerUpdateCallback = nullptr)
            : mUpdateCallback(outerUpdateCallback)
        {
            auto updateCallback = []() {
                Config::getInstance().update();
            };

            globalConfig = GlobalConfig(updateCallback);
            visionConfig = VisionConfig(updateCallback);
            motorControlConfig = MotorControlConfig(updateCallback);
            manualControlConfig = ManualControlConfig(updateCallback);
        }
        ~Config() = default;

        std::function<void()> mUpdateCallback;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, globalConfig, visionConfig, motorControlConfig, manualControlConfig)
    };
}