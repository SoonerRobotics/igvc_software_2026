#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "igvc/json.hpp"

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
            update();
        }
        
        GlobalConfig globalConfig;
        VisionConfig visionConfig;
    private:
        Config(std::function<void()> outerUpdateCallback = nullptr)
            : mUpdateCallback(outerUpdateCallback)
        {
            auto updateCallback = []() {
                Config::getInstance().update();
            };

            globalConfig = GlobalConfig(updateCallback);
            visionConfig = VisionConfig(updateCallback);
        }
        ~Config() = default;

        std::function<void()> mUpdateCallback;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Config, globalConfig, visionConfig)
    };
}