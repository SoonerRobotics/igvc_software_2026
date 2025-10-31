#include "igvc_vision/pipeline.hpp"

namespace IGVC
{
    ImagePipeline &ImagePipeline::addProcessor(const std::shared_ptr<BaseImageProcessor> &processor)
    {
        mProcessors.push_back(processor);
        return *this;
    }

    void ImagePipeline::processImage(const cv::Mat &inputImage, cv::Mat &outputImage)
    {
        cv::Mat currentImage = inputImage.clone();
        for (const auto &processor : mProcessors)
        {
            cv::Mat processedImage;
            processor->process(currentImage, processedImage);
            currentImage = processedImage;
        }
        outputImage = currentImage;
    }

    HSVThresholdProcessor::HSVThresholdProcessor(const cv::Scalar &lowerBound, const cv::Scalar &upperBound)
        : mLowerBound(lowerBound), mUpperBound(upperBound) {}

    GaussianBlurProcessor::GaussianBlurProcessor(int kernelSize, double sigmaX)
        : mKernelSize(kernelSize), mSigmaX(sigmaX) {}

    void GaussianBlurProcessor::process(const cv::Mat &inputImage, cv::Mat &outputImage)
    {
        rcpputils::assert_true(mKernelSize % 2 == 1, "Kernel size must be odd.");
        rcpputils::assert_true(mSigmaX > 0, "SigmaX must be positive.");

        cv::GaussianBlur(inputImage, outputImage, cv::Size(mKernelSize, mKernelSize), mSigmaX);
    }

    void HSVThresholdProcessor::process(const cv::Mat &inputImage, cv::Mat &outputImage)
    {
        cv::Mat hsvImage;
        cv::cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, mLowerBound, mUpperBound, outputImage);
        outputImage = 255 - outputImage; // invert the mask so that obstacles are white (1)
    }
}