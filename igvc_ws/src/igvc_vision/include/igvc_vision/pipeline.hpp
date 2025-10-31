#pragma once

#include <opencv2/opencv.hpp>
#include <rcpputils/asserts.hpp>

namespace IGVC
{
    /**
     * @brief Class representing an image processing pipeline.
     */
    class ImagePipeline
    {
    public:
        ~ImagePipeline() = default;

        /**
         * @brief Add an image processing module to the pipeline.
         * @param processor Shared pointer to the image processing module.
         */
        ImagePipeline &addProcessor(const std::shared_ptr<class BaseImageProcessor> &processor);

        /**
         * @brief Process an input image through the pipeline.
         * @param inputImage The input image to process.
         */
        void processImage(const cv::Mat &inputImage, cv::Mat &outputImage);

    private:
        std::vector<std::shared_ptr<class BaseImageProcessor>> mProcessors;
    };

    /**
     * @brief Abstract base class for image processing modules in the pipeline.
     */
    class BaseImageProcessor
    {
    public:
        virtual ~BaseImageProcessor() = default;

        /**
         * @brief Process an input image and produce an output image.
         * @param inputImage The input image to process.
         * @param outputImage The processed output image.
         */
        virtual void process(const cv::Mat &inputImage, cv::Mat &outputImage) = 0;
    };

    #pragma region Processors

    /**
     * @brief HSV Thresholding processor.
     */
    class HSVThresholdProcessor : public BaseImageProcessor
    {
    public:
        HSVThresholdProcessor(const cv::Scalar &lowerBound, const cv::Scalar &upperBound);

        void process(const cv::Mat &inputImage, cv::Mat &outputImage) override;
    private:
        cv::Scalar mLowerBound;
        cv::Scalar mUpperBound;
    };

    /**
     * @brief Gaussian Blur processor.
     */
    class GaussianBlurProcessor : public BaseImageProcessor
    {
    public:
        /** 
         * @brief Constructor for GaussianBlurProcessor.
         * @param kernelSize Size of the kernel (must be odd).
         * @param sigmaX Standard deviation in X direction.
         */
        GaussianBlurProcessor(int kernelSize, double sigmaX);
        
        void process(const cv::Mat &inputImage, cv::Mat &outputImage) override;

    private:
        int mKernelSize;
        double mSigmaX;
    };

    #pragma endregion
}