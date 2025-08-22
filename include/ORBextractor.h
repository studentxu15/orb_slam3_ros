#ifndef ORBSLAM_ROS_ORB_EXTRACTOR_H
#define ORBSLAM_ROS_ORB_EXTRACTOR_H

#include "header.h"

namespace orb_slam3_ros
{

    /*
        ORB 特征提取过程中用于自适应非极大值抑制
        在 ORB 特征提取阶段，原始 FAST 角点检测可能会在纹理丰富区域产生大量密集特征点，
        而在纹理稀疏区域特征点较少。ExtractorNode通过区域分裂和筛选，平衡特征点的空间分布，
        提升 SLAM 系统的稳定性和匹配精度。
    */
    class ExtractorNode
    {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR; // 定义当前节点的矩形区域（上左、上右、下左、下右坐标），用于划分图像区域
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    /*
    ORBextractor 类是 ORB-SLAM3 中用于提取 ORB 特征的核心类，
    负责从图像中检测关键点（KeyPoint）并计算其描述子（Descriptor），是视觉 SLAM 系统中特征匹配和位姿估计的基础。
    */
    class ORBextractor
    {
    public:
        enum
        {
            HARRIS_SCORE = 0,
            FAST_SCORE = 1
        };

        ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);

        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        int operator()(cv::InputArray _image, cv::InputArray _mask,
                       std::vector<cv::KeyPoint> &_keypoints,
                       cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

        int inline GetLevels()
        {
            return nlevels;
        }

        float inline GetScaleFactor()
        {
            return scaleFactor;
        }

        std::vector<float> inline GetScaleFactors()
        {
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors()
        {
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares()
        {
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares()
        {
            return mvInvLevelSigma2;
        }

        std::vector<cv::Mat> mvImagePyramid;

    protected:
        void ComputePyramid(cv::Mat image);
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
        std::vector<cv::Point> pattern;

        int nfeatures;
        double scaleFactor;
        int nlevels;
        int iniThFAST;
        int minThFAST;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;
    };
}

#endif // ORBSLAM_ROS_ORB_EXTRACTOR_H