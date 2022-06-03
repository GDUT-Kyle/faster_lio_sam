#ifndef _INCLUDE_FILTER_STATE_H_
#define _INCLUDE_FILTER_STATE_H_
#include <eigen3/Eigen/Dense>
class FilterState
{
public:
    Eigen::Vector3f rn_;
    Eigen::Vector3f vn_;   // velocity in n-frame
    Eigen::Quaternionf qbn_;  // rotation from b-frame to n-frame
    Eigen::Vector3f ba_;   // acceleartion bias
    Eigen::Vector3f bw_;   // gyroscope bias
    Eigen::Vector3f gn_;   // gravity
public:
    FilterState(){setIdentity();}
    FilterState(const Eigen::Vector3f& rn, const Eigen::Vector3f& vn,
                const Eigen::Quaternionf& qbn, const Eigen::Vector3f& ba,
                const Eigen::Vector3f& bw, const Eigen::Vector3f& gn)
    {
        setIdentity();
        rn_ = rn;
        vn_ = vn;
        qbn_ = qbn;
        ba_ = ba;
        bw_ = bw;
        gn_ = gn;
    }
    ~FilterState(){}

    void setIdentity()
    {
        rn_.setZero();
        vn_.setZero();
        qbn_.setIdentity();
        ba_.setZero();
        bw_.setZero();
        gn_ << 0.0, 0.0, -9.805;
    }

    void updateVelocity(float dt)
    {
        vn_ = rn_ / dt;
    }
};

Eigen::Quaternionf axis2Quat(const Eigen::Vector3f &vec)
{
    Eigen::Quaternionf q;
    float alpha = vec.norm();
    Eigen::AngleAxisf rotation_vector(alpha, vec.normalized());
    q = rotation_vector;
    return q;
}
#endif