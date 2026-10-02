#pragma once

#include <array>
#include <cmath>
#include <cstddef>

struct Vec2
{
    double x;
    double y;
};

struct JointAngles
{
    double theta1;
    double theta2;
};

class Kinematics
{
public:
    Kinematics(double l1 = 0.5, double l2 = 0.5);

    void setLinkLengths(double l1, double l2);

    // Forward Kinematics
    Vec2 getJointPosition(double theta1) const;
    Vec2 forwardKinematics(double theta1, double theta2) const;

    // Inverse Kinematics
    bool inverseKinematics(
        double x,
        double y,
        JointAngles& result
    ) const;

    bool isReachable(double x, double y) const;

private:
    double link1_length;
    double link2_length;
};

// A small, screen-space planar arm model. Angles are radians and rotate in
// the Win32 screen coordinate system (positive Y points down).
class PlanarArm
{
public:
    PlanarArm();

    void setDegreesOfFreedom(int dof);
    int degreesOfFreedom() const { return m_dof; }

    void setLinkLengths(const std::array<double, 3>& lengths);
    const std::array<double, 3>& linkLengths() const { return m_lengths; }
    const std::array<double, 3>& angles() const { return m_angles; }

    bool isReachable(const Vec2& targetFromRoot) const;
    // Returns false without changing the pose when the target is unreachable.
    bool solve(const Vec2& targetFromRoot);
    std::array<Vec2, 4> jointPositions(const Vec2& root) const;

private:
    std::array<Vec2, 4> localJointPositions() const;

    int m_dof = 2;
    std::array<double, 3> m_lengths{185.0, 145.0, 110.0};
    std::array<double, 3> m_angles{0.0, 0.0, 0.0};
};
