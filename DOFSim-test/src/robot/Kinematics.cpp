#include "Kinematics.hpp"

#include <algorithm>

namespace
{
constexpr double kEpsilon = 1e-9;

double length(const Vec2& value)
{
    return std::sqrt(value.x * value.x + value.y * value.y);
}
}

Kinematics::Kinematics(double l1, double l2)
    : link1_length(l1)
    , link2_length(l2)
{
}

void Kinematics::setLinkLengths(double l1, double l2)
{
    link1_length = std::max(0.0, l1);
    link2_length = std::max(0.0, l2);
}

Vec2 Kinematics::getJointPosition(double theta1) const
{
    return
    {
        link1_length * std::cos(theta1),
        link1_length * std::sin(theta1)
    };
}

Vec2 Kinematics::forwardKinematics(double theta1, double theta2) const
{
    const Vec2 joint = getJointPosition(theta1);
    return {
        joint.x + link2_length * std::cos(theta1 + theta2),
        joint.y + link2_length * std::sin(theta1 + theta2)
    };
}

bool Kinematics::inverseKinematics(double x, double y, JointAngles& result) const
{
    if (!isReachable(x, y))
    {
        return false;
    }

    const double radiusSquared = x * x + y * y;
    const double denominator = 2.0 * link1_length * link2_length;
    if (std::abs(denominator) < kEpsilon)
    {
        return false;
    }

    const double cosineTheta2 = std::clamp(
        (radiusSquared - link1_length * link1_length - link2_length * link2_length) / denominator,
        -1.0,
        1.0);
    result.theta2 = std::acos(cosineTheta2); // Elbow-down solution.
    result.theta1 = std::atan2(y, x) - std::atan2(
        link2_length * std::sin(result.theta2),
        link1_length + link2_length * std::cos(result.theta2));
    return true;
}

PlanarArm::PlanarArm() = default;

void PlanarArm::setDegreesOfFreedom(int dof)
{
    m_dof = (dof == 3) ? 3 : 2;
    if (m_dof == 2)
    {
        m_angles[2] = 0.0;
    }
}

void PlanarArm::setLinkLengths(const std::array<double, 3>& lengths)
{
    for (std::size_t i = 0; i < m_lengths.size(); ++i)
    {
        m_lengths[i] = std::max(1.0, lengths[i]);
    }
}

bool PlanarArm::isReachable(const Vec2& targetFromRoot) const
{
    double totalLength = 0.0;
    double longestLength = 0.0;
    for (int i = 0; i < m_dof; ++i)
    {
        totalLength += m_lengths[static_cast<std::size_t>(i)];
        longestLength = std::max(longestLength, m_lengths[static_cast<std::size_t>(i)]);
    }

    const double minimumRadius = std::max(0.0, 2.0 * longestLength - totalLength);
    const double radius = length(targetFromRoot);
    return radius <= totalLength + 0.001 && radius + 0.001 >= minimumRadius;
}

std::array<Vec2, 4> PlanarArm::localJointPositions() const
{
    std::array<Vec2, 4> joints{};
    double globalAngle = 0.0;
    for (int i = 0; i < m_dof; ++i)
    {
        globalAngle += m_angles[static_cast<std::size_t>(i)];
        joints[static_cast<std::size_t>(i + 1)] = {
            joints[static_cast<std::size_t>(i)].x + m_lengths[static_cast<std::size_t>(i)] * std::cos(globalAngle),
            joints[static_cast<std::size_t>(i)].y + m_lengths[static_cast<std::size_t>(i)] * std::sin(globalAngle)
        };
    }
    return joints;
}

bool PlanarArm::solve(const Vec2& targetFromRoot)
{
    if (!isReachable(targetFromRoot))
    {
        return false;
    }

    if (m_dof == 2)
    {
        Kinematics solver(m_lengths[0], m_lengths[1]);
        JointAngles solution{};
        if (!solver.inverseKinematics(targetFromRoot.x, targetFromRoot.y, solution))
        {
            return false;
        }
        m_angles[0] = solution.theta1;
        m_angles[1] = solution.theta2;
        m_angles[2] = 0.0;
        return true;
    }

    // Cyclic coordinate descent: rotate each joint so the current end
    // effector vector points more directly at the target, from wrist to root.
    for (int iteration = 0; iteration < 96; ++iteration)
    {
        auto joints = localJointPositions();
        const Vec2 endEffector = joints[static_cast<std::size_t>(m_dof)];
        if (length({targetFromRoot.x - endEffector.x, targetFromRoot.y - endEffector.y}) < 0.25)
        {
            return true;
        }

        for (int jointIndex = m_dof - 1; jointIndex >= 0; --jointIndex)
        {
            const Vec2 pivot = joints[static_cast<std::size_t>(jointIndex)];
            const Vec2 currentEndEffector = joints[static_cast<std::size_t>(m_dof)];
            const Vec2 toEnd{currentEndEffector.x - pivot.x, currentEndEffector.y - pivot.y};
            const Vec2 toTarget{targetFromRoot.x - pivot.x, targetFromRoot.y - pivot.y};
            const double product = toEnd.x * toTarget.x + toEnd.y * toTarget.y;
            const double cross = toEnd.x * toTarget.y - toEnd.y * toTarget.x;
            m_angles[static_cast<std::size_t>(jointIndex)] += std::atan2(cross, product);
            joints = localJointPositions();
        }
    }

    const auto joints = localJointPositions();
    const Vec2 endEffector = joints[static_cast<std::size_t>(m_dof)];
    return length({targetFromRoot.x - endEffector.x, targetFromRoot.y - endEffector.y}) < 1.0;
}

std::array<Vec2, 4> PlanarArm::jointPositions(const Vec2& root) const
{
    auto joints = localJointPositions();
    for (auto& joint : joints)
    {
        joint.x += root.x;
        joint.y += root.y;
    }
    return joints;
}

bool Kinematics::isReachable(double x, double y) const
{
    double dist = std::sqrt(x * x + y * y);

    return (dist <= link1_length+link2_length) && (dist >= std::abs(link1_length - link2_length));
}
