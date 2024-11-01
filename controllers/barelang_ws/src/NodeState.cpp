#include "icecream.hpp"
#include "NodeState.hpp"
namespace simulation
{

    Node::Node(int id, webots::Node *ref, bool enemy) : id_(id), node(ref), enemy_(enemy)
    {
    }

    void Node::setNodePositon(const umum::Point2D &p)
    {
        webots::Field *fieldRef = node->getField("translation");
        const double d[3] = {p.x, p.y, p.theta};
        fieldRef->setSFVec3f(d);
    }

    void Node::setVelocity(const umum::Point2D &pos, double z)
    {
        const double vel[6] = {pos.x, pos.y, z, 0, 0, 0};
        node->setVelocity(vel);
    }

    void Node::setVelocity(const umum::Point2D &pos)
    {
        const double vel[6] = {pos.x, pos.y, 0, 0, 0, 0};
        node->setVelocity(vel);
    }

    umum::Point2D Node::getPose()
    {
        umum::Point2D p;
        const double *pos = node->getPosition();

        double tmp = 1;
        if (this->enemy_)
            tmp = -1;
        p.x = pos[0] * tmp;
        p.y = pos[1] * tmp;
        height = pos[2];

        const double *orientation = node->getOrientation();

        double roll, pitch, yaw;
        converRot(orientation, roll, pitch, yaw);
        if (enemy_)
            yaw = yaw + M_PI;
        while (yaw > 2 * M_PI)
            yaw -= 2 * M_PI;
        while (yaw < -2 * M_PI)
            yaw += 2 * M_PI;
        p.theta = yaw;
        // IC(p.x, p.y, height, yaw);
        return p;
    }

    umum::Point2D Node::getVelocity()
    {
        const double *vel = node->getVelocity();
        umum::Point2D p;
        double tmp = 1;
        if (this->enemy_)
            tmp = -1;
        p.x = vel[0] * tmp;
        p.y = vel[1] * tmp;

        umum::Point2D local;
        auto this_pose = this->getPose();
        local.x = std::cos(this_pose.theta) * p.x + std::sin(this_pose.theta) * p.y;
        local.y = std::cos(this_pose.theta) * p.y - std::sin(this_pose.theta) * p.x;
        // p.theta = vel[5];

        return local;
    }

    void Node::converRot(const double *rotationMatrix, double &roll, double &pitch, double &yaw)
    {
        pitch = asin(-rotationMatrix[6]); // Calculate pitch

        // Check for special case: pitch close to ±90 degrees

        if (fabs(cos(pitch)) > 1e-6)
        {
            roll = atan2(rotationMatrix[7] / cos(pitch), rotationMatrix[8] / cos(pitch)); // Calculate roll
            yaw = atan2(rotationMatrix[3] / cos(pitch), rotationMatrix[0] / cos(pitch));  // Calculate yaw
        }
        else
        {
            // Gimbal lock case (pitch is close to ±90 degrees)
            roll = 0.0;                                        // Roll is 0 in this case
            yaw = atan2(rotationMatrix[2], rotationMatrix[4]); // Calculate yaw
        }

        // Convert angles to degrees if needed
        // roll = roll * 180.0 / M_PI;
        // pitch = pitch * 180.0 / M_PI;
        // yaw = yaw * 180.0 / M_PI;
    }

}
