#ifndef Node_H
#define Node_H
#include "webots/Supervisor.hpp"
#include "def/def.hpp"
namespace simulation
{
    class Node
    {
    public:
        Node(int id, webots::Node *ref, bool enemy = false);

        void setVelocity(const umum::Point2D &pos, double z);
        void setVelocity(const umum::Point2D &pos);
        umum::Point2D getPose();
        umum::Point2D getVelocity();

        void setNodePositon(const umum::Point2D &p);

        webots::Supervisor *root_;
        webots::Node *node;
        bool enemy_ = false;
        void converRot(const double *rotationMatrix, double &roll, double &pitch, double &yaw);

        std::string NodeName = "TEAM0";

        void setNodeName(const std::string name)
        {
            NodeName = name;
        }

        double getHeight()
        {
            return height;
        }

    private:
        int id_;
        double height;
    };
}

#endif