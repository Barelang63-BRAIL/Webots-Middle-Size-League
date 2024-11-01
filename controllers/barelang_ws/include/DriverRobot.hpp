#ifndef DRIVER_Node_H
#define DRIVER_Node_H

#include "NodeState.hpp"
#include "webots/Supervisor.hpp"
#include "Kinematic.hpp"
#include "webots/Keyboard.hpp"
#include "webots/Connector.hpp"
#include "webots/Motor.hpp"
#include "RtDB2.h"

#include "vector"
#include "chrono"
#include "webots/Supervisor.hpp"
#include "webots/PositionSensor.hpp"
#include "RtDB_Barelang/rtdbBarelangDefinition.hpp"

class RobotDriver
{
public:
    RobotDriver(int id, std::string netName, bool is_enemy, bool is_keeper) : robot_id(id), network_name(netName), is_enemy(is_enemy), is_keeper(is_keeper) {};

    void init(webots::Supervisor *root, int stepTime)
    {
        mtr_ = kinematic::Motor(60, 180, 300);

        // Device init

        key = root->getKeyboard();
        key->enable(stepTime);

        if (!is_keeper)
        {
            // IC("hahah");
            // Connector Init
            // IC(stepTime);
            connector_ = root->getConnector("connector");
            // IC(connector_->getPresenceSamplingPeriod());
            connector_->enablePresence(stepTime);
            // IC(connector_->getPresence());
        }

        // Wheel Device init

        wheel1 = root->getMotor("wheel1");
        wheel2 = root->getMotor("wheel2");
        wheel3 = root->getMotor("wheel3");

        // Position Sensor Init;
        webots::PositionSensor *ps1, *ps2, *ps3;
        ps1 = root->getPositionSensor("pw1");
        ps2 = root->getPositionSensor("pw2");
        ps3 = root->getPositionSensor("pw3");

        ps1->enable(stepTime);
        ps2->enable(stepTime);
        ps3->enable(stepTime);

        wheel1->setPosition(INFINITY);
        wheel2->setPosition(INFINITY);
        wheel3->setPosition(INFINITY);

        wheel1->setVelocity(0);
        wheel2->setVelocity(0);
        wheel3->setVelocity(0);
        // IC();
        ballNode = root->getFromDef("BALL");
        me_node = root->getFromDef(me_node_name);
        // IC();
        // IC(enemy_list_name);
        for (auto &name_list : enemy_list_name)
        {
            webots::Node *n = root->getFromDef(name_list);
            // IC();
            enemy_node.push_back(n);
        }

        for (auto enemy_ : enemy_node)
        {
            enemy_state.push_back(new simulation::Node(-1, enemy_, is_enemy));
        }

        ball_state = new simulation::Node(-1, ballNode, is_enemy);

        me_state = new simulation::Node(robot_id, me_node, is_enemy);

        umum::Point2D ball_pos = ball_state->getPose();

        umum::Point2D tar_kick = umum::Point2D(-6, (double)std::rand() / RAND_MAX * 2 - 1);
        auto ball2tar_n = (tar_kick - ball_pos).normalize();
        double speed = 5.0;
        umum::Point2D ball_speed = ball2tar_n * speed;
        ball_state->setVelocity(ball_speed);

        if (is_keeper)
        {
            leftrightExt = root->getMotor("extendMotor_lr");
            upExt = root->getMotor("extendMotor_up");
        }

        RtDB2Context ctx = RtDB2Context::Builder(robot_id, RtDB2ProcessType::comm).withConfigFileName("/opt/offline/config/cfg.xml").withNetwork(network_name).build();
        rtdb = new RtDB2(ctx);
    }

    void setNodeName(const std::string &name)
    {
        me_node_name = name;
    }

    void setEnemyNameList(const std::vector<std::string> &name)
    {
        enemy_list_name = name;
    }

    void setBallPosition(umum::Point2D &p)
    {
        ball_state->setNodePositon(p);
    }

    void kick(double speed)
    {
        auto Node_pos = me_state->getPose();
        auto ball_pos = ball_state->getPose();
        auto Node2ball_n = (ball_pos - Node_pos).normalize();
        umum::Point2D ball_speed_req; // = Node2ball_n * speed;
        // double sudut = me->posisi.theta;
        double sudut = me_state->getPose().theta;

        if (me_state->enemy_)
            sudut = sudut + M_PI;

        ball_speed_req.x = cos(sudut);
        ball_speed_req.y = sin(sudut);
        ball_speed_req = ball_speed_req * speed;
        if (connector_->isLocked())
        {
            connector_->unlock();
            ball_state->setVelocity(ball_speed_req);
            arduino_data.from.kicked = true;
            last_kick_time_point = std::chrono::high_resolution_clock::now();
        }
    }

    void step()
    {
        umum::Point2D me_pos = me_state->getPose();
        umum::Point2D me_vel = me_state->getVelocity();

        auto elapsed_kick = (std::chrono::high_resolution_clock::now() - last_kick_time_point).count() / 1e6;
        IC(elapsed_kick);
        if (elapsed_kick > 200)
        {
            arduino_data.from.kicked = false;
        }

        int keyboard = key->getKey();

        // if (is_keeper)
        {
            // IC(key);
            if (keyboard == 82)
            {
                ball_state->setNodePositon({0, 0, 0.05});
                umum::Point2D tar_kick = umum::Point2D(-6, (double)std::rand() / RAND_MAX * 2 - 1);
                auto ball2tar_n = (tar_kick - ball_state->getPose()).normalize();
                double speed = 8.0;
                double tarZ = (double)std::rand() / RAND_MAX * speed;
                umum::Point2D ball_speed = ball2tar_n * speed;
                ball_state->setVelocity(ball_speed, tarZ);
            }
            else if (keyboard == 49)
            { // Free kick enemy and me
                ball_state->setNodePositon({0, 2, 0});
                ball_state->setVelocity({0, 0});
            }
            else if (keyboard == 50) // goal kick enemy
            {
                ball_state->setNodePositon({4, 2, 0});
                ball_state->setVelocity({0, 0});
            }
            else if (keyboard == 52)
            {
                ball_state->setNodePositon({-4, -2, 0});
                ball_state->setVelocity({0, 0});
            }
            else if (keyboard == 51 or abs(ball_state->getPose().x) > 8 or abs(ball_state->getPose().y) > 7)
            {
                ball_state->setNodePositon({0, 0, 0});
                ball_state->setVelocity({0, 0});
            }
            else if (keyboard == 53)
            { // Trowin
                ball_state->setNodePositon({2, 4, 0});
                 ball_state->setVelocity({0, 0});
            }
            else if (keyboard == 54)
            { // Throw in  our
                ball_state->setNodePositon({-2, 4, 0});
                ball_state->setVelocity({0, 0});
            }
            else if(keyboard == 55){
                ball_state->setNodePositon({4,2,0});
                ball_state->setVelocity({0,0});
            }
        }

        umum::Point2D ball_pos = ball_state->getPose();
        umum::Point2D ball_vel = ball_state->getVelocity();

        umum::Point2D target_pos;

        std::vector<umum::Point2D> obstacles;
        obstacles.clear();
        for (auto &enemy_ : enemy_state)
        {
            IC(enemy_->getPose().x, enemy_->getPose().y);
            obstacles.push_back(me_pos.toLocal(enemy_->getPose()));
        }
        // IC(me_pos.x, me_pos.y, me_pos.theta);
        // IC(me_vel.x, me_vel.y, me_vel.theta);
        // IC(keyboard);
        // IC(ball_pos.x, ball_pos.y, ball_pos.theta);
        // IC(ball_vel.x, ball_vel.y, ball_vel.theta);

        IC(obstacles.size());

        rtdb->get("ARDUINO_TO", &arduino_data.to);
        Arduino_t::To_t arduino_data_test;
        int ageArduinoTes;
        if (rtdb->get("ARDUINO_TO_TEST", &arduino_data_test, ageArduinoTes, robot_id) == RTDB2_SUCCESS)
        {
            if (ageArduinoTes < 100)
            {
                arduino_data.to = arduino_data_test;
            }
        }
        rtdb->get("STM_TO", &stm_data.to);
        // IC(stm_data.to.x, stm_data.to.y, stm_data.to.theta);

        bool is_kick = arduino_data.to.kick_on;
        double kicker_power = arduino_data.to.kicker_power;
        // kicker_power /= 10;

        vel_local.x = stm_data.to.x * 100;
        vel_local.y = stm_data.to.y * 100;
        vel_local.theta = stm_data.to.theta;
        STM_t::To_t stm_test;
        int ageTest;
        if (rtdb->get("STM_TO_TEST", &stm_test, ageTest, robot_id) == RTDB2_SUCCESS)
        {
            if (ageTest < 100)
            {
                vel_local.x = stm_test.x * 100;
                vel_local.y = stm_test.y * 100;
                vel_local.theta = stm_test.theta;
            }
        }
        // IC();
        motor_out.v1 = ((-sqrt(3) * vel_local.x / 2) + (vel_local.y / 2) + 20.8 * vel_local.theta) / 6;
        motor_out.v2 = (-vel_local.y + 20.8 * vel_local.theta) / 6;
        motor_out.v3 = ((sqrt(3) * vel_local.x / 2) + (vel_local.y / 2) + 20.8 * vel_local.theta) / 6;

        // auto dist2ball = sqrt(pow(me_pos.x - ball_pos.x, 2) + pow(me_pos.y - ball_pos.y, 2));
        // if(dist2ball <= 0.3)
        // {
        // motor_out = kinematic::Motor();
        // }

        ///
        // IC();
        if (IC(!is_keeper))
        {
            // IC();
            // IC(connector_->isLocked());
            IC(connector_->getPresence());
            IC(arduino_data.to.kick_on, kicker_power);
            IC(is_kick);
            if (connector_->getPresence())
            {
                // IC();
                if (IC(!connector_->isLocked()))
                {
                    connector_->lock();
                    // me->posisiDapatBola = me->posisi;
                    // me->time_posisiDapatBola = std::chrono::high_resolution_clock::now();
                }
                else
                {
                    if (keyboard == 50)
                    {
                        this->kick(5);
                    }
                    if (is_kick)
                    {
                        this->kick(kicker_power);
                    }
                    // if (me->kick)
                    // {
                    //   double multiplier = 1.0;
                    //   if (me->kick_flag == 1)
                    //     multiplier = 5;
                    //   auto dist = (me->target_kick - me->posisi).dist() * multiplier;
                    //   // IC(me->kick_flag, multiplier, dist);
                    //   this->kick(dist);
                    // }
                }
                // me->has_ball = true;
                arduino_data.from.has_ball = 1;

                // motor_out.v1 = 0;
                // motor_out.v2 = 0;
                // motor_out.v3 = 0;
            }
            else
            {
                // IC();
                // me->has_ball = false;
                arduino_data.from.has_ball = 0;
            }
            // IC();
            

            if (keyboard == 49 || keyboard == 50 || keyboard == 51 || keyboard == 52 || keyboard == 53 || keyboard == 54 || keyboard == 55)
            {
                connector_->unlock();
            }

        }
        else
        {
            IC();
            double lrPos = arduino_data.to.leftExt ? 0.1 : arduino_data.to.rightExt ? -0.1
                                                                                    : 0;
            double upPos = arduino_data.to.upExt ? 0.1 : 0;
            leftrightExt->setPosition(lrPos);
            upExt->setPosition(upPos);
        }
        // IC();
        wheel1->setVelocity(motor_out.v1);
        wheel2->setVelocity(motor_out.v2);
        wheel3->setVelocity(motor_out.v3);
        ///

        // wb_motor_set_velocity(wheel1, 0.0);
        // wb_motor_set_velocity(wheel2, 0.0);
        // wb_motor_set_velocity(wheel3, 0.0);

        // IC(me_pos.x, me_pos.y, Node_pos.x, Node_pos.y,ball_vel.x,ball_vel.y);
        // IC(me_pos.x, me_pos.y, me_pos.theta);
        // IC(ball_vel.x , ball_vel.y, me_vel.x, me_vel.y);

        // rtdb->put("NodePOS", &me_pos);
        // rtdb->put("NodeVEL", &me_vel);

        // rtdb->put("BALLPOS", &ball_pos);
        // rtdb->put("BALLVEL", &ball_vel);

        //
        auto ball_relative = me_pos.toLocal(ball_pos);
        omni_data.ball_position_relative = ball_relative;
        if (ball_relative.dist() < 4.0)
            omni_data.nampak_bola = true;
        else
            omni_data.nampak_bola = false;
        omni_data.obstacles_position_relative.resize(obstacles.size());
        for (unsigned int i = 0; i < obstacles.size(); i++)
            omni_data.obstacles_position_relative[i] = obstacles.at(i);

        auto poly = umum::Polygon(8, 4, 0.1);
        poly.global(me_pos);
        if (poly.is_inside(ball_pos))
            zed_data.nampak_bola = 1;

        zed_data.ball_position_relative = {ball_relative.x, ball_relative.y};
        zed_data.height = ball_state->getHeight();
        // omni_data.ball_velocity = ball_vel;
        // me_pos.theta *= 180. / M_PI;
        amcl_data.robot_position = me_pos;
        stm_data.from.robot_velocity = me_vel;
        stm_data.from.position = me_pos;
        // IC();
        rtdb->put("AMCL_FROM", &amcl_data);
        // IC();
        rtdb->put("ARDUINO_FROM", &arduino_data.from);
        rtdb->put("STM_FROM", &stm_data.from);
        rtdb->put("OMNICAMERA_FROM", &omni_data);
        rtdb->put("ZED_FROM", &zed_data);
    }

private:
    std::string me_node_name;
    std::vector<std::string> enemy_list_name;
    int robot_id;
    bool is_enemy, is_keeper;
    std::string network_name;

    RtDB2 *rtdb;

    webots::Node *ballNode;
    webots::Node *me_node;
    std::vector<webots::Node *> enemy_node;
    std::vector<simulation::Node *> enemy_state;

    simulation::Node *ball_state, *me_state;

    kinematic::Motor mtr_;
    std::vector<double> ps{0, 0, 0};
    std::vector<double> prev_pulse{0, 0, 0};
    std::vector<double> v = {0, 0, 0};

    kinematic::Motor motor_out;
    umum::Point2D pos;
    umum::Point2D vel_world;
    umum::Point2D vel_local;
    umum::Point2D vel_glob;
    umum::Point2D velocity_local;
    umum::Point2D gpsPoint;

    webots::Connector *connector_;
    webots::Keyboard *key;
    webots::Motor *wheel1, *wheel2, *wheel3;
    webots::Motor *leftrightExt;
    webots::Motor *upExt;
    Arduino_t arduino_data;
    Amcl_t amcl_data;
    OmniCamera_t omni_data;
    STM_t stm_data;
    ZED_t zed_data;

    std::chrono::high_resolution_clock::time_point last_kick_time_point;
};

#endif