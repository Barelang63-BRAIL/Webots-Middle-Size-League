#include "icecream.hpp"
#include "iostream"
#include "DriverRobot.hpp"

using namespace webots;

int main(int argc, char const *argv[])
{

    std::string device_name, network_name;
    std::vector<std::string> enemy_list;
    int id_robot;
    bool is_enemy, is_keeper;
    if (argc <= 1 || (argc > 1 && argc < 7))
    {
        std::cout << "robot_name robot_id network_name is_enemy enemy_list is_keeper\n";
        return -1;
    }
    else
    {
        // IC();
        device_name = argv[1];
        id_robot = std::atoi(argv[2]);
        network_name = argv[3];
        is_enemy = (bool)std::atoi(argv[4]);
        std::vector<std::string> enemy_tmp = {argv[5], argv[6], argv[7]};
        is_keeper = (bool)std::atoi(argv[8]);
        for (auto enemy : enemy_tmp)
        {
            enemy_list.push_back(enemy);
        }
    }
    IC(argc);
    IC(device_name, id_robot, is_keeper, is_enemy, network_name);
    IC(enemy_list);

    Supervisor *root = new Supervisor();

    int stepTime = root->getBasicTimeStep();

    RobotDriver *driver = new RobotDriver(id_robot, network_name, is_enemy, is_keeper);

    driver->setNodeName(device_name);
    driver->setEnemyNameList(enemy_list);

    driver->init(root, stepTime);

    while (root->step(stepTime) != -1)
    {
        // IC();
        driver->step();
    }

    enemy_list.clear();

    // IC();
    /* code */
    return 0;
}
