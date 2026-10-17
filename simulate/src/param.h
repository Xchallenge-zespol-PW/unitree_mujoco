#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>

namespace param
{

constexpr int IDL_AUTO = -1;
constexpr int IDL_GO = 0;
constexpr int IDL_HG = 1;
constexpr int IDL_GO_MOTOR_LIMIT = 20;
constexpr int IDL_HG_MOTOR_LIMIT = 35;

inline int ResolveIdlType(const std::string &robot, int actuator_count, int configured_idl_type)
{
    if (configured_idl_type != IDL_AUTO &&
        configured_idl_type != IDL_GO &&
        configured_idl_type != IDL_HG)
    {
        throw std::invalid_argument("idl_type must be -1 (auto), 0 (unitree_go), or 1 (unitree_hg)");
    }

    int idl_type = configured_idl_type;
    if (idl_type == IDL_AUTO)
    {
        idl_type = robot == "as2" || actuator_count > IDL_GO_MOTOR_LIMIT ? IDL_HG : IDL_GO;
    }

    const int motor_limit = idl_type == IDL_GO ? IDL_GO_MOTOR_LIMIT : IDL_HG_MOTOR_LIMIT;
    if (actuator_count > motor_limit)
    {
        throw std::invalid_argument(
            std::string(idl_type == IDL_GO ? "unitree_go" : "unitree_hg") +
            " IDL supports at most " + std::to_string(motor_limit) +
            " actuators, but the model has " + std::to_string(actuator_count));
    }

    return idl_type;
}

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;
    int idl_type = IDL_AUTO;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    bool lidar_enabled = false;
    std::string lidar_site_name = "lidar_site";
    std::string lidar_frame_id = "lidar_frame";
    std::string lidar_topic = "/lidar/points";
    int lidar_horizontal_samples = 180;
    int lidar_vertical_samples = 16;
    double lidar_vertical_min_deg = -15.0;
    double lidar_vertical_max_deg = 15.0;
    double lidar_range_min_m = 0.1;
    double lidar_range_max_m = 20.0;
    double lidar_rate_hz = 5.0;
    bool lidar_exclude_mount_body = true;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            if (cfg["idl_type"])
            {
                idl_type = cfg["idl_type"].as<int>();
            }
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["lidar"])
            {
                const auto lidar = cfg["lidar"];
                if (lidar["enabled"]) lidar_enabled = lidar["enabled"].as<bool>();
                if (lidar["site_name"]) lidar_site_name = lidar["site_name"].as<std::string>();
                if (lidar["frame_id"]) lidar_frame_id = lidar["frame_id"].as<std::string>();
                if (lidar["topic"]) lidar_topic = lidar["topic"].as<std::string>();
                if (lidar["horizontal_samples"]) lidar_horizontal_samples = lidar["horizontal_samples"].as<int>();
                if (lidar["vertical_samples"]) lidar_vertical_samples = lidar["vertical_samples"].as<int>();
                if (lidar["vertical_min_deg"]) lidar_vertical_min_deg = lidar["vertical_min_deg"].as<double>();
                if (lidar["vertical_max_deg"]) lidar_vertical_max_deg = lidar["vertical_max_deg"].as<double>();
                if (lidar["range_min_m"]) lidar_range_min_m = lidar["range_min_m"].as<double>();
                if (lidar["range_max_m"]) lidar_range_max_m = lidar["range_max_m"].as<double>();
                if (lidar["rate_hz"]) lidar_rate_hz = lidar["rate_hz"].as<double>();
                if (lidar["exclude_mount_body"]) lidar_exclude_mount_body = lidar["exclude_mount_body"].as<bool>();
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("idl_type,t", po::value<int>(&config.idl_type), "DDS IDL type: -1 auto, 0 unitree_go, 1 unitree_hg")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}