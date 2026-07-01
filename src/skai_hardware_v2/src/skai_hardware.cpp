#include "skai_hardware_v2/skai_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <iostream>
#include <cmath>
#include <cstring>
#include <errno.h>

constexpr size_t ACTIVE_JOINTS = 8;

namespace skai_hardware_v2
{

hardware_interface::CallbackReturn SKAIHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  /* --- INITIALIZE JOINT STORAGE --- */
  position_states_.resize(info_.joints.size(), 0.0);
  position_commands_.resize(info_.joints.size(), 0.0);
  raw_position_states_.resize(info_.joints.size(), 0.0);

  std::cout << "\nSKAI Hardware Initialized" << std::endl;

  /* --- SOCKETCAN INITIALIZATION --- */
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0)
  {
    std::cout << "Failed to create CAN socket" << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  strcpy(ifr_.ifr_name, "can0");
  ioctl(can_socket_, SIOCGIFINDEX, &ifr_);

  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
  {
    std::cout << "Failed to bind CAN socket" << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout << "SocketCAN initialized" << std::endl;

  /* --- CAN SEND GATE NODE --- */
  enable_node_ = rclcpp::Node::make_shared("skai_can_gate");

  enable_sub_ = enable_node_->create_subscription<std_msgs::msg::Bool>(
    "/can_send_enable",
    10,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      send_enabled_ = msg->data;
      std::cout << "CAN send " << (msg->data ? "ENABLED" : "DISABLED") << std::endl;
    }
  );

  enable_thread_ = std::thread([this]() { rclcpp::spin(enable_node_); });
  enable_thread_.detach();

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* --- ON ACTIVATE --- */
hardware_interface::CallbackReturn SKAIHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  clear_errors();
  usleep(100000);

  set_closed_loop();
  usleep(100000);

  std::cout << "\n===== JOINT ORDER =====\n";

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    std::cout
      << i
      << " : "
      << info_.joints[i].name
      << std::endl;
  }

  send_enabled_ = true;
  commands_seeded_ = false;

  std::cout << "SKAI Hardware activated" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* --- CLEAR ODRIVE ERRORS --- */
void SKAIHardware::clear_errors()
{
  for (int node_id : node_ids_)
  {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x018;
    frame.can_dlc = 0;

    ::write(can_socket_, &frame, sizeof(frame));
  }
  std::cout << "ODrive errors cleared" << std::endl;
}

/* --- SET CLOSED LOOP CONTROL --- */
void SKAIHardware::set_closed_loop()
{
  for (int node_id : node_ids_)
  {
    // if (node_id == 27)
    // {
    //   continue;   // skip gripper for now
    // }

    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x007;
    frame.can_dlc = 4;

    uint32_t state = 8;
    memcpy(&frame.data[0], &state, sizeof(uint32_t));

    ::write(can_socket_, &frame, sizeof(frame));
  }

  std::cout << "ODrive CLOSED_LOOP sent" << std::endl;
}

/* --- EXPORT STATE INTERFACES --- */
std::vector<hardware_interface::StateInterface> SKAIHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[i]
    );
  }
  return state_interfaces;
}

/* --- EXPORT COMMAND INTERFACES --- */
std::vector<hardware_interface::CommandInterface> SKAIHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_commands_[i]
    );
  }
  return command_interfaces;
}

/* --- READ --- */
hardware_interface::return_type SKAIHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) 
{
  /* Request encoder estimates from connected ODrives */
  for (size_t i = 0; i < ACTIVE_JOINTS; i++)
  {
    struct can_frame req;
    req.can_id = ((uint32_t)node_ids_[i] << 5) | 0x009 | CAN_RTR_FLAG;
    req.can_dlc = 0;

    ::write(can_socket_, &req, sizeof(req));
  }

  /* Collect Responses */
  std::vector<bool> received(ACTIVE_JOINTS, false);
  int received_count = 0;
  const int needed = static_cast<int>(ACTIVE_JOINTS);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);

  while (received_count < needed)
  {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) break;

    long us_left = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = static_cast<suseconds_t>(us_left);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(can_socket_, &read_fds);

    int ret = select(can_socket_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) break;

    struct can_frame frame;
    int nbytes = ::read(can_socket_, &frame, sizeof(frame));
    if (nbytes < static_cast<int>(sizeof(frame))) continue;

    uint32_t raw_id = frame.can_id & ~(CAN_RTR_FLAG | CAN_EFF_FLAG | CAN_ERR_FLAG);
    int cmd_id = static_cast<int>(raw_id & 0x1F);
    int node_id = static_cast<int>(raw_id >> 5);

    if (cmd_id != 0x009) continue;

    for (size_t i = 0; i < ACTIVE_JOINTS; i++)
    {
      if (node_ids_[i] == node_id && !received[i])
      {
        float output_turns;
        memcpy(&output_turns, &frame.data[0], sizeof(float));

        double motor_turns = static_cast<double>(output_turns);
        double radians =
            motor_turns * 2.0 * M_PI;
            
        const std::string & jname = info_.joints[i].name;


      /* Existing sign corrections */
      if (jname == "PITCH_1" || jname == "PITCH_2")
      {
        radians = -radians;
      }

      /*
      * Store RAW encoder value
      * (used later for command seeding)
      */
      raw_position_states_[i] = radians;

      /*
      * Software calibration offsets
      * Physical EXT pose -> CAD/MoveIt zero pose
      */
      constexpr double ROT1_OFFSET = 0.261799;  

      double moveit_value = radians;

      if (jname == "ROT_1")
      {
          moveit_value += ROT1_OFFSET;
      }
      else if (jname == "PITCH_1")
      {
          moveit_value += 1.464297;
      }
      else if (jname == "PITCH_2")
      {
          moveit_value += 0.113097;
      }
      else if (jname == "FGR_1")
      {
          /*
          * ODrive feedback is in TURNS
          *
          * OPEN   = -0.1800 turns
          * CLOSED = -0.1435 turns
          *
          * MoveIt:
          * OPEN   = 0.000 m
          * CLOSED = 0.026 m
          */

          moveit_value =
              ((motor_turns + 0.180) / 0.030) * 0.026;

          moveit_value =
              std::clamp(moveit_value, 0.0, 0.026);
      }

      /*
      * Publish calibrated value to ROS/MoveIt
      */
      position_states_[i] = moveit_value;

      if (jname == "FGR_1")
      {
          for (size_t j = 0; j < info_.joints.size(); j++)
          {
              if (info_.joints[j].name == "FGR_2")
              {
                  position_states_[j] = -position_states_[i];
                  break;
              }
          }
      }

      received[i] = true;
      received_count++;
      break;
      }
    }
  }

  /* Debug Logging */
  if (received_count > 0)
  {
    static int enc_log_ctr = 0;
    if (++enc_log_ctr % 40 == 0)
    {
      std::cout << "[ENC]";
      for (size_t k = 0; k < position_states_.size(); k++)
      {
        std::cout << " J" << k << "=" << position_states_[k] << "rad";
      }
      std::cout << std::endl;
    }
  }

  return hardware_interface::return_type::OK;
}

/* --- WRITE --- */
hardware_interface::return_type SKAIHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!send_enabled_)
  {
    return hardware_interface::return_type::OK;
  }

  /* Seed commands from live positions on first gate enable */
  if (!commands_seeded_)
  {
      for (size_t i = 0; i < position_commands_.size(); i++)
      {
          position_commands_[i] = position_states_[i];

          if (info_.joints[i].name == "ROT_1")
          {
              std::cout
                << "\n=== ROT1 SEED ==="
                << "\nraw encoder = " << raw_position_states_[i]
                << "\nmoveit      = " << position_states_[i]
                << std::endl;
          }
      }

      commands_seeded_ = true;
  }

  for (size_t i = 0; i < ACTIVE_JOINTS; i++)
  {

    constexpr double ROT1_OFFSET = 0.261799;

    float joint_radians = position_commands_[i];
    const std::string & jname = info_.joints[i].name;

    // if (jname == "FGR_1")
    // {
    //     continue;
    // }

    if (jname == "FGR_1")
    {
        if (first_gripper_write_)
        {
            /*
            * raw_position_states_ is stored in radians
            * convert back to turns for ODrive
            */
            joint_radians =
                raw_position_states_[i] / (2.0 * M_PI);

            first_gripper_write_ = false;

            std::cout
                << "\n===== FGR_1 SEEDED ====="
                << "\nCurrent encoder turns = "
                << joint_radians
                << std::endl;
        }
        else
        {
            double moveit_pos = position_commands_[i];

            if (moveit_pos < 0.0)
                moveit_pos = 0.0;

            if (moveit_pos > 0.026)
                moveit_pos = 0.026;

            joint_radians =
                -0.180 +
                (moveit_pos / 0.026) * 0.030;

            static int grip_dbg = 0;

            if (++grip_dbg % 20 == 0)
            {
                std::cout
                  << "\n===== FGR_1 ====="
                  << "\nMoveIt Position = " << moveit_pos
                  << "\nMotor Turns     = " << joint_radians
                  << std::endl;
            }
        }
    }

    if (jname == "FGR_2")
    {
        static int grip2_dbg = 0;

        if (++grip2_dbg % 20 == 0)
        {
            std::cout
              << "\n[FGR_2]"
              << "\nMoveIt Position = "
              << position_commands_[i]
              << " m"
              << std::endl;
        }
    }

    if (jname == "ROT_1")
    {
        joint_radians -= ROT1_OFFSET;
    }

    if (jname == "PITCH_1")
    {
        joint_radians -= 1.464297;
    }

    if (jname == "PITCH_2")
    {
        joint_radians -= 0.113097;
    }


    if (jname == "PITCH_1" || jname == "PITCH_2")
    {
        joint_radians = -joint_radians;
    }

    float motor_turns;

    if (jname == "FGR_1")
    {
        motor_turns = joint_radians;
    }
    else
    {
        motor_turns =
            joint_radians /
            (2.0f * static_cast<float>(M_PI));
    }
    if (jname == "ROT_1")
    {
        std::cout
          << "\n=== ROT1 WRITE ==="
          << "\nMoveIt cmd  = " << position_commands_[i]
          << "\nCAN target  = " << joint_radians
          << "\nMotor turns = " << motor_turns
          << std::endl;
    }

    if (jname == "PITCH_1")
    {
        std::cout
          << "\n=== PITCH_1 WRITE ==="
          << "\nMoveIt cmd  = " << position_commands_[i]
          << "\nCAN target  = " << joint_radians
          << "\nMotor turns = " << motor_turns
          << std::endl;
    }

    struct can_frame frame;
    int command_id = 0x0C; // SET_INPUT_POS

    frame.can_id = (node_ids_[i] << 5) | command_id;
    frame.can_dlc = 8;

    float pos = motor_turns;
    int16_t vel_ff = 0;
    int16_t torque_ff = 0;

    memcpy(&frame.data[0], &pos, 4);
    memcpy(&frame.data[4], &vel_ff, 2);
    memcpy(&frame.data[6], &torque_ff, 2);

    int bytes_sent = ::write(can_socket_, &frame, sizeof(frame));
    if (bytes_sent < 0)
    {
      static int error_counter = 0;
      if (++error_counter % 100 == 0)
      {
        std::cout << "CAN Write Error: " << strerror(errno) << std::endl;
      }
    }
    static int dbg = 0;

    if (++dbg % 20 == 0)
    {
        std::cout << "\n===== COMMANDS =====\n";

        for(size_t i=0;i<ACTIVE_JOINTS;i++)
        {
            std::cout
                << info_.joints[i].name
                << " cmd(rad)=" << position_commands_[i]
                << " turns=" << position_commands_[i]/(2.0*M_PI)
                << std::endl;
        }
    }
    // if (jname == "ROT_1")
    // {
    //   std::cout
    //   << "\n========== ROT_1 =========="
    //   << "\nMoveIt cmd (rad)    = " << position_commands_[i]
    //   << "\nRaw encoder (rad)   = " << raw_position_states_[i]
    //   << "\nCAN target (rad)    = " << joint_radians
    //   << "\nCAN target (turns)  = " << motor_turns
    //   << "\nCAN target (deg)    = " << (joint_radians * 180.0 / M_PI)
    //   << "\n==========================="
    //   << std::endl;
    // }
    
  }

  return hardware_interface::return_type::OK;
}

} // namespace skai_hardware_v2

PLUGINLIB_EXPORT_CLASS(skai_hardware_v2::SKAIHardware, hardware_interface::SystemInterface)