/**
 * @file aeb_controller_node.cpp
 * @brief AEB Controller — thin ROS2 wrapper over the embedded C library.
 *
 * This node contains NO AEB logic.  All decisions are delegated to the
 * embedded C modules (aeb_main.c / aeb_fsm.c / aeb_pid.c …) compiled
 * directly into this executable, exactly as they would run on the MCU.
 *
 * CAN topology (mirrors aeb_system.dbc):
 *
 *   Subscribes  (inputs  → C library):
 *     /can/radar_target   [AEB_RadarTarget]  20 ms  — Radar_ECU
 *     /can/ego_vehicle    [AEB_EgoVehicle]   10 ms  — Vehicle_ECU
 *
 *   Publishes   (outputs ← C library):
 *     /can/brake_cmd      [AEB_BrakeCmd]     10 ms  — AEB_Controller
 *     /can/fsm_state      [AEB_FSMState]     50 ms  — AEB_Controller
 *     /can/alert          [AEB_Alert]        event  — AEB_Controller
 */

#include <rclcpp/rclcpp.hpp>

/* ── Generated ROS2 message headers ──────────────────────────────────────── */
#include "aeb_gazebo/msg/aeb_radar_target.hpp"
#include "aeb_gazebo/msg/aeb_ego_vehicle.hpp"
#include "aeb_gazebo/msg/aeb_brake_cmd.hpp"
#include "aeb_gazebo/msg/aeb_fsm_state.hpp"
#include "aeb_gazebo/msg/aeb_alert.hpp"
// Note: rosidl generates snake_case header names from CamelCase msg names:
//   AebRadarTarget -> aeb_radar_target.hpp  etc.

/* ── Embedded C library (compiled alongside this file) ───────────────────── */
extern "C" {
#include "aeb_main.h"
#include "aeb_types.h"
}

#include <chrono>
#include <cstdint>
#include <string>

using namespace std::chrono_literals;

/* ── CAN IDs (matches DBC) ───────────────────────────────────────────────── */
static constexpr uint32_t CAN_ID_RADAR_TARGET = 0x120U;
static constexpr uint32_t CAN_ID_EGO_VEHICLE  = 0x100U;
static constexpr uint32_t CAN_ID_BRAKE_CMD    = 0x080U;
static constexpr uint32_t CAN_ID_FSM_STATE    = 0x200U;
static constexpr uint32_t CAN_ID_ALERT        = 0x300U;

/* ── BrakeMode mapping from FSM state (matches DBC VAL_) ─────────────────── */
static uint8_t fsm_state_to_brake_mode(AEB_State_t st)
{
    switch (st) {
        case AEB_WARNING:    return 1U;
        case AEB_BRAKE_L1:   return 2U;
        case AEB_BRAKE_L2:   return 3U;
        case AEB_BRAKE_L3:   return 4U;
        case AEB_POST_BRAKE: return 5U;
        default:             return 0U;
    }
}

/* ── AlertLevel mapping (matches DBC VAL_) ───────────────────────────────── */
static uint8_t fsm_state_to_alert_level(AEB_State_t st)
{
    switch (st) {
        case AEB_WARNING:  return 1U;
        case AEB_BRAKE_L1: return 2U;
        case AEB_BRAKE_L2: return 3U;
        case AEB_BRAKE_L3: return 3U;
        default:           return 0U;
    }
}

/* ── TTC threshold for current state (matches aeb_config.h) ──────────────── */
static float fsm_state_to_ttc_threshold(AEB_State_t st)
{
    switch (st) {
        case AEB_WARNING:  return 4.0F;
        case AEB_BRAKE_L1: return 3.0F;
        case AEB_BRAKE_L2: return 2.2F;
        case AEB_BRAKE_L3: return 1.8F;
        default:           return 0.0F;
    }
}

/* ── State name string (matches DBC VAL_) ────────────────────────────────── */
static std::string fsm_state_name(AEB_State_t st)
{
    switch (st) {
        case AEB_OFF:        return "OFF";
        case AEB_STANDBY:    return "STANDBY";
        case AEB_WARNING:    return "WARNING";
        case AEB_BRAKE_L1:   return "BRAKE_L1";
        case AEB_BRAKE_L2:   return "BRAKE_L2";
        case AEB_BRAKE_L3:   return "BRAKE_L3";
        case AEB_POST_BRAKE: return "POST_BRAKE";
        default:             return "UNKNOWN";
    }
}

/* ── 4-bit XOR CRC over three payload bytes ─────────────────────────────── */
static uint8_t compute_crc4(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint8_t crc = (b0 ^ b1 ^ b2) & 0x0FU;
    return crc;
}

/* ════════════════════════════════════════════════════════════════════════════
 * AEBControllerNode
 * ══════════════════════════════════════════════════════════════════════════ */
class AEBControllerNode : public rclcpp::Node
{
public:
    AEBControllerNode() : Node("aeb_controller_node"),
        alive_brake_(0U), alive_fsm_(0U),
        prev_state_(AEB_STANDBY),
        radar_ready_(false), ego_ready_(false),
        v_target_cached_(0.0F)
    {
        /* ── Initialise embedded C library ── */
        aeb_init();

        /* ── Subscribers ── */
        radar_sub_ = create_subscription<aeb_gazebo::msg::AebRadarTarget>(
            "/can/radar_target", 10,
            [this](const aeb_gazebo::msg::AebRadarTarget::SharedPtr msg) {
                radar_ = *msg;
                /* Compute v_target while ego speed is synchronous with this
                 * radar message. Caching here prevents the ROC fault that
                 * occurs when v_ego_raw and radar_.relative_speed come from
                 * different 10ms/20ms time slots. */
                const float v_ego_at_radar = static_cast<float>(ego_.vehicle_speed);
                const float vt = v_ego_at_radar - static_cast<float>(msg->relative_speed);
                v_target_cached_ = (vt > 0.0F) ? vt : 0.0F;
                radar_ready_ = true;
            });

        ego_sub_ = create_subscription<aeb_gazebo::msg::AebEgoVehicle>(
            "/can/ego_vehicle", 10,
            [this](const aeb_gazebo::msg::AebEgoVehicle::SharedPtr msg) {
                ego_ = *msg;
                ego_ready_ = true;
            });

        /* ── Publishers ── */
        brake_pub_ = create_publisher<aeb_gazebo::msg::AebBrakeCmd>(
            "/can/brake_cmd", 10);

        fsm_pub_ = create_publisher<aeb_gazebo::msg::AebFsmState>(
            "/can/fsm_state", 10);

        alert_pub_ = create_publisher<aeb_gazebo::msg::AebAlert>(
            "/can/alert", 10);

        /* ── 10 ms control loop (matches embedded cycle) ── */
        ctrl_timer_ = create_wall_timer(10ms,
            std::bind(&AEBControllerNode::control_loop, this));

        /* ── 50 ms FSM state broadcast ── */
        fsm_timer_ = create_wall_timer(50ms,
            std::bind(&AEBControllerNode::publish_fsm_state, this));

        RCLCPP_INFO(get_logger(), "AEB Controller Node started — C library active");
    }

private:
    /* ── Latest received CAN frames ── */
    aeb_gazebo::msg::AebRadarTarget radar_{};
    aeb_gazebo::msg::AebEgoVehicle  ego_{};

    /* ── Sensors-ready guard: prevents startup fault from zero-initialised
     *    radar_.target_distance (0 < RANGE_MIN = 0.5 m → C fault latch) ── */
    bool radar_ready_;
    bool ego_ready_;

    /* ── v_target cached at radar-callback time (avoids 10ms/20ms async) ── */
    float32_t v_target_cached_;

    /* ── Rolling counters ── */
    uint8_t alive_brake_;
    uint8_t alive_fsm_;

    /* ── Previous alert state for event-driven publishing ── */
    AEB_State_t prev_state_;

    /* ── ROS2 handles ── */
    rclcpp::Subscription<aeb_gazebo::msg::AebRadarTarget>::SharedPtr radar_sub_;
    rclcpp::Subscription<aeb_gazebo::msg::AebEgoVehicle>::SharedPtr  ego_sub_;
    rclcpp::Publisher<aeb_gazebo::msg::AebBrakeCmd>::SharedPtr  brake_pub_;
    rclcpp::Publisher<aeb_gazebo::msg::AebFsmState>::SharedPtr  fsm_pub_;
    rclcpp::Publisher<aeb_gazebo::msg::AebAlert>::SharedPtr     alert_pub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;

    /* ── 10 ms cycle: call C library, publish brake command ── */
    void control_loop()
    {
        /* Guard: wait until at least one message from each sensor has arrived.
         * Without this, radar_.target_distance = 0 < RANGE_MIN (0.5 m) and
         * the C perception module latches a fault after 3 cycles. */
        if (!radar_ready_ || !ego_ready_) {
            return;
        }

        /* Decode signals from latest CAN frames.
         * v_target_cached_ was computed in the radar callback where ego speed
         * and relative_speed are synchronous, preventing ROC false-faults.
         * actual_decel is passed as 0 — the Gazebo odom derivative is too
         * noisy for closed-loop PID feedback; open-loop is more stable here. */
        const float32_t distance_raw   = static_cast<float32_t>(radar_.target_distance);
        const float32_t v_ego_raw      = static_cast<float32_t>(ego_.vehicle_speed);
        const float32_t v_target_raw   = v_target_cached_;
        const uint8_t   brake_pedal    = ego_.brake_pedal ? 1U : 0U;
        const float32_t steering_angle = static_cast<float32_t>(ego_.steering_angle);
        const float32_t actual_decel   = 0.0F;   /* open-loop in simulation */

        /* ── Call embedded C algorithm — no logic lives here ── */
        aeb_cycle_10ms(distance_raw, v_ego_raw, v_target_raw,
                       brake_pedal, steering_angle, actual_decel);

        const float32_t  brake_pct = aeb_get_brake_cmd();   /* 0-100 % */
        const AEB_State_t state    = aeb_get_state();

        /* ── Build AebBrakeCmd (0x080) ── */
        aeb_gazebo::msg::AebBrakeCmd bc;
        bc.can_id         = CAN_ID_BRAKE_CMD;
        bc.alive_counter  = alive_brake_;
        bc.brake_request  = (brake_pct > 0.0F);
        bc.brake_pressure = brake_pct * 0.1F;   /* map 0-100 % → 0-10 bar */
        bc.brake_mode     = fsm_state_to_brake_mode(state);

        /* CRC over first 3 payload bytes: [alive|request|mode] */
        const uint8_t b0 = static_cast<uint8_t>(alive_brake_ & 0x0FU);
        const uint8_t b1 = bc.brake_request ? 1U : 0U;
        const uint8_t b2 = bc.brake_mode;
        bc.crc = compute_crc4(b0, b1, b2);

        brake_pub_->publish(bc);

        /* Advance alive counter 0→15→0 */
        alive_brake_ = static_cast<uint8_t>((alive_brake_ + 1U) & 0x0FU);

        /* ── Publish AEB_Alert (0x300) only on FSM state change ── */
        if (state != prev_state_) {
            const std::string from = fsm_state_name(prev_state_);  /* capture before update */
            publish_alert(state);
            prev_state_ = state;

            RCLCPP_INFO(get_logger(), "FSM: %s -> %s  brake=%.0f%%",
                from.c_str(),
                fsm_state_name(state).c_str(),
                static_cast<double>(brake_pct));
        }
    }

    /* ── 50 ms broadcast of FSM status frame (0x200) ── */
    void publish_fsm_state()
    {
        const AEB_State_t state = aeb_get_state();
        const float32_t   brake = aeb_get_brake_cmd();

        aeb_gazebo::msg::AebFsmState fs;
        fs.can_id        = CAN_ID_FSM_STATE;
        fs.fsm_state     = static_cast<uint8_t>(state);
        fs.fsm_state_name = fsm_state_name(state);
        fs.alert_level   = fsm_state_to_alert_level(state);
        fs.brake_active  = (brake > 0.0F);
        fs.ttc_threshold = fsm_state_to_ttc_threshold(state);

        fsm_pub_->publish(fs);

        alive_fsm_ = static_cast<uint8_t>((alive_fsm_ + 1U) & 0x0FU);
    }

    /* ── Event-driven alert frame (0x300) ── */
    void publish_alert(AEB_State_t state)
    {
        const bool visual  = (state == AEB_WARNING  || state == AEB_BRAKE_L1 ||
                               state == AEB_BRAKE_L2 || state == AEB_BRAKE_L3 ||
                               state == AEB_POST_BRAKE);
        const bool audible = (state == AEB_WARNING  || state == AEB_BRAKE_L1 ||
                               state == AEB_BRAKE_L2 || state == AEB_BRAKE_L3);

        uint8_t alert_type = 0U;
        if (visual && audible) { alert_type = 3U; }
        else if (audible)      { alert_type = 2U; }
        else if (visual)       { alert_type = 1U; }

        /* BuzzerCmd: ramp up with threat level */
        uint8_t buzzer = 0U;
        switch (state) {
            case AEB_WARNING:  buzzer = 1U; break;  /* SingleBeep  */
            case AEB_BRAKE_L1: buzzer = 2U; break;  /* DoubleBeep  */
            case AEB_BRAKE_L2: buzzer = 4U; break;  /* FastPulse   */
            case AEB_BRAKE_L3: buzzer = 3U; break;  /* Continuous  */
            default:           buzzer = 0U; break;
        }

        aeb_gazebo::msg::AebAlert al;
        al.can_id        = CAN_ID_ALERT;
        al.alert_type    = alert_type;
        al.alert_active  = visual || audible;
        al.visual_active = visual;
        al.audible_active = audible;
        al.buzzer_cmd    = buzzer;

        alert_pub_->publish(al);
    }
};

/* ── Entry point ─────────────────────────────────────────────────────────── */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AEBControllerNode>());
    rclcpp::shutdown();
    return 0;
}
