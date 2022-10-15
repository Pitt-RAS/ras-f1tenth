#ifndef STATES_H_
#define STATES_H_

#include <string>
#include <vector>
#include <memory>

namespace States {
    /**
     * @brief Mux states that control the drive topic
     *
     */
    typedef enum StateId
    {
        OFF=0,
        MANUAL,
        WALLFOLLOWING,
        GAPFOLLOWING,
        SAFETYBRAKE,

        _SIZE_
    };

    namespace Off
    {
        const std::string NAME = "OFF";
        constexpr char INPUT_CHAR = 'b';
    } // namespace Off

    namespace Manual
    {
        const std::string NAME = "MANUAL";
        const std::string DRIVE_TOPIC = "/manual_drive";
        constexpr char INPUT_CHAR = 'm';
    } // namespace Manual

    namespace Autonmous
    {
        const std::string NAME = "AUTONOMOUS";
        const std::string DRIVE_TOPIC = "/auto_drive";
    } // namespace Autonmous

    namespace WallFollowing
    {
        const std::string NAME = "WALL-FOLLOWING";
        const std::string DRIVE_TOPIC = "/wall_following";
        constexpr char INPUT_CHAR = 'f';
    } // namespace WallFollowing

    namespace GapFollowing
    {
        const std::string NAME = "GAP-FOLLOWING";
        const std::string DRIVE_TOPIC = "/gap_following";
        constexpr char INPUT_CHAR = 'g';
    } // namespace GapFollowing

    namespace SafetyBrake
    {
        const std::string NAME = "SAFETY-BRAKE";
        const std::string DRIVE_TOPIC = "/brake";
        constexpr char INPUT_CHAR = 0x01;
    } // namespace SafetyBrake

} // namespace States

#endif