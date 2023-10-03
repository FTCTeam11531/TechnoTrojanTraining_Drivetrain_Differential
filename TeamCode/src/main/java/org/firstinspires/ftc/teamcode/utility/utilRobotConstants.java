package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class utilRobotConstants {

    // General 'Global' Constants
    // ---------------------------------------

    // ---------------------------------------
    // Conversion Factor(s)
    // ---------------------------------------
    // Conversion Millimeters and Inches
    // - - Divide Millimeters by Conversion to get Inches
    // - - Multiply Inches by Conversion to get mm
    public static final double CONVERSION_FACTOR_MM_INCH = 25.4;

    // Conversion Radians and Degrees
    // - - Divide Degrees by Conversion to get Radians
    // - - Multiply Radians by Conversion to get Degrees
    public static final double CONVERSION_FACTOR_RADIAN_DEGREE = 57.2957795;

    /**
     *  <h2>Robot Constant Values - About The Robot</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines 'About the Team' robot constant settings and properties.
     * </p>
     * <li>Program Team</li>
     * <li>Development Season</li>
     * <br>
     */
    public static final class About {
        // Document Comments and any other 'About the Program' reference
        public static final String COMMENT_AUTHOR_NAME = "FTC Team 11531 - Green Team";
        public static final String COMMENT_SEASON_PERIOD = "2023 / 2024";
    }

    public static final class Configuration {
        // Hardware Configuration from Control Hub and/or Expansion Hub
        // ------------------------------------------------------------

        // Drivetrain
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT = "left_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_LEFT_BACK = "left_back_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT = "right_front_drive";
        public static final String LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK = "right_back_drive";

        public static final String LABEL_CONTROLHUB_IMU = "imu_ch";
        public static final String LABEL_EXPANSIONHUB_IMU = "imu_eh";

        // Lighting
        public static final String LABEL_CONTROLLER_LIGHTING = "lighting_control";

    }

    /**
     *  <h2>Robot Constant Values - Common</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Defines common constants used throughout.
     * Can include timeout values, math functions, etc.
     * </p>
     * <li>Command Sleep Timeout(s)</li>
     * <li>Math Constants</li>
     * <br>
     */
    public static final class CommonSettings {
        public static final int SLEEP_TIMER_MILLISECONDS_DEFAULT = 100;

        // Initialization Display
        public static final String INIT_SETTING_DISPLAY_MODE_AUTONOMOUS = "autonomous";
        public static final String INIT_SETTING_DISPLAY_MODE_TELEOP = "teleop";

        // Transition from Autonomous to Teleop - Global Static variables (not constants)
        public static double IMU_TRANSITION_ADJUSTMENT = 0;

        // Get Properties
        public static double getImuTransitionAdjustment() {
            return IMU_TRANSITION_ADJUSTMENT; }

        // Set Properties
        public static void setImuTransitionAdjustment(double inHeading) {
            IMU_TRANSITION_ADJUSTMENT = inHeading; }

    }

    /**
     *  <h2>Robot Constant Values - Drivetrain</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Drivetrain constants
     * </p>
     * <li>Drivetrain Constants</li>
     * <br>
     */
    public static final class Drivetrain {

        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_DRIVE_MOTOR = "goBuilda Yellow Jacket 5203-2402-0019";
        public static final String COMMENT_DRIVE_WHEEL_TYPE = "Differential";

        // List Item - Drivetrain Mode Type(s)
        public static final String LIST_MODE_TYPE_DRIVETRAIN_FIELDCENTRIC = "Field Centric";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC = "Robot Centric";

        public static final String LIST_MODE_TYPE_DRIVETRAIN_ARCADE_MODE = "Arcade Mode";
        public static final String LIST_MODE_TYPE_DRIVETRAIN_TANK_MODE = "Tank Mode";

        // Physical Robot Settings
        public static final double DRIVETRAIN_TRACK_WIDTH_INCHES = 10; // !! need to tune !!
        public static final double WHEEL_DIAMETER_MILLIMETER = 96; // !! need to tune !!
        public static final double WHEEL_DIAMETER_INCHES = convertMillimetersToInches(WHEEL_DIAMETER_MILLIMETER);

        public static final double WHEEL_RADIUS_MILLIMETER = WHEEL_DIAMETER_MILLIMETER / 2;
        public static final double WHEEL_RADIUS_INCHES = convertMillimetersToInches(WHEEL_RADIUS_MILLIMETER);

        public static final double EXTERNAL_GEAR_RATIO = 1.0; // 1 = No External Gearing // !! need to tune !!

        // Motor/Encoder Configuration Settings
        public static final double MOTOR_MAX_RPM = 312; // Set from Vendor Specs
        public static final double ENCODER_TICKS_PER_REV = 537.7; // Set from Vendor Specs

        public static final double ENCODER_TICKS_PER_INCH = (ENCODER_TICKS_PER_REV * EXTERNAL_GEAR_RATIO) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

        // Motor Encoder PIDF Settings
        public static final double MOTOR_VELOCITY_P = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_I = 0; // !! need to tune !!
        public static final double MOTOR_VELOCITY_D = 0; // !! need to tune !!
        public static final PIDFCoefficients DRIVE_MOTOR_VELOCITY_PIDF = new PIDFCoefficients(
                MOTOR_VELOCITY_P,
                MOTOR_VELOCITY_I,
                MOTOR_VELOCITY_D,
                getMotorVelocityF(MOTOR_MAX_RPM / 60 * ENCODER_TICKS_PER_REV));

        // Proportional control coefficient (or GAIN) for "heading control"
        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
        // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
        public static final double HEADING_P_TURN_GAIN = 0.02;   // Larger is more responsive, but also less stable
        public static final double HEADING_P_DRIVE_GAIN = 0.03;  // Larger is more responsive, but also less stable

        // These constants define the desired driving/control characteristics
        // They can/should be tweaked to suit the specific robot drive train.
        public static final double AUTO_DRIVE_SPEED = 0;         // Max driving speed for better distance accuracy.
        public static final double AUTO_TURN_SPEED = 0;          // Max Turn speed to limit turn rate
        public static final double AUTO_HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

        // Motor Encoder Feed Forward Configuration(s)
        public static final double FEED_FORWARD_KV = 1.0 / getRpmToVelocity(MOTOR_MAX_RPM); // volts * seconds / distance
        public static final double FEED_FORWARD_KA = 0; // volts * seconds^2 / distance
        public static final double FEED_FORWARD_KS = 0; // volts

        // Motor Limit 'Max' Settings
        public static final double LIMIT_MAX_VELOCITY = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ACCELERATION = 30; // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_VELOCITY = Math.toRadians(60); // !! need to tune !!
        public static final double LIMIT_MAX_ANGLE_ACCELERATION = Math.toRadians(60); // !! need to tune !!


        // Motor Output Setting(s)
        public static final double MOTOR_LATERAL_MOVEMENT_STRAFING_CORRECTION = 1.1;
        public static final double MOTOR_OUTPUT_POWER_MAX = 1;

        public static final double MOTOR_OUTPUT_POWER_HIGH = 1;
        public static final double MOTOR_OUTPUT_POWER_MED = 0.75;
        public static final double MOTOR_OUTPUT_POWER_LOW = 0.50;
        public static final double MOTOR_OUTPUT_POWER_SNAIL = 0.15;

        /**
         * <h2>Convert: Encoder Ticks to Inches</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert Encoder Ticks/Pulses/Cycles to Inches
         * </p>
         * <br>
         *
         * @param inTicks
         *
         * @return double (Inches)
         */
        public static double getEncoderTicksToInches(double inTicks) {
            double outInches = 0;

            outInches = WHEEL_RADIUS_INCHES * 2 * Math.PI * EXTERNAL_GEAR_RATIO * inTicks / ENCODER_TICKS_PER_REV;

            return outInches;
        }

        /**
         * <h2>Convert: RPM to Velocity</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Conversion Method: Convert RPM to Velocity
         * </p>
         * <br>
         *
         * @param inRpm
         *
         * @return double (Velocity)
         */
        public static double getRpmToVelocity(double inRpm) {
            double outVelocity = 0;

            outVelocity = inRpm * EXTERNAL_GEAR_RATIO * Math.PI * WHEEL_RADIUS_INCHES / 60.0;

            return outVelocity;
        }

        /**
         * <h2>Get Motor Velocity F Value for PIDF</h2>
         * <hr>
         * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
         * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
         * <hr>
         * <p>
         * Calculating PIDF Values from Maximum Velocity
         * Once you have your maximum velocity (maxV), you can calculate the velocity PIDF values. Your F value is calculated like this: F = 32767maxV. So if your max velocity is 2600 ticks per second (reasonable for a mechanism that uses the HD Hex Motor), then your F value should be about 12.6.
         * </p>
         * <br>
         * Source: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
         *
         * @param inTicksPerSecond
         *
         * @return double (Velocity F - for PIDF)
         */
        public static double getMotorVelocityF(double inTicksPerSecond) {
            double outVelocityF = 0;

            outVelocityF = 32767 / inTicksPerSecond;

            return outVelocityF;
        }

    }

    /**
     *  <h2>Robot Constant Values - Lighting</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Anything and everything related to Lighting constants
     * </p>
     * <li>Lighting Constants</li>
     *
     * CP1 = Green
     * CP2 = Blue
     *
     * CP1_2_COLOR_GRADIENT
     * BEATS_PER_MINUTE_FOREST_PALETTE
     * BEATS_PER_MINUTE_OCEAN_PALETTE
     * BEATS_PER_MINUTE_PARTY_PALETTE
     * SHOT_WHITE
     * CP2_SHOT
     * CP2_BREATH_SLOW
     * CP2_HEARTBEAT_SLOW
     * CP2_LARSON_SCANNER
     *
     * CP2_LIGHT_CHASE
     *
     * CONFETTI
     * RAINBOW_LAVA_PALETTE
     * RAINBOW_PARTY_PALETTE
     * RAINBOW_WITH_GLITTER
     * RAINBOW_FOREST_PALETTE
     * RAINBOW_OCEAN_PALETTE
     *
     * CP1_2_SPARKLE_2_ON_1
     * CP1_2_SINELON
     * CP1_2_BEATS_PER_MINUTE
     *
     * CP1_2_TWINKLES **
     * TWINKLES_FOREST_PALETTE **
     *
     * CP1_2_NO_BLENDING
     *
     * CP1_STROBE
     *
     * STROBE_BLUE
     *
     * FIRE_LARGE
     *
     *
     *
     * HEARTBEAT_BLUE
     * <br>
     */
    public static final class Lighting {
        // Hardware Settings based on the following [comments]:
        public static final String COMMENT_LIGHTING_CONTROLLER = "REV-11-1105 Blinkin LED Driver";

        // Patterns to Avoid
        public static final String LIGHT_PATTERN_AVOID_KEYWORD_TWINKLES = "Twinkles";

        // Default Light Pattern
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT_TELEOP = LIGHT_PATTERN_DEFAULT;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_DEFAULT_AUTONOMOUS = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_PREGAME_OPTION_CONFIG = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

        // System Initialize Light Patter/state
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_LIGHTING = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST; // CP1_STROBE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN = RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST; // STROBE_WHITE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_VISION = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW; // CP2_STROBE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_LINEARSLIDE = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE; // STROBE_GOLD
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_CLAW = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_TARGETING = RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_SOUND = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE; // STROBE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_SYSTEM_INIT_COMPLETE = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE;

        // Light Pattern(s) for Autonomous
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE = RevBlinkinLedDriver.BlinkinPattern.GOLD;    // Peach
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO = RevBlinkinLedDriver.BlinkinPattern.AQUA;    // Trojan
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE = RevBlinkinLedDriver.BlinkinPattern.GREEN; // Cards
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID = RevBlinkinLedDriver.BlinkinPattern.RED;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER; // RAINBOW_WITH_GLITTER
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE; // RAINBOW_PARTY_PALETTE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE; // RAINBOW_RAINBOW_PALETTE
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_INVALID = LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID;

        // Light Patterns for Robot State(s) - Linear Slide
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_HIGH = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_MED = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_LOW = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_GOAL_GROUND = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_LIMIT_MAX = RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_LINEAR_SLIDE_DRIFT = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;

        // Light Patterns for Robot State(s) - Claw
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_OPEN = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE; // LIGHT_CHASE_RED
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_CLAW_CLAMP_CLOSED = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE; // LIGHT_CHASE_BLUE

        // Light Patterns for Targeting
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_TARGET_JUNCTION_ZONE_ACTIVE = RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT;
        public static final RevBlinkinLedDriver.BlinkinPattern LIGHT_PATTERN_TARGET_JUNCTION_ZONE_INACTIVE = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;

    }

    // --------------------------------------------
    // General Method(s)
    // --------------------------------------------


    /**
     * <h2>utilRobotConstants Method: convertMillimetersToInches</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from millimeters to inches
     * </p>
     * @param inValueInMillimeters - value to be converted (in millimeters)
     *
     * @return double - value converted to inches
     */
    public static double convertMillimetersToInches(double inValueInMillimeters) {
        return inValueInMillimeters / CONVERSION_FACTOR_MM_INCH;
    }

    /**
     * <h2>RobotConstants Method: convertInchesToMillimeters</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Convert a value from inches to millimeters
     * </p>
     * @param inValueInInches - value to be converted (in inches)
     *
     * @return double - value converted to millimeters
     */
    public static double convertInchesToMillimeters(double inValueInInches) {
        return inValueInInches * CONVERSION_FACTOR_MM_INCH;
    }

    public static double convertRadiansToDegrees(double inValueInRadians) {
        return inValueInRadians * CONVERSION_FACTOR_RADIAN_DEGREE;
    }

    public static double convertDegreesToRadians(double inValueInDegrees) {
        return inValueInDegrees / CONVERSION_FACTOR_RADIAN_DEGREE;
    }
}
