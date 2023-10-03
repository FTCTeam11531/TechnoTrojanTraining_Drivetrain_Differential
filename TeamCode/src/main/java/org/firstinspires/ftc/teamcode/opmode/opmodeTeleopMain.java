package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.sysDrivetrainDifferential;
import org.firstinspires.ftc.teamcode.utility.enumStateDriveMotorMaxOutputPower;
import org.firstinspires.ftc.teamcode.utility.enumStateDrivetrainMode;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

@TeleOp(name="TTT: Drivetrain Differential", group="training")
//@Disabled
public class opmodeTeleopMain extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion

    // -- Lighting System
//    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
    sysDrivetrainDifferential sysDrivetrain = new sysDrivetrainDifferential(this);

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

//        sysLighting.init();
//        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

        sysDrivetrain.init();
//        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        // ------------------------------------------------------------
        // Configure drivetrain for Teleop Mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        double inputDrive, inputTurn, inputLeftTrack, inputRightTrack;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

//        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);
        waitForStart();

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        // Starting LED mode
//        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);

        // Reset runtime clock
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) {

            // Update the Transition Adjustment Value for the IMU
            utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

            return;
        }

        // ------------------------------------------------------------
        // Command Loop: run until the end of the match (driver presses STOP)
        // ------------------------------------------------------------
        while (opModeIsActive()) {

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_y): Drive / Left Track
            // -- -- Axis (right_stick_x): Turn
            // -- -- Axis (right_stick_y): Right Track
            // -- -- X: Set Output Speed to High
            // -- -- Y: Set Output Speed to Med
            // -- -- A: Set Output Speed to Low
            // -- -- B: Set Output Speed to Snail
            //
            // -- Drive Mode Settings
            // -- -- D-Pad Left: Arcade Mode
            // -- -- D-Pad Left: Tank Mode
            //
            // -- Override Settings
            // -- -- D-Pad Up + X: Reset Heading Override (and Raw)
            //
            // ------------------------------------------------------------
            // Gamepad2 = Co-Driver
            // ------------------------------------------------------------
            // NA

            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors

            // Arcade Assignments
            inputDrive =  -(gamepad1.left_stick_y);
            inputTurn = gamepad1.right_stick_x;

            // Tank Assignments
            inputLeftTrack =  -(gamepad1.left_stick_y);
            inputRightTrack = -(gamepad1.right_stick_y);

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
            if(sysDrivetrain.getLabelDrivetrainMode().equals(utilRobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_TANK_MODE)) {
                // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
                sysDrivetrain.driveDifferentialTank(inputLeftTrack, inputRightTrack, sysDrivetrain.getValueDrivetrainOutputPower());
            }
            else {
                // Send gamepad input for drivetrain to driveMecanumFieldCentric method in the drivetrain system class
                sysDrivetrain.driveDifferentialArcade(inputDrive, inputTurn, sysDrivetrain.getValueDrivetrainOutputPower());
            }

            // Button Action - Set Output Power Mode to High
            if(gamepad1.x) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.High;
            }

            // Button Action - Set Output Power Mode to Medium
            if(gamepad1.y) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Medium;
            }

            // Button Action - Set Output Power Mode to Low
            if(gamepad1.a) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Low;
            }

            // Button Action - Set Output Power Mode to Snail Mode
            if(gamepad1.b) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Snail;
            }

            // Button Action - Set drive mode to 'Arcade'
            if(gamepad1.dpad_left) {
                sysDrivetrain.stateDrivetrainMode = enumStateDrivetrainMode.Arcade_Mode;
            }

            // Button Action - Set drive mode to 'Tank'
            if(gamepad1.dpad_right) {
                sysDrivetrain.stateDrivetrainMode = enumStateDrivetrainMode.Tank_Mode;
            }

            // ------------------------------------------------------------
            // Override
            // ------------------------------------------------------------
            // Button Action - Reset Heading Override (and Raw)
            if(gamepad1.dpad_up && gamepad1.x) {

                // Reset the Robot Heading (normally done on init of Drivetrain system)
                sysDrivetrain.resetZeroRobotHeading();

                // Cycle Pause
                sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Run Time", runtime.toString());

            // ------------------------------------------------------------
            // - Gamepad telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Gamepad");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Gamepad 1 - [Y] Axial", "%4.2f", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 - [X] Lateral", "%4.2f", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 - [R] Rotation", "%4.2f", gamepad1.right_stick_x);
            telemetry.addData("-", "------------------------------");

            // ------------------------------------------------------------
            // - Drivetrain telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Drivetrain");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Drivetrain Mode", sysDrivetrain.getLabelDrivetrainMode());
            telemetry.addData("Drivetrain Power", sysDrivetrain.getLabelDrivetrainOutputPower());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Power Front left/Right", "%4.2f, %4.2f"
                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
            telemetry.addData("Power Back  left/Right", "%4.2f, %4.2f"
                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Encoder Front left/Right", "%7d, %7d"
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
            telemetry.addData("Encoder Back  left/Right", "%7d, %7d"
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Robot Heading Raw", sysDrivetrain.getRobotHeadingRaw());
            telemetry.addData("Heading Adjustment", utilRobotConstants.CommonSettings.getImuTransitionAdjustment());
            telemetry.addData("Robot Heading (Adjusted)", sysDrivetrain.getRobotHeadingAdj());

            // ------------------------------------------------------------
            // - Lighting telemetry
            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Lighting");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------

            // Input assignment to 'pause' telemetry update(s)
//            if (!gamepad1.dpad_right) {
                telemetry.update();
//            }

            // Pace this loop so commands move at a reasonable speed.
            sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }

        // ------------------------------------------------------------
        // Closing Teleop
        // ------------------------------------------------------------
        // Update the Transition Adjustment Value for the IMU
        utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

    }
}
