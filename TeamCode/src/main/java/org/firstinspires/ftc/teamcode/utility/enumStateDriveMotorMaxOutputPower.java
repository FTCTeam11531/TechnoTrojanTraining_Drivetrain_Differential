package org.firstinspires.ftc.teamcode.utility;

/**
 * <h2>Enum: Drive Motor(s) Max Output</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Enumeration for the state of the drive motor(s) max output
 * </p>
 */
public enum enumStateDriveMotorMaxOutputPower {
    High {

        // Setting for Next State
        @Override
        public enumStateDriveMotorMaxOutputPower nextState() { return Medium; }

        // Output Power Value
        @Override
        public double getValue() { return utilRobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_HIGH; }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() { return "High"; }
    },
    Medium {

        // Setting for Next State
        @Override
        public enumStateDriveMotorMaxOutputPower nextState() { return Low; }

        // Output Power Value
        @Override
        public double getValue() { return utilRobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_MED; }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() { return "Medium"; }
    },
    Low {

        // Setting for Next State
        @Override
        public enumStateDriveMotorMaxOutputPower nextState() { return Snail; }

        // Output Power Value
        @Override
        public double getValue() { return utilRobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_LOW; }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() { return "Low"; }
    },
    Snail {

        // Setting for Next State
        @Override
        public enumStateDriveMotorMaxOutputPower nextState() { return High; }

        // Output Power Value
        @Override
        public double getValue() { return utilRobotConstants.Drivetrain.MOTOR_OUTPUT_POWER_SNAIL; }

        // Output Power Value Label (Text)
        @Override
        public String getLabel() { return "Snail"; }
    };

    /**
     * <h2>Enum Method: nextState</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Advance to the next state in the Drive Motor Output Power enumeration
     * </p>
     * @return StateDrivetrainMode - sets the next state for the enumeration
     */
    public abstract enumStateDriveMotorMaxOutputPower nextState();

    /**
     * <h2>Enum Method: getValue</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the value for the current state in the drive motor max output power enumeration
     * </p>
     * @return double - The value for the state
     */

    public abstract double getValue();

    /**
     * <h2>Enum Method: getLabel</h2>
     * <hr>
     * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
     * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
     * <hr>
     * <p>
     * Get the label for the current state in the drive motor max output power enumeration
     * </p>
     * @return String - The string label for the state
     */
    public abstract String getLabel();

}
