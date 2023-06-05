// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean USE_CONTROLLER = true; //set to false to use set turning speeds
    public static final boolean USE_PID = false; //set to true to use PID

    //set turn speeds
    public static final double TEST_TURN_SPEED = 0.2;
    public static final double TEST_MOVE_SPEED = 0.2;

    public static final int TEST_TURN_MOTOR_ID = 2;
    public static final int TEST_MOVE_MOTOR_ID = 1;

    public static final int TURN_MOTOR_GEAR_RATIO = 1; // UNKNOWN
    public static final int MOVE_MOTOR_GEAR_RATIO = 1; // UNKNOWN

    public static final int MOVE_WHEEL_CIRCUMFERENCE = 1; // UNKNOWN

    public static final double[][] WHEEL_POSITIONS = {
        {1, 1}, // front right - +x, +y
        {-1, 1}, // front left - -x, +y
        {-1, -1}, // back left - -x, -y
        {1, -1} // back right - +x, -y
    };

    //Swerve PID constants
    public static final double TURN_P = 0.02;
    public static final double TURN_I = 0.001;
    public static final double TURN_D = 0.0;
    public static final double TURN_IZ = 0;
    public static final double TURN_FF = 0.00156;

    public static final double TURN_MAX_OUTPUT = 1.0;
    public static final double TURN_MIN_OUTPUT = -1.0;

    public static final double TURN_MAX_VELVELOCITY = 2000;
    public static final double TURN_MA0_XACCELERATION= 1500;

    public static final double KS_TURN_VOLTS = 0.0;
    public static final double KV_TURN_VOLT_SECONDS_PER_ROTATION = 0.0;

    public static final double K_TURN_ENCODER_COUNTS_PER_ROTATION = 8192.0;
    public static final double K_TURN_ENCODER_ROTATIONS_PER_PULSE =
        1 / K_TURN_ENCODER_COUNTS_PER_ROTATION;
    public static final double K_TURN_MAX_RPM = 5600; //under load

    public static final double K_TURN_TOLERANCE = 30;
    public static final double K_TURN_TOLERANCE_PER_SECOND = 10.0;

    public static final double ROBOT_LENGTH = 1; // UNKNOWN
    public static final double ROBOT_WIDTH = 1; // UNKNOWN
}
