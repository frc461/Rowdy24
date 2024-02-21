package frc.robot.constants;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static double STICK_DEADBAND;
    public static double TRIGGER_DEADBAND;
    public static double MINIMUM_ANGLE;
    public static double MAXIMUM_ANGLE;

    public static final class Auto {
        public static double AUTO_DRIVE_P;
        public static double AUTO_DRIVE_I;
        public static double AUTO_DRIVE_D;
        public static double AUTO_ANGLE_P;
        public static double AUTO_ANGLE_I;
        public static double AUTO_ANGLE_D;
    }

    public static final class Angler {
        public static int ANGLER_ID;
        public static double ANGLER_P;
        public static double ANGLER_I;
        public static double ANGLER_D;
        public static int ANGLER_CURRENT_LIMIT;
        public static boolean ANGLER_INVERT;
        public static int ANGLER_LOWER_LIMIT_SWITCH_PORT;
        public static int ANGLER_UPPER_LIMIT_SWITCH_PORT;
        public static double ANGLER_LOWER_LIMIT;
        public static double ANGLER_UPPER_LIMIT;
        public static double UPPER_BOUND_LIMIT;
        public static double UPPER_BOUND_COEFFICIENT;
        public static double UPPER_BOUND_SERIES;
        public static double TIGHT_BOUND_COEFFICIENT;
        public static double TIGHT_BOUND_SERIES;
    }

    public static final class Elevator {
        public static int ELEVATOR_ID;
        public static int ELEVATOR_CURRENT_LIMIT;
        public static int ELEVATOR_LIMIT_SWITCH;
        public static int ELEVATOR_SERVO_PORT;
        public static double ELEVATOR_SERVO_CLAMPED_POS;
        public static double ELEVATOR_SERVO_UNCLAMPED_POS;
        public static boolean ELEVATOR_INVERT;
        public static double ELEVATOR_P;
        public static double ELEVATOR_I;
        public static double ELEVATOR_D;
        public static double ELEVATOR_LOWER_LIMIT;
        public static double ELEVATOR_UPPER_LIMIT;
        public static double ELEVATOR_AMP;
        public static double ELEVATOR_STOW;
    }

    public static final class IntakeCarriage {
        public static int INTAKE_ID;
        public static int CARRIAGE_ID;
        public static double IDLE_INTAKE_SPEED;
        public static int CARRIAGE_BEAM;
        public static int SHOOTER_BEAM;
        public static int AMP_BEAM;
    }

    public static final class Limelight {
        public static double LIMELIGHT_P;
        public static double LIMELIGHT_I;
        public static double LIMELIGHT_D;
        public static double YAW_OFFSET;
    }

    public static final class Shooter {
        public static int BOTTOM_SHOOTER_ID; //bottom wheel
        public static int TOP_SHOOTER_ID; //top wheel
        public static int SHOOTER_CURRENT_LIMIT;
        public static boolean SHOOTER_INVERT;
        public static double BASE_SHOOTER_SPEED; //baseline shooter speed in rpm
        public static double IDLE_SHOOTER_SPEED;
        public static double SHOOTER_ERROR_TOLERANCE; // +/-tolerance for considering if the shooter is up to speed
        public static double DISTANCE_MULTIPLIER;
        public static double SHOOTER_P;
        public static double SHOOTER_I;
        public static double SHOOTER_D;
        public static double SHOOTER_FF;
        public static int FEEDER_ID;
        public static int FEEDER_CURRENT_LIMIT;
        public static boolean FEEDER_INVERT;
    }

    public static final class Swerve {
        public static double GYRO_OFFSET;
        public static int PIGEON_ID;
        public static boolean INVERT_GYRO; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        public static COTSFalconSwerveConstants CHOSEN_MODULE;

        /* Drivetrain Constants */
        public static double TRACK_WIDTH;
        public static double WHEEL_BASE;
        public static double CENTER_TO_WHEEL;
        public static double WHEEL_CIRCUMFERENCE;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static SwerveDriveKinematics SWERVE_KINEMATICS;

        /* Swerve Voltage Compensation */
        public static double VOLTAGE_COMP;

        /* Module Gear Ratios */
        public static double DRIVE_GEAR_RATIO;
        public static double ANGLE_GEAR_RATIO;
        public static boolean ANGLE_MOTOR_INVERT;
        public static boolean DRIVE_MOTOR_INVERT;

        /* Angle Encoder Invert */
        public static SensorDirectionValue CANCODER_SENSOR_DIRECTION;

        /* Swerve Current Limiting */
        public static int ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        public static int ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
        public static double ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        public static boolean ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        public static int DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        public static int DRIVE_PEAK_SUPPLY_CURRENT_LIMIT;
        public static double DRIVE_PEAK_SUPPLY_CURRENT_DURATION;
        public static boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static double OPEN_LOOP_RAMP;
        public static double CLOSED_LOOP_RAMP;

        /* Angle Motor PID Values */
        public static double ANGLE_P;
        public static double ANGLE_I;
        public static double ANGLE_D;
        public static double ANGLE_F;

        /* Drive Motor PID Values */
        public static double DRIVE_P;
        public static double DRIVE_I;
        public static double DRIVE_D;
        public static double DRIVE_F;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static double DRIVE_S;
        public static double DRIVE_V;
        public static double DRIVE_A;

        /* Drive Motor Conversion Factors */
        public static double DRIVE_CONVERSION_POSITION_FACTOR;
        public static double DRIVE_CONVERSION_VELOCITY_FACTOR;
        public static double ANGLE_CONVERSION_FACTOR;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double MAX_SPEED;
        public static double MAX_ACCEL;

        /** Radians per Second */
        public static double MAX_ANGULAR_VELOCITY;

        /* Neutral Modes */
        public static IdleMode ANGLE_NEUTRAL_MODE;
        public static IdleMode DRIVE_NEUTRAL_MODE;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static int DRIVE_MOTOR_ID_0;
            public static int ANGLE_MOTOR_ID_0;
            public static int CANCODER_ID_0;
            public static Rotation2d ANGLE_OFFSET_0;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS_0;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS_3;
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static int DRIVE_MOTOR_ID_1;
            public static int ANGLE_MOTOR_ID_1;
            public static int CANCODER_ID_1;
            public static Rotation2d ANGLE_OFFSET_1;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS_1;
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static int DRIVE_MOTOR_ID_2;
            public static int ANGLE_MOTOR_ID_2;
            public static int CANCODER_ID_2;
            public static Rotation2d ANGLE_OFFSET_2;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS_2;
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static int DRIVE_MOTOR_ID_3;
            public static int ANGLE_MOTOR_ID_3;
            public static int CANCODER_ID_3;
            public static Rotation2d ANGLE_OFFSET_3;
        }
    }
}
