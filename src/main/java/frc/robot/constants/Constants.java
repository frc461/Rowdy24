package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import java.util.function.BiFunction;
import java.util.function.Function;

public final class Constants {
    public static double STICK_DEADBAND;

    public static final class Auto {
        public static double AUTO_DRIVE_P;
        public static double AUTO_DRIVE_I;
        public static double AUTO_DRIVE_D;
        public static double AUTO_ANGLE_P;
        public static double AUTO_ANGLE_I;
        public static double AUTO_ANGLE_D;
    }

    public static final class Angler {
        // basic configs
        public static int ANGLER_ID;
        public static int ANGLER_CURRENT_LIMIT;
        public static boolean ANGLER_INVERT;

        // pid for angler
        public static double ANGLER_P;
        public static double ANGLER_I;
        public static double ANGLER_D;

        // code limits on encoder values
        public static double ANGLER_LOWER_LIMIT;
        public static double ANGLER_UPPER_LIMIT;

        // setpoint(s)
        public static double ANGLER_LAYUP_POSITION;

        // equation constants for shooting with distance from april tag
        public static double CLOSE_AIM_LIMIT;
        public static BiFunction<Double, Double, Double> CLOSE_AIM_EQUATION;
        public static BiFunction<Double, Double, Double> FAR_AIM_EQUATION;

        public static double SPEAKER_HEIGHT;
        public static double SHOOTER_HEIGHT;
        public static double Y_COMPONENT_AIM;
        public static double Z_DEPTH_OFFSET;
        public static Function<Double, Double> ANGLE_TO_ENCODER_VALUE;
        public static BiFunction<Double, Double, Double> AUTO_ANGLER_AIM_EQUATION;
    }

    public static final class Elevator {
        // basic configs
        public static int ELEVATOR_ID;
        public static int ELEVATOR_CURRENT_LIMIT;
        public static int ELEVATOR_LIMIT_SWITCH;
        public static InvertedValue ELEVATOR_INVERT;

        // servo to hold elevator in endgame
        public static int ELEVATOR_SERVO_PORT;
        public static double ELEVATOR_SERVO_CLAMPED_POS;
        public static double ELEVATOR_SERVO_UNCLAMPED_POS;

        // pid
        public static double ELEVATOR_P;
        public static double ELEVATOR_I;
        public static double ELEVATOR_D;

        // presets
        public static double ELEVATOR_LOWER_LIMIT;
        public static double ELEVATOR_UPPER_LIMIT;
        public static double ELEVATOR_AMP;
        public static double ELEVATOR_STOW;
    }

    public static final class IntakeCarriage {
        // basic configs
        public static int INTAKE_ID;
        public static int CARRIAGE_ID;

        // idle when intake is not actively being used
        public static double IDLE_INTAKE_SPEED;

        // beam breaks
        public static int CARRIAGE_BEAM;
        public static int SHOOTER_BEAM;
        public static int AMP_BEAM;
    }

    public static final class Limelight {
        // pid for limelight alignment
        public static double LIMELIGHT_P;
        public static double LIMELIGHT_I;
        public static double LIMELIGHT_D;

        // turn slightly to the right
        public static double YAW_OFFSET;
    }

    public static final class Shooter {
        // basic configs
        public static int BOTTOM_SHOOTER_ID; //bottom wheel
        public static int TOP_SHOOTER_ID; //top wheel
        public static int SHOOTER_CURRENT_LIMIT;
        public static boolean SHOOTER_INVERT;

        // baseline shooter speed in RPM
        public static double BASE_SHOOTER_SPEED; //baseline shooter speed in rpm

        // percentage to idle at when shooter is not in use
        public static double IDLE_SHOOTER_SPEED;

        // +/-tolerance for considering if the shooter is up to speed
        public static double SHOOTER_ERROR_TOLERANCE; // +/-tolerance for considering if the shooter is up to speed

        // increases shooter speed as distance from speaker increases
        public static double DISTANCE_MULTIPLIER;

        // shooter pidf
        public static double SHOOTER_P;
        public static double SHOOTER_I;
        public static double SHOOTER_D;
        public static double SHOOTER_FF;
    }

    public static final class Swerve {
        // gyro config
        public static double GYRO_OFFSET;
        public static int PIGEON_ID;
        public static boolean INVERT_GYRO; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        // min and max angle of swerve
        public static double MINIMUM_ANGLE;
        public static double MAXIMUM_ANGLE;
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

        /* Motor Inverts */
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
            public static int DRIVE_MOTOR_ID;
            public static int ANGLE_MOTOR_ID;
            public static int CANCODER_ID;
            public static Rotation2d ANGLE_OFFSET;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS;
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static int DRIVE_MOTOR_ID;
            public static int ANGLE_MOTOR_ID;
            public static int CANCODER_ID;
            public static Rotation2d ANGLE_OFFSET;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS;
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static int DRIVE_MOTOR_ID;
            public static int ANGLE_MOTOR_ID;
            public static int CANCODER_ID;
            public static Rotation2d ANGLE_OFFSET;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS;
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static int DRIVE_MOTOR_ID;
            public static int ANGLE_MOTOR_ID;
            public static int CANCODER_ID;
            public static Rotation2d ANGLE_OFFSET;
            public static SwerveModuleConstants SWERVE_MODULE_CONSTANTS;
        }
    }
}
