package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import java.util.function.BiFunction;
import java.util.function.Function;

public final class Constants {
    public static final double STICK_DEADBAND = 0.1;

    public static final class Auto {
        // pid values for pathplanner
        public static final double AUTO_DRIVE_P = 1.5;
        public static final double AUTO_DRIVE_I = 0.0;
        public static final double AUTO_DRIVE_D = 0.0;
        public static final double AUTO_ANGLE_P = 1.0;
        public static final double AUTO_ANGLE_I = 0.0;
        public static final double AUTO_ANGLE_D = 0.0;
    }

    public static final class Angler {
        // basic configs
        public static final int ANGLER_ID = 62;
        public static final int ANGLER_CURRENT_LIMIT = 35;
        public static final boolean ANGLER_INVERT = false;

        // pid for angler
        public static final double ANGLER_P = 0.2;
        public static final double ANGLER_I = 0.00001;
        public static final double ANGLER_D = 0.0001;

        // required accuracy to consider angler accurate to shoot a note
        public static final double ANGLER_ACCURACY_REQUIREMENT = 0.97;

        // limit switches
        public static final int ANGLER_LOWER_LIMIT_SWITCH = 6;

        // code limits on encoder values
        public static final double ANGLER_LOWER_LIMIT = 0;
        public static final double ANGLER_UPPER_LIMIT = 20;

        // preset
        public static final double ANGLER_LAYUP_PRESET = 18;
        public static final double ANGLER_SHUTTLE_PRESET = 18;

        // angler equation to shoot from anywhere
        public static double ANGLER_ENCODER_OFFSET = 0;
        public static final double SPEAKER_HEIGHT = 1.98;
        public static final double SHOOTER_HEIGHT = 0.2989; // CAD
        public static final double SLANT_HEIGHT = .23 * Math.tan(Math.toRadians(14));
        public static final double Z_COMPONENT_AIM = SPEAKER_HEIGHT + SLANT_HEIGHT - SHOOTER_HEIGHT;
        public static final BiFunction<Double, Double, Double> ANGLE_TO_ENCODER_VALUE =
                (angle, dist) -> 20.0 / 46.4 * (angle - 12.4) + 0.55 * dist + ANGLER_ENCODER_OFFSET;
        public static final Function<Double, Double> AUTO_ANGLER_AIM_EQUATION =
                (dist) -> ANGLE_TO_ENCODER_VALUE.apply(
                        Math.toDegrees(Math.atan(Z_COMPONENT_AIM / dist)),
                        dist
                );
    }

    public static final class Elevator {
        // basic configs
        public static final int ELEVATOR_ID = 31;
        public static final int ELEVATOR_FOLLOWER_ID = 32;
        public static final int ELEVATOR_CURRENT_LIMIT = 80;
        public static final int SERVO_LIMIT_SWITCH = 1;
        public static final int ELEVATOR_LIMIT_SWITCH = 2;
        public static final InvertedValue ELEVATOR_INVERT = InvertedValue.Clockwise_Positive;

        // servo to hold elevator in endgame
        public static final int ELEVATOR_SERVO_PORT = 1;
        public static final double ELEVATOR_SERVO_CLAMPED_POS = .07; // new servo values
        public static final double ELEVATOR_SERVO_UNCLAMPED_POS = .39;
        
        // pid
        public static final double ELEVATOR_P = 0.05;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;

        // required accuracy to consider angler accurate to shoot a note
        public static final double ELEVATOR_ACCURACY_REQUIREMENT = 0.85;

        // presets
        public static final double ELEVATOR_LOWER_LIMIT = 0;
        public static final double ELEVATOR_UPPER_LIMIT = 37;
        public static final double ELEVATOR_STOW = ELEVATOR_LOWER_LIMIT;
        public static final double ELEVATOR_OUTTAKE = 10;
        public static final double ELEVATOR_AMP = 36;
    }

    public static final class IntakeCarriage {
        // basic configs
        public static final int INTAKE_ID = 41;
        public static final int CARRIAGE_ID = 42;

        // beam breaks
        public static final int CARRIAGE_BEAM = 4;
        public static final int SHOOTER_BEAM = 3;
        public static final int AMP_BEAM = 5;

        // Blinkin
        public static final int LIGHT_ID = 0;
    }

    public static final class Limelight {
        public static final double CAMERA_FORWARD = 0.0223774;
        public static final double CAMERA_UP = 0.5916676;
        public static final double CAMERA_PITCH = 23.6838871;
        
        // turn slightly to the right 
        public static final double YAW_OFFSET = 0;
        public static final double X_DEPTH_OFFSET = -0.23; // Half of the depth of the speaker into the field
        public static final double Y_DEPTH_OFFSET = Units.inchesToMeters(6.5 / 2);
    }

    public static final class Shooter {
        // basic configs
        public static final int BOTTOM_SHOOTER_ID = 60; // BOTTOM WHEEL
        public static final int TOP_SHOOTER_ID = 61; // TOP WHEEL
        public static final int SHOOTER_CURRENT_LIMIT = 60;
        public static final boolean SHOOTER_INVERT = false;

        // idle shooter speed in RPM
        public static final double IDLE_SHOOTER_POWER = 0.2;

        // shuttle power out of full power 1
        // TODO: test shuttle power at worlds
        public static final double SHUTTLE_SHOOTER_POWER = 0.5;
        public static final double SHUTTLE_X_BLUE = 1.23;
        public static final double SHUTTLE_X_RED = 15.31;
        public static final double SHUTTLE_Y = 7.19;

        // baseline shooter speed in RPM
        public static final double BASE_SHOOTER_SPEED = 6000;

        // boundary within which shooter will automatically rev up (if default command is enabled)
        public static final double SHOOTER_TRIGGER_DISTANCE = 5.75;

        // required accuracy to consider shooter up to speed
        public static final double SHOOTER_ACCURACY_REQUIREMENT = 0.8;

        // increases shooter speed as distance from speaker increases
        public static final double DISTANCE_MULTIPLIER = 10;

        // shooter pidf
        public static final double SHOOTER_P = 0.00062; // was 0.003
        public static final double SHOOTER_I = 0.000000001;
        public static final double SHOOTER_D = 0.0005;
        public static final double SHOOTER_FF = 0.1;
    }

    public static final class Swerve {
        // required accuracy to consider turret accurate to shoot a note
        public static final double TURRET_ACCURACY_REQUIREMENT = 2.5;
        // gyro config
        public static final double GYRO_OFFSET = 180;
        public static final int PIGEON_ID = 51;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)
        
        // min and max angle of swerve
        public static final double MINIMUM_ANGLE = -180.0;
        public static final double MAXIMUM_ANGLE = 180.0;

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3_PLUS);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(18.375);
        public static final double WHEEL_BASE = Units.inchesToMeters(18.375);
        public static final double CENTER_TO_WHEEL = Math
                .sqrt(Math.pow(WHEEL_BASE / 2.0, 2) + Math.pow(TRACK_WIDTH / 2.0, 2));
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Voltage Compensation */
        public static final double VOLTAGE_COMP = 12.0;

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_SENSOR_DIRECTION = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_SUPPLY_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_SUPPLY_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor (Velocity) PID Values */
        public static final double ANGLE_P = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_I = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_D = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_F = CHOSEN_MODULE.angleKF;

        /* Integrated Angular Position PID Values */
        public static final double ANGULAR_POSITION_P = 0.02;
        public static final double ANGULAR_POSITION_I = 0;
        public static final double ANGULAR_POSITION_D = 0.0006;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.05;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_S = (0.16861 / 12);
        public static final double DRIVE_V = (2.6686 / 12);
        public static final double DRIVE_A = (0.34757 / 12);

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_CONVERSION_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
        public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.671;
        public static final double MAX_ACCEL = 5.5;

        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod3 {

            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(226.05);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */

        public static final class Mod2 {

            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(138.87);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(300.32); // 127.3 original value
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(168.31);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }
}