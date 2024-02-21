package frc.robot.constants.variants;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;

public final class RowdyConstants implements RobotConstants {
    public static final double STICK_DEADBAND = 0.1;
    public static final double TRIGGER_DEADBAND = 0.5;
    public static final double MINIMUM_ANGLE = -180.0;
    public static final double MAXIMUM_ANGLE = 180.0;

    public static final class Auto {
        public static final double AUTO_DRIVE_P = 0.1;
        public static final double AUTO_DRIVE_I = 0.0;
        public static final double AUTO_DRIVE_D = 0.00001;
        public static final double AUTO_ANGLE_P = 0.2;
        public static final double AUTO_ANGLE_I = 0.0;
        public static final double AUTO_ANGLE_D = 0.0;
    }

    public static final class Angler {
        public static final int ANGLER_ID = 62;
        public static final double ANGLER_P = 0.035;
        public static final double ANGLER_I = 0.00009;
        public static final double ANGLER_D = 0.0001;
        public static final int ANGLER_CURRENT_LIMIT = 35;
        public static final boolean ANGLER_INVERT = false;
        public static final int ANGLER_LOWER_LIMIT_SWITCH_PORT = 1;
        public static final int ANGLER_UPPER_LIMIT_SWTICH_PORT = 0;
        public static final double ANGLER_LOWER_LIMIT = 0;
        public static final double ANGLER_UPPER_LIMIT = 20;
        public static final double UPPER_BOUND_LIMIT = 2.26;
        public static final double UPPER_BOUND_COEFFICIENT = 51.8;
        public static final double UPPER_BOUND_SERIES = -1.31;
        public static final double TIGHT_BOUND_COEFFICIENT = 40.9; // 43.9
        public static final double TIGHT_BOUND_SERIES = -1.3;
    }

    public static final class Elevator {
        public static final int ELEVATOR_ID = 31;
        public static final int ELEVATOR_CURRENT_LIMIT = 70;
        public static final int ELEVATOR_LIMIT_SWITCH = 2;
        public static final int ELEVATOR_SERVO_PORT = 1;
        public static final double ELEVATOR_SERVO_CLAMPED_POS = 1;
        public static final double ELEVATOR_SERVO_UNCLAMPED_POS = 1;
        public static final boolean ELEVATOR_INVERT = true;
        public static final double ELEVATOR_P = 0.097;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;
        public static final double ELEVATOR_LOWER_LIMIT = 0;
        public static final double ELEVATOR_UPPER_LIMIT = 61;
        public static final double ELEVATOR_AMP = 61.0;
        public static final double ELEVATOR_STOW = 0.0;
    }

    public static final class IntakeCarriage {
        public static final int INTAKE_ID = 41;
        public static final int CARRIAGE_ID = 42;
        public static final double IDLE_INTAKE_SPEED = -0.15;
        public static final int CARRIAGE_BEAM = 4;
        public static final int SHOOTER_BEAM = 3;
        public static final int AMP_BEAM = 5;
    }

    public static final class Limelight {
        public static final double LIMELIGHT_P = 0.07;
        public static final double LIMELIGHT_I = 0.03;
        public static final double LIMELIGHT_D = 0;
        public static final double YAW_OFFSET = -10.5;
    }

    public static final class Shooter {
        public static final int BOTTOM_SHOOTER_ID = 60; //BOTTOM WHEEL
        public static final int TOP_SHOOTER_ID = 61; //TOP WHEEL
        public static final int SHOOTER_CURRENT_LIMIT = 60;
        public static final boolean SHOOTER_INVERT = false;
        //baseline shooter speed in RPM
        public static final double BASE_SHOOTER_SPEED = 6000;
        public static final double IDLE_SHOOTER_SPEED = 0.3;
        // +/-tolerance for considering if the shooter is up to speed
        public static final double SHOOTER_ERROR_TOLERANCE = 300;
        public static final double DISTANCE_MULTIPLIER = 10;
        public static final double SHOOTER_P = 0.00062; //was 0.003
        public static final double SHOOTER_I = 0.000000001;
        public static final double SHOOTER_D = 0.0005;
        public static final double SHOOTER_FF = 0.1;
        public static final int FEEDER_ID = 59;
        public static final int FEEDER_CURRENT_LIMIT = 50;
        public static final boolean FEEDER_INVERT = false;
    }

    public static final class Swerve {
        public static final double GYRO_OFFSET = 0;
        public static final int PIGEON_ID = 51;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(18.375);
        public static final double WHEEL_BASE = Units.inchesToMeters(18.375);
        public static final double CENTER_TO_WHEEL =
                Math.sqrt(Math.pow(WHEEL_BASE / 2.0, 2) + Math.pow(TRACK_WIDTH / 2.0, 2));
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
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

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

        /* Angle Motor PID Values */
        public static final double ANGLE_P = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_I = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_D = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_F = CHOSEN_MODULE.angleKF;

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
        public static final double MAX_SPEED = 4.1;
        public static final double MAX_ACCEL = 4.1;

        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(38.759+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(331.699+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(81.73+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(308.32+90); // 127.3 original value
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }
    private final Constants constants = new Constants();

    public RowdyConstants() {
        Constants.Auto.AUTO_DRIVE_P = Auto.AUTO_DRIVE_P;
        Constants.Auto.AUTO_DRIVE_I = Auto.AUTO_DRIVE_I;
        Constants.Auto.AUTO_DRIVE_D = Auto.AUTO_DRIVE_D;
        Constants.Auto.AUTO_ANGLE_P = Auto.AUTO_ANGLE_P;
        Constants.Auto.AUTO_ANGLE_I = Auto.AUTO_ANGLE_I;
        Constants.Auto.AUTO_ANGLE_D = Auto.AUTO_ANGLE_D;
        
        Constants.Angler.ANGLER_ID = Angler.ANGLER_ID;
        Constants.Angler.ANGLER_P = Angler.ANGLER_P;
        Constants.Angler.ANGLER_I = Angler.ANGLER_I;
        Constants.Angler.ANGLER_D = Angler.ANGLER_D;
        Constants.Angler.ANGLER_CURRENT_LIMIT = Angler.ANGLER_CURRENT_LIMIT;
        Constants.Angler.ANGLER_INVERT = Angler.ANGLER_INVERT;
        Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT = Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT;
        Constants.Angler.ANGLER_UPPER_LIMIT_SWITCH_PORT = Angler.ANGLER_UPPER_LIMIT_SWTICH_PORT;
        Constants.Angler.ANGLER_LOWER_LIMIT = Angler.ANGLER_LOWER_LIMIT;
        Constants.Angler.ANGLER_UPPER_LIMIT = Angler.ANGLER_UPPER_LIMIT;
        Constants.Angler.UPPER_BOUND_LIMIT = Angler.UPPER_BOUND_LIMIT;
        Constants.Angler.UPPER_BOUND_COEFFICIENT = Angler.UPPER_BOUND_COEFFICIENT;
        Constants.Angler.UPPER_BOUND_SERIES = Angler.UPPER_BOUND_SERIES;
        Constants.Angler.TIGHT_BOUND_COEFFICIENT = Angler.TIGHT_BOUND_COEFFICIENT;
        Constants.Angler.TIGHT_BOUND_SERIES = Angler.TIGHT_BOUND_SERIES;

        Constants.Elevator.ELEVATOR_ID = Elevator.ELEVATOR_ID;
        Constants.Elevator.ELEVATOR_CURRENT_LIMIT = Elevator.ELEVATOR_CURRENT_LIMIT;
        Constants.Elevator.ELEVATOR_LIMIT_SWITCH = Elevator.ELEVATOR_LIMIT_SWITCH;
        Constants.Elevator.ELEVATOR_SERVO_PORT = Elevator.ELEVATOR_SERVO_PORT;
        Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS = Elevator.ELEVATOR_SERVO_CLAMPED_POS;
        Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS = Elevator.ELEVATOR_SERVO_UNCLAMPED_POS;
        Constants.Elevator.ELEVATOR_INVERT = Elevator.ELEVATOR_INVERT;
        Constants.Elevator.ELEVATOR_P = Elevator.ELEVATOR_P;
        Constants.Elevator.ELEVATOR_I = Elevator.ELEVATOR_I;
        Constants.Elevator.ELEVATOR_D = Elevator.ELEVATOR_D;
        Constants.Elevator.ELEVATOR_LOWER_LIMIT = Elevator.ELEVATOR_LOWER_LIMIT;
        Constants.Elevator.ELEVATOR_UPPER_LIMIT = Elevator.ELEVATOR_UPPER_LIMIT;
        Constants.Elevator.ELEVATOR_AMP = Elevator.ELEVATOR_AMP;
        Constants.Elevator.ELEVATOR_STOW = Elevator.ELEVATOR_STOW;

        Constants.IntakeCarriage.INTAKE_ID = IntakeCarriage.INTAKE_ID;
        Constants.IntakeCarriage.CARRIAGE_ID = IntakeCarriage.CARRIAGE_ID;
        Constants.IntakeCarriage.IDLE_INTAKE_SPEED = IntakeCarriage.IDLE_INTAKE_SPEED;
        Constants.IntakeCarriage.CARRIAGE_BEAM = IntakeCarriage.CARRIAGE_BEAM;
        Constants.IntakeCarriage.SHOOTER_BEAM = IntakeCarriage.SHOOTER_BEAM;
        Constants.IntakeCarriage.AMP_BEAM = IntakeCarriage.AMP_BEAM;

        Constants.Limelight.LIMELIGHT_P = Limelight.LIMELIGHT_P;
        Constants.Limelight.LIMELIGHT_I = Limelight.LIMELIGHT_I;
        Constants.Limelight.LIMELIGHT_D = Limelight.LIMELIGHT_D;
        Constants.Limelight.YAW_OFFSET = Limelight.YAW_OFFSET;

        Constants.Shooter.BOTTOM_SHOOTER_ID = Shooter.BOTTOM_SHOOTER_ID;
        Constants.Shooter.TOP_SHOOTER_ID = Shooter.TOP_SHOOTER_ID;
        Constants.Shooter.SHOOTER_CURRENT_LIMIT = Shooter.SHOOTER_CURRENT_LIMIT;
        Constants.Shooter.SHOOTER_INVERT = Shooter.SHOOTER_INVERT;
        Constants.Shooter.BASE_SHOOTER_SPEED = Shooter.BASE_SHOOTER_SPEED;
        Constants.Shooter.IDLE_SHOOTER_SPEED = Shooter.IDLE_SHOOTER_SPEED;
        Constants.Shooter.SHOOTER_ERROR_TOLERANCE = Shooter.SHOOTER_ERROR_TOLERANCE;
        Constants.Shooter.DISTANCE_MULTIPLIER = Shooter.DISTANCE_MULTIPLIER;
        Constants.Shooter.SHOOTER_P = Shooter.SHOOTER_P;
        Constants.Shooter.SHOOTER_I = Shooter.SHOOTER_I;
        Constants.Shooter.SHOOTER_D = Shooter.SHOOTER_D;
        Constants.Shooter.SHOOTER_FF = Shooter.SHOOTER_FF;
        Constants.Shooter.FEEDER_ID = Shooter.FEEDER_ID;
        Constants.Shooter.FEEDER_CURRENT_LIMIT = Shooter.FEEDER_CURRENT_LIMIT;
        Constants.Shooter.FEEDER_INVERT = Shooter.FEEDER_INVERT;

        Constants.Swerve.GYRO_OFFSET = Swerve.GYRO_OFFSET;
        Constants.Swerve.PIGEON_ID = Swerve.PIGEON_ID;
        Constants.Swerve.INVERT_GYRO = Swerve.INVERT_GYRO;
        Constants.Swerve.CHOSEN_MODULE = Swerve.CHOSEN_MODULE;
        Constants.Swerve.TRACK_WIDTH = Swerve.TRACK_WIDTH;
        Constants.Swerve.WHEEL_BASE = Swerve.WHEEL_BASE;
        Constants.Swerve.CENTER_TO_WHEEL = Swerve.CENTER_TO_WHEEL;
        Constants.Swerve.WHEEL_CIRCUMFERENCE = Swerve.WHEEL_CIRCUMFERENCE;
        Constants.Swerve.SWERVE_KINEMATICS = Swerve.SWERVE_KINEMATICS;
        Constants.Swerve.VOLTAGE_COMP = Swerve.VOLTAGE_COMP;
        Constants.Swerve.DRIVE_GEAR_RATIO = Swerve.DRIVE_GEAR_RATIO;
        Constants.Swerve.ANGLE_GEAR_RATIO = Swerve.ANGLE_GEAR_RATIO;
        Constants.Swerve.ANGLE_MOTOR_INVERT = Swerve.ANGLE_MOTOR_INVERT;
        Constants.Swerve.DRIVE_MOTOR_INVERT = Swerve.DRIVE_MOTOR_INVERT;
        Constants.Swerve.CANCODER_SENSOR_DIRECTION = Swerve.CANCODER_SENSOR_DIRECTION;
        Constants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION = Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        Constants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION = Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION;
        Constants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        Constants.Swerve.OPEN_LOOP_RAMP = Swerve.OPEN_LOOP_RAMP;
        Constants.Swerve.CLOSED_LOOP_RAMP = Swerve.CLOSED_LOOP_RAMP;
        Constants.Swerve.ANGLE_P = Swerve.ANGLE_P;
        Constants.Swerve.ANGLE_I = Swerve.ANGLE_I;
        Constants.Swerve.ANGLE_D = Swerve.ANGLE_D;
        Constants.Swerve.ANGLE_F = Swerve.ANGLE_F;
        Constants.Swerve.DRIVE_S = Swerve.DRIVE_S;
        Constants.Swerve.DRIVE_V = Swerve.DRIVE_V;
        Constants.Swerve.DRIVE_A = Swerve.DRIVE_A;
        Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR = Swerve.DRIVE_CONVERSION_POSITION_FACTOR;
        Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR = Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR;
        Constants.Swerve.ANGLE_CONVERSION_FACTOR = Swerve.ANGLE_CONVERSION_FACTOR;
        Constants.Swerve.MAX_SPEED = Swerve.MAX_SPEED;
        Constants.Swerve.MAX_ACCEL = Swerve.MAX_ACCEL;
        Constants.Swerve.MAX_ANGULAR_VELOCITY = Swerve.MAX_ANGULAR_VELOCITY;
        Constants.Swerve.ANGLE_NEUTRAL_MODE = Swerve.ANGLE_NEUTRAL_MODE;
        Constants.Swerve.DRIVE_NEUTRAL_MODE = Swerve.DRIVE_NEUTRAL_MODE;

        Constants.Swerve.Mod3.DRIVE_MOTOR_ID_3 = Swerve.Mod3.DRIVE_MOTOR_ID;
        Constants.Swerve.Mod3.ANGLE_MOTOR_ID_3 = Swerve.Mod3.ANGLE_MOTOR_ID;
        Constants.Swerve.Mod3.CANCODER_ID_3 = Swerve.Mod3.CANCODER_ID;
        Constants.Swerve.Mod3.ANGLE_OFFSET_3 = Swerve.Mod3.ANGLE_OFFSET;
        Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_3 = Swerve.Mod3.SWERVE_MODULE_CONSTANTS;

        Constants.Swerve.Mod2.DRIVE_MOTOR_ID_2 = Swerve.Mod2.DRIVE_MOTOR_ID;
        Constants.Swerve.Mod2.ANGLE_MOTOR_ID_2 = Swerve.Mod2.ANGLE_MOTOR_ID;
        Constants.Swerve.Mod2.CANCODER_ID_2 = Swerve.Mod2.CANCODER_ID;
        Constants.Swerve.Mod2.ANGLE_OFFSET_2 = Swerve.Mod2.ANGLE_OFFSET;
        Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS_2 = Swerve.Mod2.SWERVE_MODULE_CONSTANTS;

        Constants.Swerve.Mod0.DRIVE_MOTOR_ID_0 = Swerve.Mod0.DRIVE_MOTOR_ID;
        Constants.Swerve.Mod0.ANGLE_MOTOR_ID_0 = Swerve.Mod0.ANGLE_MOTOR_ID;
        Constants.Swerve.Mod0.CANCODER_ID_0 = Swerve.Mod0.CANCODER_ID;
        Constants.Swerve.Mod0.ANGLE_OFFSET_0 = Swerve.Mod0.ANGLE_OFFSET;
        Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_0 = Swerve.Mod0.SWERVE_MODULE_CONSTANTS;

        Constants.Swerve.Mod1.DRIVE_MOTOR_ID_1 = Swerve.Mod1.DRIVE_MOTOR_ID;
        Constants.Swerve.Mod1.ANGLE_MOTOR_ID_1 = Swerve.Mod1.ANGLE_MOTOR_ID;
        Constants.Swerve.Mod1.CANCODER_ID_1 = Swerve.Mod1.CANCODER_ID;
        Constants.Swerve.Mod1.ANGLE_OFFSET_1 = Swerve.Mod1.ANGLE_OFFSET;
        Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS_1 = Swerve.Mod1.SWERVE_MODULE_CONSTANTS;
    }
    @Override
    public Constants getConfiguration() {
        return constants;
    }
}