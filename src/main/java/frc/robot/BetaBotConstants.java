package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class BetaBotConstants implements RobotConstants{
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
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(308.32+90); // 127.3 original value
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(81.73+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(38.759+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(331.699+90);
            public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }
    private Configuration configuration = new Configuration();

    public BetaBotConstants() {
        configuration.auto_drive_p = Auto.AUTO_DRIVE_P;
        configuration.auto_drive_i = Auto.AUTO_DRIVE_I;
        configuration.auto_drive_d = Auto.AUTO_DRIVE_D;
        configuration.auto_angle_p = Auto.AUTO_ANGLE_P;
        configuration.auto_angle_i = Auto.AUTO_ANGLE_I;
        configuration.auto_angle_d = Auto.AUTO_ANGLE_D;
        
        configuration.angler_id = Angler.ANGLER_ID;
        configuration.angler_p = Angler.ANGLER_P;
        configuration.angler_i = Angler.ANGLER_I;
        configuration.angler_d = Angler.ANGLER_D;
        configuration.angler_current_limit = Angler.ANGLER_CURRENT_LIMIT;
        configuration.angler_invert = Angler.ANGLER_INVERT;
        configuration.angler_lower_limit_switch_port = Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT;
        configuration.angler_upper_limit_switch_port = Angler.ANGLER_UPPER_LIMIT_SWTICH_PORT;
        configuration.angler_lower_limit = Angler.ANGLER_LOWER_LIMIT;
        configuration.angler_upper_limit = Angler.ANGLER_UPPER_LIMIT;
        configuration.upper_bound_limit = Angler.UPPER_BOUND_LIMIT;
        configuration.upper_bound_coefficient = Angler.UPPER_BOUND_COEFFICIENT;
        configuration.upper_bound_series = Angler.UPPER_BOUND_SERIES;
        configuration.tight_bound_coefficient = Angler.TIGHT_BOUND_COEFFICIENT;
        configuration.tight_bound_series = Angler.TIGHT_BOUND_SERIES;

        configuration.elevator_id = Elevator.ELEVATOR_ID;
        configuration.elevator_current_limit = Elevator.ELEVATOR_CURRENT_LIMIT;
        configuration.elevator_limit_switch = Elevator.ELEVATOR_LIMIT_SWITCH;
        configuration.elevator_servo_port = Elevator.ELEVATOR_SERVO_PORT;
        configuration.elevator_servo_clamped_pos = Elevator.ELEVATOR_SERVO_CLAMPED_POS;
        configuration.elevator_servo_unclamped_pos = Elevator.ELEVATOR_SERVO_UNCLAMPED_POS;
        configuration.elevator_invert = Elevator.ELEVATOR_INVERT;
        configuration.elevator_p = Elevator.ELEVATOR_P;
        configuration.elevator_i = Elevator.ELEVATOR_I;
        configuration.elevator_d = Elevator.ELEVATOR_D;
        configuration.elevator_lower_limit = Elevator.ELEVATOR_LOWER_LIMIT;
        configuration.elevator_upper_limit = Elevator.ELEVATOR_UPPER_LIMIT;
        configuration.elevator_amp = Elevator.ELEVATOR_AMP;
        configuration.elevator_stow = Elevator.ELEVATOR_STOW;

        configuration.intake_id = IntakeCarriage.INTAKE_ID;
        configuration.carriage_id = IntakeCarriage.CARRIAGE_ID;
        configuration.idle_intake_speed = IntakeCarriage.IDLE_INTAKE_SPEED;
        configuration.carriage_beam = IntakeCarriage.CARRIAGE_BEAM;
        configuration.shooter_beam = IntakeCarriage.SHOOTER_BEAM;
        configuration.amp_beam = IntakeCarriage.AMP_BEAM;

        configuration.limelight_p = Limelight.LIMELIGHT_P;
        configuration.limelight_i = Limelight.LIMELIGHT_I;
        configuration.limelight_d = Limelight.LIMELIGHT_D;
        configuration.yaw_offset = Limelight.YAW_OFFSET;

        configuration.bottom_shooter_id = Shooter.BOTTOM_SHOOTER_ID;
        configuration.top_shooter_id = Shooter.TOP_SHOOTER_ID;
        configuration.shooter_current_limit = Shooter.SHOOTER_CURRENT_LIMIT;
        configuration.shooter_invert = Shooter.SHOOTER_INVERT;
        configuration.base_shooter_speed = Shooter.BASE_SHOOTER_SPEED;
        configuration.idle_shooter_speed = Shooter.IDLE_SHOOTER_SPEED;
        configuration.shooter_error_tolerance = Shooter.SHOOTER_ERROR_TOLERANCE;
        configuration.distance_multiplier = Shooter.DISTANCE_MULTIPLIER;
        configuration.shooter_p = Shooter.SHOOTER_P;
        configuration.shooter_i = Shooter.SHOOTER_I;
        configuration.shooter_d = Shooter.SHOOTER_D;
        configuration.shooter_ff = Shooter.SHOOTER_FF;
        configuration.feeder_id = Shooter.FEEDER_ID;
        configuration.feeder_current_limit = Shooter.FEEDER_CURRENT_LIMIT;
        configuration.feeder_invert = Shooter.FEEDER_INVERT;

        configuration.gyro_offset = Swerve.GYRO_OFFSET;
        configuration.pigeon_id = Swerve.PIGEON_ID;
        configuration.invert_gyro = Swerve.INVERT_GYRO;
        configuration.chosen_module = Swerve.CHOSEN_MODULE;
        configuration.track_width = Swerve.TRACK_WIDTH;
        configuration.wheel_base = Swerve.WHEEL_BASE;
        configuration.center_to_wheel = Swerve.CENTER_TO_WHEEL;
        configuration.wheel_circumference = Swerve.WHEEL_CIRCUMFERENCE;
        configuration.swerve_kinematics = Swerve.SWERVE_KINEMATICS;
        configuration.voltage_comp = Swerve.VOLTAGE_COMP;
        configuration.drive_gear_ratio = Swerve.DRIVE_GEAR_RATIO;
        configuration.angle_gear_ratio = Swerve.ANGLE_GEAR_RATIO;
        configuration.angle_motor_invert = Swerve.ANGLE_MOTOR_INVERT;
        configuration.drive_motor_invert = Swerve.DRIVE_MOTOR_INVERT;
        configuration.cancoder_sensor_direction = Swerve.CANCODER_SENSOR_DIRECTION;
        configuration.angle_continuous_supply_current_limit = Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        configuration.angle_peak_supply_current_limit = Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
        configuration.angle_peak_supply_current_duration = Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
        configuration.angle_enable_supply_current_limit = Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
        configuration.drive_continuous_supply_current_limit = Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
        configuration.drive_peak_supply_current_limit = Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT;
        configuration.drive_peak_supply_current_duration = Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION;
        configuration.drive_enable_supply_current_limit = Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
        configuration.open_loop_ramp = Swerve.OPEN_LOOP_RAMP;
        configuration.closed_loop_ramp = Swerve.CLOSED_LOOP_RAMP;
        configuration.angle_p = Swerve.ANGLE_P;
        configuration.angle_i = Swerve.ANGLE_I;
        configuration.angle_d = Swerve.ANGLE_D;
        configuration.angle_f = Swerve.ANGLE_F;
        configuration.drive_s = Swerve.DRIVE_S;
        configuration.drive_v = Swerve.DRIVE_V;
        configuration.drive_a = Swerve.DRIVE_A;
        configuration.drive_conversion_position_factor = Swerve.DRIVE_CONVERSION_POSITION_FACTOR;
        configuration.drive_conversion_velocity_factor = Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR;
        configuration.angle_conversion_factor = Swerve.ANGLE_CONVERSION_FACTOR;
        configuration.max_speed = Swerve.MAX_SPEED;
        configuration.max_accel = Swerve.MAX_ACCEL;
        configuration.max_angular_velocity = Swerve.MAX_ANGULAR_VELOCITY;
        configuration.angle_neutral_mode = Swerve.ANGLE_NEUTRAL_MODE;
        configuration.drive_neutral_mode = Swerve.DRIVE_NEUTRAL_MODE;

        configuration.drive_motor_id_3 = Swerve.Mod3.DRIVE_MOTOR_ID;
        configuration.angle_motor_id_3 = Swerve.Mod3.ANGLE_MOTOR_ID;
        configuration.cancoder_id_3 = Swerve.Mod3.CANCODER_ID;
        configuration.angle_offset_3 = Swerve.Mod3.ANGLE_OFFSET;
        configuration.swerve_module_constants_3 = Swerve.Mod3.SWERVE_MODULE_CONSTANTS;

        configuration.drive_motor_id_2 = Swerve.Mod2.DRIVE_MOTOR_ID;
        configuration.angle_motor_id_2 = Swerve.Mod2.ANGLE_MOTOR_ID;
        configuration.cancoder_id_2 = Swerve.Mod2.CANCODER_ID;
        configuration.angle_offset_2 = Swerve.Mod2.ANGLE_OFFSET;
        configuration.swerve_module_constants_2 = Swerve.Mod2.SWERVE_MODULE_CONSTANTS;

        configuration.drive_motor_id_0 = Swerve.Mod0.DRIVE_MOTOR_ID;
        configuration.angle_motor_id_0 = Swerve.Mod0.ANGLE_MOTOR_ID;
        configuration.cancoder_id_0 = Swerve.Mod0.CANCODER_ID;
        configuration.angle_offset_0 = Swerve.Mod0.ANGLE_OFFSET;
        configuration.swerve_module_constants_0 = Swerve.Mod0.SWERVE_MODULE_CONSTANTS;

        configuration.drive_motor_id_1 = Swerve.Mod1.DRIVE_MOTOR_ID;
        configuration.angle_motor_id_1 = Swerve.Mod1.ANGLE_MOTOR_ID;
        configuration.cancoder_id_1 = Swerve.Mod1.CANCODER_ID;
        configuration.angle_offset_1 = Swerve.Mod1.ANGLE_OFFSET;
        configuration.swerve_module_constants_1 = Swerve.Mod1.SWERVE_MODULE_CONSTANTS;
    }
    @Override
    public Configuration getConfiguration() {
        return configuration;
    }
}