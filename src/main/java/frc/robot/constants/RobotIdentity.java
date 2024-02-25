package frc.robot.constants;

import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.variants.RowdyConstants;

public enum RobotIdentity {
    PRACTICE,
    ROWDY;

    public static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (!mac.isEmpty()) {
            if (mac.equals(MacAddress.PRACTICE)) {
                return PRACTICE;
            }
            else if (mac.equals(MacAddress.ROWDY)) {
                return ROWDY;
            }
        }
        return ROWDY;
    }

    public static void initializeConstants() {
        switch (RobotIdentity.getIdentity()) {
            case PRACTICE -> {
                Constants.STICK_DEADBAND = PracticeConstants.STICK_DEADBAND;
                Constants.TRIGGER_DEADBAND = PracticeConstants.TRIGGER_DEADBAND;
                Constants.MINIMUM_ANGLE = PracticeConstants.MINIMUM_ANGLE;
                Constants.MAXIMUM_ANGLE = PracticeConstants.MAXIMUM_ANGLE;

                Constants.Auto.AUTO_DRIVE_P = PracticeConstants.Auto.AUTO_DRIVE_P;
                Constants.Auto.AUTO_DRIVE_I = PracticeConstants.Auto.AUTO_DRIVE_I;
                Constants.Auto.AUTO_DRIVE_D = PracticeConstants.Auto.AUTO_DRIVE_D;
                Constants.Auto.AUTO_ANGLE_P = PracticeConstants.Auto.AUTO_ANGLE_P;
                Constants.Auto.AUTO_ANGLE_I = PracticeConstants.Auto.AUTO_ANGLE_I;
                Constants.Auto.AUTO_ANGLE_D = PracticeConstants.Auto.AUTO_ANGLE_D;

                Constants.Angler.ANGLER_ID = PracticeConstants.Angler.ANGLER_ID;
                Constants.Angler.ANGLER_P = PracticeConstants.Angler.ANGLER_P;
                Constants.Angler.ANGLER_I = PracticeConstants.Angler.ANGLER_I;
                Constants.Angler.ANGLER_D = PracticeConstants.Angler.ANGLER_D;
                Constants.Angler.ANGLER_CURRENT_LIMIT = PracticeConstants.Angler.ANGLER_CURRENT_LIMIT;
                Constants.Angler.ANGLER_INVERT = PracticeConstants.Angler.ANGLER_INVERT;
                Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT = PracticeConstants.Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT;
                Constants.Angler.ANGLER_UPPER_LIMIT_SWITCH_PORT = PracticeConstants.Angler.ANGLER_UPPER_LIMIT_SWTICH_PORT;
                Constants.Angler.ANGLER_LOWER_LIMIT = PracticeConstants.Angler.ANGLER_LOWER_LIMIT;
                Constants.Angler.ANGLER_UPPER_LIMIT = PracticeConstants.Angler.ANGLER_UPPER_LIMIT;
                Constants.Angler.UPPER_AIM_LIMIT = PracticeConstants.Angler.UPPER_AIM_LIMIT;
                Constants.Angler.UPPER_AIM_EQUATION = PracticeConstants.Angler::UPPER_AIM_EQUATION;
                Constants.Angler.TIGHT_AIM_EQUATION = PracticeConstants.Angler::TIGHT_AIM_EQUATION;

                Constants.Elevator.ELEVATOR_ID = PracticeConstants.Elevator.ELEVATOR_ID;
                Constants.Elevator.ELEVATOR_CURRENT_LIMIT = PracticeConstants.Elevator.ELEVATOR_CURRENT_LIMIT;
                Constants.Elevator.ELEVATOR_LIMIT_SWITCH = PracticeConstants.Elevator.ELEVATOR_LIMIT_SWITCH;
                Constants.Elevator.ELEVATOR_SERVO_PORT = PracticeConstants.Elevator.ELEVATOR_SERVO_PORT;
                Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS = PracticeConstants.Elevator.ELEVATOR_SERVO_CLAMPED_POS;
                Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS = PracticeConstants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS;
                Constants.Elevator.ELEVATOR_INVERT = PracticeConstants.Elevator.ELEVATOR_INVERT;
                Constants.Elevator.ELEVATOR_P = PracticeConstants.Elevator.ELEVATOR_P;
                Constants.Elevator.ELEVATOR_I = PracticeConstants.Elevator.ELEVATOR_I;
                Constants.Elevator.ELEVATOR_D = PracticeConstants.Elevator.ELEVATOR_D;
                Constants.Elevator.ELEVATOR_LOWER_LIMIT = PracticeConstants.Elevator.ELEVATOR_LOWER_LIMIT;
                Constants.Elevator.ELEVATOR_UPPER_LIMIT = PracticeConstants.Elevator.ELEVATOR_UPPER_LIMIT;
                Constants.Elevator.ELEVATOR_AMP = PracticeConstants.Elevator.ELEVATOR_AMP;
                Constants.Elevator.ELEVATOR_STOW = PracticeConstants.Elevator.ELEVATOR_STOW;

                Constants.IntakeCarriage.INTAKE_ID = PracticeConstants.IntakeCarriage.INTAKE_ID;
                Constants.IntakeCarriage.CARRIAGE_ID = PracticeConstants.IntakeCarriage.CARRIAGE_ID;
                Constants.IntakeCarriage.IDLE_INTAKE_SPEED = PracticeConstants.IntakeCarriage.IDLE_INTAKE_SPEED;
                Constants.IntakeCarriage.CARRIAGE_BEAM = PracticeConstants.IntakeCarriage.CARRIAGE_BEAM;
                Constants.IntakeCarriage.SHOOTER_BEAM = PracticeConstants.IntakeCarriage.SHOOTER_BEAM;
                Constants.IntakeCarriage.AMP_BEAM = PracticeConstants.IntakeCarriage.AMP_BEAM;

                Constants.Limelight.LIMELIGHT_P = PracticeConstants.Limelight.LIMELIGHT_P;
                Constants.Limelight.LIMELIGHT_I = PracticeConstants.Limelight.LIMELIGHT_I;
                Constants.Limelight.LIMELIGHT_D = PracticeConstants.Limelight.LIMELIGHT_D;
                Constants.Limelight.YAW_OFFSET = PracticeConstants.Limelight.YAW_OFFSET;

                Constants.Shooter.BOTTOM_SHOOTER_ID = PracticeConstants.Shooter.BOTTOM_SHOOTER_ID;
                Constants.Shooter.TOP_SHOOTER_ID = PracticeConstants.Shooter.TOP_SHOOTER_ID;
                Constants.Shooter.SHOOTER_CURRENT_LIMIT = PracticeConstants.Shooter.SHOOTER_CURRENT_LIMIT;
                Constants.Shooter.SHOOTER_INVERT = PracticeConstants.Shooter.SHOOTER_INVERT;
                Constants.Shooter.BASE_SHOOTER_SPEED = PracticeConstants.Shooter.BASE_SHOOTER_SPEED;
                Constants.Shooter.IDLE_SHOOTER_SPEED = PracticeConstants.Shooter.IDLE_SHOOTER_SPEED;
                Constants.Shooter.SHOOTER_ERROR_TOLERANCE = PracticeConstants.Shooter.SHOOTER_ERROR_TOLERANCE;
                Constants.Shooter.DISTANCE_MULTIPLIER = PracticeConstants.Shooter.DISTANCE_MULTIPLIER;
                Constants.Shooter.SHOOTER_P = PracticeConstants.Shooter.SHOOTER_P;
                Constants.Shooter.SHOOTER_I = PracticeConstants.Shooter.SHOOTER_I;
                Constants.Shooter.SHOOTER_D = PracticeConstants.Shooter.SHOOTER_D;
                Constants.Shooter.SHOOTER_FF = PracticeConstants.Shooter.SHOOTER_FF;
                Constants.Shooter.FEEDER_ID = PracticeConstants.Shooter.FEEDER_ID;
                Constants.Shooter.FEEDER_CURRENT_LIMIT = PracticeConstants.Shooter.FEEDER_CURRENT_LIMIT;
                Constants.Shooter.FEEDER_INVERT = PracticeConstants.Shooter.FEEDER_INVERT;

                Constants.Swerve.GYRO_OFFSET = PracticeConstants.Swerve.GYRO_OFFSET;
                Constants.Swerve.PIGEON_ID = PracticeConstants.Swerve.PIGEON_ID;
                Constants.Swerve.INVERT_GYRO = PracticeConstants.Swerve.INVERT_GYRO;
                Constants.Swerve.CHOSEN_MODULE = PracticeConstants.Swerve.CHOSEN_MODULE;
                Constants.Swerve.TRACK_WIDTH = PracticeConstants.Swerve.TRACK_WIDTH;
                Constants.Swerve.WHEEL_BASE = PracticeConstants.Swerve.WHEEL_BASE;
                Constants.Swerve.CENTER_TO_WHEEL = PracticeConstants.Swerve.CENTER_TO_WHEEL;
                Constants.Swerve.WHEEL_CIRCUMFERENCE = PracticeConstants.Swerve.WHEEL_CIRCUMFERENCE;
                Constants.Swerve.SWERVE_KINEMATICS = PracticeConstants.Swerve.SWERVE_KINEMATICS;
                Constants.Swerve.VOLTAGE_COMP = PracticeConstants.Swerve.VOLTAGE_COMP;
                Constants.Swerve.DRIVE_GEAR_RATIO = PracticeConstants.Swerve.DRIVE_GEAR_RATIO;
                Constants.Swerve.ANGLE_GEAR_RATIO = PracticeConstants.Swerve.ANGLE_GEAR_RATIO;
                Constants.Swerve.ANGLE_MOTOR_INVERT = PracticeConstants.Swerve.ANGLE_MOTOR_INVERT;
                Constants.Swerve.DRIVE_MOTOR_INVERT = PracticeConstants.Swerve.DRIVE_MOTOR_INVERT;
                Constants.Swerve.CANCODER_SENSOR_DIRECTION = PracticeConstants.Swerve.CANCODER_SENSOR_DIRECTION;
                Constants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION = PracticeConstants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
                Constants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION = PracticeConstants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION;
                Constants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = PracticeConstants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.OPEN_LOOP_RAMP = PracticeConstants.Swerve.OPEN_LOOP_RAMP;
                Constants.Swerve.CLOSED_LOOP_RAMP = PracticeConstants.Swerve.CLOSED_LOOP_RAMP;
                Constants.Swerve.ANGLE_P = PracticeConstants.Swerve.ANGLE_P;
                Constants.Swerve.ANGLE_I = PracticeConstants.Swerve.ANGLE_I;
                Constants.Swerve.ANGLE_D = PracticeConstants.Swerve.ANGLE_D;
                Constants.Swerve.ANGLE_F = PracticeConstants.Swerve.ANGLE_F;
                Constants.Swerve.DRIVE_S = PracticeConstants.Swerve.DRIVE_S;
                Constants.Swerve.DRIVE_V = PracticeConstants.Swerve.DRIVE_V;
                Constants.Swerve.DRIVE_A = PracticeConstants.Swerve.DRIVE_A;
                Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR = PracticeConstants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR;
                Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR = PracticeConstants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR;
                Constants.Swerve.ANGLE_CONVERSION_FACTOR = PracticeConstants.Swerve.ANGLE_CONVERSION_FACTOR;
                Constants.Swerve.MAX_SPEED = PracticeConstants.Swerve.MAX_SPEED;
                Constants.Swerve.MAX_ACCEL = PracticeConstants.Swerve.MAX_ACCEL;
                Constants.Swerve.MAX_ANGULAR_VELOCITY = PracticeConstants.Swerve.MAX_ANGULAR_VELOCITY;
                Constants.Swerve.ANGLE_NEUTRAL_MODE = PracticeConstants.Swerve.ANGLE_NEUTRAL_MODE;
                Constants.Swerve.DRIVE_NEUTRAL_MODE = PracticeConstants.Swerve.DRIVE_NEUTRAL_MODE;

                Constants.Swerve.Mod3.DRIVE_MOTOR_ID_3 = PracticeConstants.Swerve.Mod3.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod3.ANGLE_MOTOR_ID_3 = PracticeConstants.Swerve.Mod3.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod3.CANCODER_ID_3 = PracticeConstants.Swerve.Mod3.CANCODER_ID;
                Constants.Swerve.Mod3.ANGLE_OFFSET_3 = PracticeConstants.Swerve.Mod3.ANGLE_OFFSET;
                Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_3 = PracticeConstants.Swerve.Mod3.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod2.DRIVE_MOTOR_ID_2 = PracticeConstants.Swerve.Mod2.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod2.ANGLE_MOTOR_ID_2 = PracticeConstants.Swerve.Mod2.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod2.CANCODER_ID_2 = PracticeConstants.Swerve.Mod2.CANCODER_ID;
                Constants.Swerve.Mod2.ANGLE_OFFSET_2 = PracticeConstants.Swerve.Mod2.ANGLE_OFFSET;
                Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS_2 = PracticeConstants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod0.DRIVE_MOTOR_ID_0 = PracticeConstants.Swerve.Mod0.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod0.ANGLE_MOTOR_ID_0 = PracticeConstants.Swerve.Mod0.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod0.CANCODER_ID_0 = PracticeConstants.Swerve.Mod0.CANCODER_ID;
                Constants.Swerve.Mod0.ANGLE_OFFSET_0 = PracticeConstants.Swerve.Mod0.ANGLE_OFFSET;
                Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_0 = PracticeConstants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod1.DRIVE_MOTOR_ID_1 = PracticeConstants.Swerve.Mod1.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod1.ANGLE_MOTOR_ID_1 = PracticeConstants.Swerve.Mod1.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod1.CANCODER_ID_1 = PracticeConstants.Swerve.Mod1.CANCODER_ID;
                Constants.Swerve.Mod1.ANGLE_OFFSET_1 = PracticeConstants.Swerve.Mod1.ANGLE_OFFSET;
                Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS_1 = PracticeConstants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS;
            }
            case ROWDY -> {
                Constants.STICK_DEADBAND = RowdyConstants.STICK_DEADBAND;
                Constants.TRIGGER_DEADBAND = RowdyConstants.TRIGGER_DEADBAND;
                Constants.MINIMUM_ANGLE = RowdyConstants.MINIMUM_ANGLE;
                Constants.MAXIMUM_ANGLE = RowdyConstants.MAXIMUM_ANGLE;

                Constants.Auto.AUTO_DRIVE_P = RowdyConstants.Auto.AUTO_DRIVE_P;
                Constants.Auto.AUTO_DRIVE_I = RowdyConstants.Auto.AUTO_DRIVE_I;
                Constants.Auto.AUTO_DRIVE_D = RowdyConstants.Auto.AUTO_DRIVE_D;
                Constants.Auto.AUTO_ANGLE_P = RowdyConstants.Auto.AUTO_ANGLE_P;
                Constants.Auto.AUTO_ANGLE_I = RowdyConstants.Auto.AUTO_ANGLE_I;
                Constants.Auto.AUTO_ANGLE_D = RowdyConstants.Auto.AUTO_ANGLE_D;

                Constants.Angler.ANGLER_ID = RowdyConstants.Angler.ANGLER_ID;
                Constants.Angler.ANGLER_P = RowdyConstants.Angler.ANGLER_P;
                Constants.Angler.ANGLER_I = RowdyConstants.Angler.ANGLER_I;
                Constants.Angler.ANGLER_D = RowdyConstants.Angler.ANGLER_D;
                Constants.Angler.ANGLER_CURRENT_LIMIT = RowdyConstants.Angler.ANGLER_CURRENT_LIMIT;
                Constants.Angler.ANGLER_INVERT = RowdyConstants.Angler.ANGLER_INVERT;
                Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT = RowdyConstants.Angler.ANGLER_LOWER_LIMIT_SWITCH_PORT;
                Constants.Angler.ANGLER_UPPER_LIMIT_SWITCH_PORT = RowdyConstants.Angler.ANGLER_UPPER_LIMIT_SWTICH_PORT;
                Constants.Angler.ANGLER_LOWER_LIMIT = RowdyConstants.Angler.ANGLER_LOWER_LIMIT;
                Constants.Angler.ANGLER_UPPER_LIMIT = RowdyConstants.Angler.ANGLER_UPPER_LIMIT;
                Constants.Angler.UPPER_AIM_LIMIT = RowdyConstants.Angler.UPPER_AIM_LIMIT;
                Constants.Angler.UPPER_AIM_EQUATION = RowdyConstants.Angler::UPPER_AIM_EQUATION;
                Constants.Angler.TIGHT_AIM_EQUATION = RowdyConstants.Angler::TIGHT_AIM_EQUATION;

                Constants.Elevator.ELEVATOR_ID = RowdyConstants.Elevator.ELEVATOR_ID;
                Constants.Elevator.ELEVATOR_CURRENT_LIMIT = RowdyConstants.Elevator.ELEVATOR_CURRENT_LIMIT;
                Constants.Elevator.ELEVATOR_LIMIT_SWITCH = RowdyConstants.Elevator.ELEVATOR_LIMIT_SWITCH;
                Constants.Elevator.ELEVATOR_SERVO_PORT = RowdyConstants.Elevator.ELEVATOR_SERVO_PORT;
                Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS = RowdyConstants.Elevator.ELEVATOR_SERVO_CLAMPED_POS;
                Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS = RowdyConstants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS;
                Constants.Elevator.ELEVATOR_INVERT = RowdyConstants.Elevator.ELEVATOR_INVERT;
                Constants.Elevator.ELEVATOR_P = RowdyConstants.Elevator.ELEVATOR_P;
                Constants.Elevator.ELEVATOR_I = RowdyConstants.Elevator.ELEVATOR_I;
                Constants.Elevator.ELEVATOR_D = RowdyConstants.Elevator.ELEVATOR_D;
                Constants.Elevator.ELEVATOR_LOWER_LIMIT = RowdyConstants.Elevator.ELEVATOR_LOWER_LIMIT;
                Constants.Elevator.ELEVATOR_UPPER_LIMIT = RowdyConstants.Elevator.ELEVATOR_UPPER_LIMIT;
                Constants.Elevator.ELEVATOR_AMP = RowdyConstants.Elevator.ELEVATOR_AMP;
                Constants.Elevator.ELEVATOR_STOW = RowdyConstants.Elevator.ELEVATOR_STOW;

                Constants.IntakeCarriage.INTAKE_ID = RowdyConstants.IntakeCarriage.INTAKE_ID;
                Constants.IntakeCarriage.CARRIAGE_ID = RowdyConstants.IntakeCarriage.CARRIAGE_ID;
                Constants.IntakeCarriage.IDLE_INTAKE_SPEED = RowdyConstants.IntakeCarriage.IDLE_INTAKE_SPEED;
                Constants.IntakeCarriage.CARRIAGE_BEAM = RowdyConstants.IntakeCarriage.CARRIAGE_BEAM;
                Constants.IntakeCarriage.SHOOTER_BEAM = RowdyConstants.IntakeCarriage.SHOOTER_BEAM;
                Constants.IntakeCarriage.AMP_BEAM = RowdyConstants.IntakeCarriage.AMP_BEAM;

                Constants.Limelight.LIMELIGHT_P = RowdyConstants.Limelight.LIMELIGHT_P;
                Constants.Limelight.LIMELIGHT_I = RowdyConstants.Limelight.LIMELIGHT_I;
                Constants.Limelight.LIMELIGHT_D = RowdyConstants.Limelight.LIMELIGHT_D;
                Constants.Limelight.YAW_OFFSET = RowdyConstants.Limelight.YAW_OFFSET;

                Constants.Shooter.BOTTOM_SHOOTER_ID = RowdyConstants.Shooter.BOTTOM_SHOOTER_ID;
                Constants.Shooter.TOP_SHOOTER_ID = RowdyConstants.Shooter.TOP_SHOOTER_ID;
                Constants.Shooter.SHOOTER_CURRENT_LIMIT = RowdyConstants.Shooter.SHOOTER_CURRENT_LIMIT;
                Constants.Shooter.SHOOTER_INVERT = RowdyConstants.Shooter.SHOOTER_INVERT;
                Constants.Shooter.BASE_SHOOTER_SPEED = RowdyConstants.Shooter.BASE_SHOOTER_SPEED;
                Constants.Shooter.IDLE_SHOOTER_SPEED = RowdyConstants.Shooter.IDLE_SHOOTER_SPEED;
                Constants.Shooter.SHOOTER_ERROR_TOLERANCE = RowdyConstants.Shooter.SHOOTER_ERROR_TOLERANCE;
                Constants.Shooter.DISTANCE_MULTIPLIER = RowdyConstants.Shooter.DISTANCE_MULTIPLIER;
                Constants.Shooter.SHOOTER_P = RowdyConstants.Shooter.SHOOTER_P;
                Constants.Shooter.SHOOTER_I = RowdyConstants.Shooter.SHOOTER_I;
                Constants.Shooter.SHOOTER_D = RowdyConstants.Shooter.SHOOTER_D;
                Constants.Shooter.SHOOTER_FF = RowdyConstants.Shooter.SHOOTER_FF;
                Constants.Shooter.FEEDER_ID = RowdyConstants.Shooter.FEEDER_ID;
                Constants.Shooter.FEEDER_CURRENT_LIMIT = RowdyConstants.Shooter.FEEDER_CURRENT_LIMIT;
                Constants.Shooter.FEEDER_INVERT = RowdyConstants.Shooter.FEEDER_INVERT;

                Constants.Swerve.GYRO_OFFSET = RowdyConstants.Swerve.GYRO_OFFSET;
                Constants.Swerve.PIGEON_ID = RowdyConstants.Swerve.PIGEON_ID;
                Constants.Swerve.INVERT_GYRO = RowdyConstants.Swerve.INVERT_GYRO;
                Constants.Swerve.CHOSEN_MODULE = RowdyConstants.Swerve.CHOSEN_MODULE;
                Constants.Swerve.TRACK_WIDTH = RowdyConstants.Swerve.TRACK_WIDTH;
                Constants.Swerve.WHEEL_BASE = RowdyConstants.Swerve.WHEEL_BASE;
                Constants.Swerve.CENTER_TO_WHEEL = RowdyConstants.Swerve.CENTER_TO_WHEEL;
                Constants.Swerve.WHEEL_CIRCUMFERENCE = RowdyConstants.Swerve.WHEEL_CIRCUMFERENCE;
                Constants.Swerve.SWERVE_KINEMATICS = RowdyConstants.Swerve.SWERVE_KINEMATICS;
                Constants.Swerve.VOLTAGE_COMP = RowdyConstants.Swerve.VOLTAGE_COMP;
                Constants.Swerve.DRIVE_GEAR_RATIO = RowdyConstants.Swerve.DRIVE_GEAR_RATIO;
                Constants.Swerve.ANGLE_GEAR_RATIO = RowdyConstants.Swerve.ANGLE_GEAR_RATIO;
                Constants.Swerve.ANGLE_MOTOR_INVERT = RowdyConstants.Swerve.ANGLE_MOTOR_INVERT;
                Constants.Swerve.DRIVE_MOTOR_INVERT = RowdyConstants.Swerve.DRIVE_MOTOR_INVERT;
                Constants.Swerve.CANCODER_SENSOR_DIRECTION = RowdyConstants.Swerve.CANCODER_SENSOR_DIRECTION;
                Constants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION = RowdyConstants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION;
                Constants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION = RowdyConstants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION;
                Constants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = RowdyConstants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
                Constants.Swerve.OPEN_LOOP_RAMP = RowdyConstants.Swerve.OPEN_LOOP_RAMP;
                Constants.Swerve.CLOSED_LOOP_RAMP = RowdyConstants.Swerve.CLOSED_LOOP_RAMP;
                Constants.Swerve.ANGLE_P = RowdyConstants.Swerve.ANGLE_P;
                Constants.Swerve.ANGLE_I = RowdyConstants.Swerve.ANGLE_I;
                Constants.Swerve.ANGLE_D = RowdyConstants.Swerve.ANGLE_D;
                Constants.Swerve.ANGLE_F = RowdyConstants.Swerve.ANGLE_F;
                Constants.Swerve.DRIVE_S = RowdyConstants.Swerve.DRIVE_S;
                Constants.Swerve.DRIVE_V = RowdyConstants.Swerve.DRIVE_V;
                Constants.Swerve.DRIVE_A = RowdyConstants.Swerve.DRIVE_A;
                Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR = RowdyConstants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR;
                Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR = RowdyConstants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR;
                Constants.Swerve.ANGLE_CONVERSION_FACTOR = RowdyConstants.Swerve.ANGLE_CONVERSION_FACTOR;
                Constants.Swerve.MAX_SPEED = RowdyConstants.Swerve.MAX_SPEED;
                Constants.Swerve.MAX_ACCEL = RowdyConstants.Swerve.MAX_ACCEL;
                Constants.Swerve.MAX_ANGULAR_VELOCITY = RowdyConstants.Swerve.MAX_ANGULAR_VELOCITY;
                Constants.Swerve.ANGLE_NEUTRAL_MODE = RowdyConstants.Swerve.ANGLE_NEUTRAL_MODE;
                Constants.Swerve.DRIVE_NEUTRAL_MODE = RowdyConstants.Swerve.DRIVE_NEUTRAL_MODE;

                Constants.Swerve.Mod3.DRIVE_MOTOR_ID_3 = RowdyConstants.Swerve.Mod3.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod3.ANGLE_MOTOR_ID_3 = RowdyConstants.Swerve.Mod3.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod3.CANCODER_ID_3 = RowdyConstants.Swerve.Mod3.CANCODER_ID;
                Constants.Swerve.Mod3.ANGLE_OFFSET_3 = RowdyConstants.Swerve.Mod3.ANGLE_OFFSET;
                Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_3 = RowdyConstants.Swerve.Mod3.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod2.DRIVE_MOTOR_ID_2 = RowdyConstants.Swerve.Mod2.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod2.ANGLE_MOTOR_ID_2 = RowdyConstants.Swerve.Mod2.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod2.CANCODER_ID_2 = RowdyConstants.Swerve.Mod2.CANCODER_ID;
                Constants.Swerve.Mod2.ANGLE_OFFSET_2 = RowdyConstants.Swerve.Mod2.ANGLE_OFFSET;
                Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS_2 = RowdyConstants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod0.DRIVE_MOTOR_ID_0 = RowdyConstants.Swerve.Mod0.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod0.ANGLE_MOTOR_ID_0 = RowdyConstants.Swerve.Mod0.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod0.CANCODER_ID_0 = RowdyConstants.Swerve.Mod0.CANCODER_ID;
                Constants.Swerve.Mod0.ANGLE_OFFSET_0 = RowdyConstants.Swerve.Mod0.ANGLE_OFFSET;
                Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS_0 = RowdyConstants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS;

                Constants.Swerve.Mod1.DRIVE_MOTOR_ID_1 = RowdyConstants.Swerve.Mod1.DRIVE_MOTOR_ID;
                Constants.Swerve.Mod1.ANGLE_MOTOR_ID_1 = RowdyConstants.Swerve.Mod1.ANGLE_MOTOR_ID;
                Constants.Swerve.Mod1.CANCODER_ID_1 = RowdyConstants.Swerve.Mod1.CANCODER_ID;
                Constants.Swerve.Mod1.ANGLE_OFFSET_1 = RowdyConstants.Swerve.Mod1.ANGLE_OFFSET;
                Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS_1 = RowdyConstants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS;
            }
        }
    }
}
