package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Configuration {

    public double auto_drive_p;
    public double auto_drive_i;
    public double auto_drive_d;
    public double auto_angle_p;
    public double auto_angle_i;
    public double auto_angle_d;

    public int angler_id;
    public double angler_p;
    public double angler_i;
    public double angler_d;
    public int angler_current_limit;
    public boolean angler_invert;
    public int angler_lower_limit_switch_port;
    public int angler_upper_limit_switch_port;
    public double angler_lower_limit;
    public double angler_upper_limit;
    public double upper_bound_limit;
    public double upper_bound_coefficient;
    public double upper_bound_series;
    public double tight_bound_coefficient;
    public double tight_bound_series;

    public int elevator_id;
    public int elevator_current_limit;
    public int elevator_limit_switch;
    public int elevator_servo_port;
    public double elevator_servo_clamped_pos;
    public double elevator_servo_unclamped_pos;
    public boolean elevator_invert;
    public double elevator_p;
    public double elevator_i;
    public double elevator_d;
    public double elevator_lower_limit;
    public double elevator_upper_limit;
    public double elevator_amp;
    public double elevator_stow;

    public int intake_id;
    public int carriage_id;
    public double idle_intake_speed;
    public int carriage_beam;
    public int shooter_beam;
    public int amp_beam;

    public double limelight_p;
    public double limelight_i;
    public double limelight_d;
    public double yaw_offset;

    public int bottom_shooter_id; //bottom wheel
    public int top_shooter_id; //top wheel
    public int shooter_current_limit;
    public boolean shooter_invert;
    //baselpeed in rpm
    public double base_shooter_speed;
    public double idle_shooter_speed;
    // +/-tconsidering if the shooter is up to speed
    public double shooter_error_tolerance;
    public double distance_multiplier;
    public double shooter_p; //was 0.003
    public double shooter_i;
    public double shooter_d;
    public double shooter_ff;
    public int feeder_id;
    public int feeder_current_limit;
    public boolean feeder_invert;

    public double gyro_offset;
    public int pigeon_id;
    public boolean invert_gyro; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)

    public COTSFalconSwerveConstants chosen_module;
    /* Drivetrain Constants */
    public double track_width;
    public double wheel_base;
    public double center_to_wheel;
    public double wheel_circumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public SwerveDriveKinematics swerve_kinematics;
    /* Swerve Voltage Compensation */
    public double voltage_comp;

    /* modus */
    public double drive_gear_ratio;
    public double angle_gear_ratio;

    public boolean angle_motor_invert;
    public boolean drive_motor_invert;

    /* Anglert */
    public SensorDirectionValue cancoder_sensor_direction;

    /* Swermiting */
    public int angle_continuous_supply_current_limit;
    public int angle_peak_supply_current_limit;
    public double angle_peak_supply_current_duration;
    public boolean angle_enable_supply_current_limit;

    public int drive_continuous_supply_current_limit;
    public int drive_peak_supply_current_limit;
    public double drive_peak_supply_current_duration;
    public boolean drive_enable_supply_current_limit;

    /*
     * thesused by the drive falcon to ramp in open loop and closed
     * loop
     * we fopen loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public double open_loop_ramp;
    public double closed_loop_ramp;

    /* anglalues */
    public double angle_p;
    public double angle_i;
    public double angle_d;
    public double angle_f;

    /* drivalues */
    public double drive_p;
    public double drive_i;
    public double drive_d;
    public double drive_f;

    /*
     * drivcterization values
     * divies by 12 to convert from volts to percent output for ctre
     */
    public double drive_s;
    public double drive_v;
    public double drive_a;

    /* drivrsion factors */
    public double drive_conversion_position_factor;
    public double drive_conversion_velocity_factor;
    public double angle_conversion_factor;

    /* swervalues */
    /** metd */
    public double max_speed;
    public double max_accel;

    /** radnd */
    public double max_angular_velocity;

    public IdleMode angle_neutral_mode;
    public IdleMode drive_neutral_mode;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public int drive_motor_id_3;
    public int angle_motor_id_3 ;
    public int cancoder_id_3;
    public Rotation2d angle_offset_3;
    public SwerveModuleConstants swerve_module_constants_3;

    /* Front Right Module - Module 1 */
    public int drive_motor_id_2;
    public int angle_motor_id_2;
    public int cancoder_id_2;
    public Rotation2d angle_offset_2;
    public SwerveModuleConstants swerve_module_constants_2;

    /* Back Left Module - Module 2 */
    public int drive_motor_id_0;
    public int angle_motor_id_0;
    public int cancoder_id_0;
    public Rotation2d angle_offset_0;
    public SwerveModuleConstants swerve_module_constants_0;

    /* Back Right Module - Module 3 */
    public int drive_motor_id_1;
    public int angle_motor_id_1;
    public int cancoder_id_1;
    public Rotation2d angle_offset_1;
    public SwerveModuleConstants swerve_module_constants_1;
}
