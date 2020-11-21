package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private final int TICKS_PER_ROTATION = 1440;
    private final int TIMEOUT_MS = 30; // The amount of time the Roborio watis before it sends a failed signal back to
                                       // the Driver Station
    private final int PID_INDEX = 0; // The index for witch pid index to use (0 for primary, 1 for auxilliary)
    private final double MOTOR_DEADBAND_PERCENTAGE = 0.001;
    private final double NOMINAL_OUTPUT = 0;
    private final double NOMINAL_OUTPUT_REVERSE = -NOMINAL_OUTPUT; 
    private final double PEAK_OUTPUT = 1;
    private final double PEAK_OUTPUT_REVERSE = -PEAK_OUTPUT;
    private final double WHEEL_CIRCUMFERENCE_FEET = (6 * Math.PI) / 12;
    private final int SENSOR_POS = 0;
    private final double SENSOR_UNITS = 0;
    

    private final double K_P = 0.25; // A gain of 0.25 maximizes the motor output when the error is one revolution.
    private final double K_I = 0;
    private final double K_D = 0;

    private double length;
    private double width;
    private WPI_TalonFX left_front, left_back, right_front, right_back;
    private SpeedController m_left, m_right;
    private SpeedControllerGroup left_side, right_side;
    private DifferentialDrive diffDrive;
    private PIDController pidController;

    public Drivetrain(double l, double w, int idbl, int idfl, int idfr, int idbr) {
        this.length = l;
        this.width = w;
        this.left_front = new WPI_TalonFX(idfl);
        this.right_front = new WPI_TalonFX(idfr);
        this.right_back = new WPI_TalonFX(idbr);
        this.left_back = new WPI_TalonFX(idbl);
        
        this.left_side = new SpeedControllerGroup(left_front, left_back);
        this.right_side = new SpeedControllerGroup(right_front, right_back);
        this.diffDrive = new DifferentialDrive(left_side, right_side);

        

        // Configure Talon FXs

        // Reset Factory Defaults
        left_back.configFactoryDefault();
        left_front.configFactoryDefault();
        right_back.configFactoryDefault();
        right_front.configFactoryDefault();

        // Config the encoder to be the integrated encoder in the TalonFX
        left_back.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        left_front.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        right_back.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        right_front.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);

        // Set deadbands to the default value.
        // This configures the sensitivity of the motors.
        left_back.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        left_front.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        right_back.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        right_front.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);

        // Invert one side of the drivetrain to ensure that positive values indicate
        // forward motion.

        // left_side.setInverted(true); right_side.setInverted(false);

        left_back.setInverted(TalonFXInvertType.CounterClockwise);
        left_front.setInverted(TalonFXInvertType.CounterClockwise);
        right_back.setInverted(TalonFXInvertType.Clockwise);
        right_front.setInverted(TalonFXInvertType.Clockwise);

        // Sets the minimum motor output.
        // Also sets the maximum motor output.
        {
            left_front.configPeakOutputForward(PEAK_OUTPUT);
            left_front.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            left_front.configNominalOutputForward(NOMINAL_OUTPUT);
            left_front.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            left_back.configPeakOutputForward(PEAK_OUTPUT);
            left_back.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            left_back.configNominalOutputForward(NOMINAL_OUTPUT);
            left_back.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            right_front.configPeakOutputForward(PEAK_OUTPUT);
            right_front.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            right_front.configNominalOutputForward(NOMINAL_OUTPUT);
            right_front.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            right_back.configPeakOutputForward(PEAK_OUTPUT);
            right_back.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            right_back.configNominalOutputForward(NOMINAL_OUTPUT);
            right_back.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);
        }
        
        // Sets the P constant given the parameter
        left_front.config_kP(PID_INDEX, K_P);
        left_back.config_kP(PID_INDEX, K_P);
        right_front.config_kP(PID_INDEX, K_P);
        right_back.config_kP(PID_INDEX, K_P);

        // Sets the D constant given the parameter
        left_front.config_kI(PID_INDEX, K_I);
        left_back.config_kI(PID_INDEX, K_I);
        right_front.config_kI(PID_INDEX, K_I);
        right_back.config_kI(PID_INDEX, K_I);

        // Sets the D constant given the parameter
        left_front.config_kD(PID_INDEX, K_D);
        left_back.config_kD(PID_INDEX, K_D);
        right_front.config_kD(PID_INDEX, K_D);
        right_back.config_kD(PID_INDEX, K_D);

        
        //Sets the sensor position to 0
        left_front.setSelectedSensorPosition(SENSOR_POS);
        left_back.setSelectedSensorPosition(SENSOR_POS);
        right_front.setSelectedSensorPosition(SENSOR_POS);
        right_back.setSelectedSensorPosition(SENSOR_POS);

        //Sets acceleration adndnnd cruise velocity << motion magic
        /*
        * left_front.configMotionCruiseVelocity(SENSOR_UNITS, TIMEOUT_MS);
        * left_back.configMotionCruiseVelocity(SENSOR_UNITS, TIMEOUT_MS);
        * right_front.configMotionCruiseVelocity(SENSOR_UNITS, TIMEOUT_MS);
        * right_back.configMotionCruiseVelocity(SENSOR_UNITS, TIMEOUT_MS);

        * left_front.configMotionAcceleration(sensorUnitsPer100msPerSec)
        */


    }

    /*
     * public void driveForward(double speed) { diffDrive.arcadeDrive(speed, 0);
     * left_front.configFactoryDefault(); }
     * 
     * // pid values Branch: Drivetrain Motion profiling, File; DrivetrainSubsystem.
     * public void driveFeet(double speed, double distance) {
     * 
     * driveForward(speed);
     * 
     * }
     */

    //Make the robot move and turn. Used for manual control
    public void driveManual(double speed, double rotation){
        diffDrive.arcadeDrive(speed, rotation);
    }
    
    //Make the robot drive a certain amout of feet forward along a given arc
    public void driveDistance(double speed, double rotation_deg, double distance) { // Distance is in feet
        
        // Create known arc.
        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_FEET;
        double ticksToMove = totalRevolutions * TICKS_PER_ROTATION;
        
    }
    
    public void driveDistance(double speed, double distance) {

        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_FEET; //This takes the amount of feet to move, and returns the amount of wheel rotations
        int ticksToMove = (int)(totalRevolutions * TICKS_PER_ROTATION); //This takes the amount of wheel rotations and returns the amount of ticks to move
        int currentEncoderPos = left_front.getSelectedSensorPosition(); //This returns the amount of ticks the encoder is on
        
        //Make the left back & right back motors follow the left front & right front motors
        left_back.follow(left_front);
        right_back.follow(right_front);
        
        //TODO see if we can implement motion magic control
        //TODO resolve conflict between speed controller groups and following
        
        left_front.set(ControlMode.Position, currentEncoderPos + ticksToMove);
        right_front.set(ControlMode.Position, currentEncoderPos + ticksToMove);

    }

}