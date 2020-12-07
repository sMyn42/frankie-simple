package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.operator.controllers.BionicF310;

public class Drivetrain extends SubsystemBase {

    private final int TICKS_PER_ROTATION = 90; //prev. 1440 //TODO fix value 
    private final int TIMEOUT_MS = 200; // The amount of time the Roborio watis before it sends a failed signal back to
                                       // the Driver Station
    private final double MOTOR_DEADBAND_PERCENTAGE = 0.001;
    private final double NOMINAL_OUTPUT = 0;
    private final double PEAK_OUTPUT = 1;
    private final double WHEEL_CIRCUMFERENCE_INCHES = (6 * Math.PI); //TODO fix value
    private final double HALF_WHEELBASE_INCHES = 10; //TODO fix value
    private final double REVS_PER_NINETY = (Math.PI/2 * HALF_WHEELBASE_INCHES)/WHEEL_CIRCUMFERENCE_INCHES;
    
    private final double move_kP = 0.25; // A gain of 0.25 maximizes the motor output when the error is one revolution.
    private final double move_kI = 0;
    private final double move_kD = 0;

    private CANSparkMax left, right;
    private Encoder left_e, right_e;
    private DifferentialDrive diffDrive;
    private PIDController pidController;

    public Drivetrain(int id_l, int id_r) {
    
        left = new CANSparkMax(id_l, MotorType.kBrushless); //TODO is it brushless?
        left = new CANSparkMax(id_r, MotorType.kBrushless); //TODO is it brushless?
    
        left.setInverted(false);
        right.setInverted(true);

        diffDrive = new DifferentialDrive(left, right);
        diffDrive.setRightSideInverted(false);

        {
            // Configure Talon SRXs

            // Reset Factory Defaults
            CANPIDController pid_l = left.getPIDController();
            CANPIDController pid_r = right.getPIDController();

            // Config the encoders
            Encoder left_e = new Encoder(0, 1);
            Encoder right_e = new Encoder(2, 3);

            right_e.setReverseDirection(true);

            left_e.setDistancePerPulse(1./TICKS_PER_ROTATION);
            right_e.setDistancePerPulse(1./TICKS_PER_ROTATION);

            pid_l.setP(0.25);
            pid_l.setI(0);
            pid_l.setD(0);

            pid_r.setP(0.25);
            pid_r.setI(0);
            pid_r.setD(0);
        }

    }

    //Make the robot move and turn. Used for manual control
    public void driveManual(double left_speed, double right_speed){
        diffDrive.tankDrive(left_speed, right_speed);
    }
    
    public void driveDistance(double distance) {
        
        left_e.reset();
        right_e.reset();

        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_INCHES; //This takes the amount of feet to move, and returns the amount of wheel rotations
        double lCurrentEncoderPos = left_e.getDistance(); //This returns the amount of ticks the encoder is on
        double rCurrentEncoderPos = right_e.getDistance(); //This returns the amount of ticks the encoder is on
        double lTargetEncoderPos = lCurrentEncoderPos + totalRevolutions; 
        //double rTargetEncoderPos = rCurrentEncoderPos + totalRevolutions; 

        while(Math.abs(left_e.getDistance() - lTargetEncoderPos) > 0.05){
            left.set(0.6);
            right.set(0.6);
        }

    }

    public void turnDegree(double turn, boolean isLeft) {

        left_e.reset();
        right_e.reset();

        double totalRevolutions = REVS_PER_NINETY * turn/90; //This takes the amount of feet to move, and returns the amount of wheel rotations
        double lCurrentEncoderPos = left_e.getDistance(); //This returns the amount of ticks the encoder is on
        double rCurrentEncoderPos = right_e.getDistance(); //This returns the amount of ticks the encoder is on
        double lTargetEncoderPos; 
        double rTargetEncoderPos;
        
        
        //TODO see if we can implement motion magic control
        
        if(isLeft){
            
            lTargetEncoderPos = lCurrentEncoderPos - totalRevolutions;
            rTargetEncoderPos = rCurrentEncoderPos + totalRevolutions;

            while(Math.abs(left_e.getDistance() - lTargetEncoderPos) > 0.05){
                left.set(-0.6);
                right.set(0.6);
            }

        } else {
            
            lTargetEncoderPos = lCurrentEncoderPos + totalRevolutions;
            rTargetEncoderPos = rCurrentEncoderPos - totalRevolutions;

            while(Math.abs(right_e.getDistance() - rTargetEncoderPos) > 0.05){
                left.set(0.6);
                right.set(-0.6);
            }
        
        }

    }

    @Override
    public void periodic() {

        
    
    }



}