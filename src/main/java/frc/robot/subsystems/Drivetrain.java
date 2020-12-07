package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.operator.controllers.BionicF310;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax left, right;
    private DifferentialDrive diffDrive;
    private PIDController pidController;

    public Drivetrain(int l, int r) {
        
        left = new CANSparkMax(l, MotorType.kBrushless); //TODO is it brushless?
        left = new CANSparkMax(r, MotorType.kBrushless); //TODO is it brushless?

        left.setInverted(false);
        right.setInverted(true);

        diffDrive = new DifferentialDrive(left, right);
        diffDrive.setRightSideInverted(false);

    }
    
    public void driveManual(double speed, double rotation){
        diffDrive.arcadeDrive(speed, rotation);
    }


    @Override
    public void periodic() {
    
    }

}