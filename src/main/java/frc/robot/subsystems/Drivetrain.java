package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax left, right;
    private DifferentialDrive diffDrive;

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