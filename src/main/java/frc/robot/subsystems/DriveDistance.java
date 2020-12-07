package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveDistance extends CommandBase {

    private double dist;

    public DriveDistance(Drivetrain subsystem, double d){
        
        dist = d;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
        Robot.drivetrain.driveDistance(dist);

    }

    @Override
    public void execute(){

    }

}