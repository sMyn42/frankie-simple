package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends CommandBase {

    private double turning;

    public TurnDegrees(Drivetrain subsystem, double t){
        
        turning = t;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
        Robot.drivetrain.turnDegree(turning, true);

    }

    @Override
    public void execute(){

    }

}
