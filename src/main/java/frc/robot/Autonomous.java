package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveDistance;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(){
        addCommands(

            new DriveDistance(Robot.drivetrain, 10),

            new TurnDegrees(Robot.drivetrain, 90)
        );
    }
}