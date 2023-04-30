package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CommandFactory {
    
    private final Drivetrain drivetrain;

    public CommandFactory(Drivetrain _drivetrain){
        drivetrain = _drivetrain;
    }

    public Command TeleopSwerve(double xInput, double yInput, double rInput){
        return new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(xInput, DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(yInput, DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rInput, DriverConstants.kDriveDeadband),
                true, 
                true
            ),
            drivetrain
        );
    }

}
