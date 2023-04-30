package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CommandFactory {
    
    private final Drivetrain drivetrain;

    public CommandFactory(Drivetrain _drivetrain){
        drivetrain = _drivetrain;
    }

    public Command TeleopSwerve(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput){
        return new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(xInput.getAsDouble(), DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(yInput.getAsDouble(), DriverConstants.kDriveDeadband),
                MathUtil.applyDeadband(rInput.getAsDouble(), DriverConstants.kDriveDeadband),
                true, 
                RobotBase.isReal() ? true : false
            ),
            drivetrain
        );
    }

}
