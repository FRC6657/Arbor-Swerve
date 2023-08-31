package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CommandFactory {
    
    private final Drivetrain drivetrain;

    /*
     * Constructor for CommandFactory
     * 
     * The command factory is used to create commands that combine multiple subsystems.
     * 
     * @param _drivetrain The drivetrain subsystem
     */
    public CommandFactory(Drivetrain _drivetrain){
        drivetrain = _drivetrain;
    }

    /**
     * Creates a command to drive the robot using the swerve drive during teleop.
`    * All inputs are (-1,1) and scaled to the max speed.
     * Output is field relative
     */
    public Command TeleopSwerve(DoubleSupplier _xInput, DoubleSupplier _yInput, DoubleSupplier _rInput){
        return new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(_xInput.getAsDouble(), DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(_yInput.getAsDouble(), DriverConstants.kDriveDeadband),
                MathUtil.applyDeadband(_rInput.getAsDouble(), DriverConstants.kDriveDeadband),
                true, 
                RobotBase.isReal()
            ),
            drivetrain
        );
    }

    public Command AutoPath(String _pathName, PathConstraints _pathConstraints, HashMap<String, Command> _markers){

        //Split up path beteen markers
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(_pathName, _pathConstraints);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose, // Pose2d supplier
            drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.25, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            _markers,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);

    }

}
