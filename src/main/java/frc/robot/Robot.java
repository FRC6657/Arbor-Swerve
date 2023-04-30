// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class Robot extends TimedRobot {

  private final Drivetrain mDrivetrain = new Drivetrain();

  private final CommandFactory mCommandFactory = new CommandFactory(mDrivetrain);

  private final CommandXboxController mDriveController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  private Command m_autonomousCommand;

  @Override
  public void robotInit(){
    
    mDrivetrain.setDefaultCommand(
      mCommandFactory.TeleopSwerve(
        () -> mDriveController.getLeftY(),
        () ->  mDriveController.getLeftX(),
        () -> mDriveController.getRightX()
      )
    );

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    mDrivetrain.updatePoseEstimator();

  }

  @Override
  public void simulationPeriodic() {
    mDrivetrain.simulate();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}
