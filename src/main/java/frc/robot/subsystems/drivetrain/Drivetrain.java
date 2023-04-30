// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // Create MAXSwerveModules
  private final SwerveModule mFrontLeft = SwerveModule.create(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final SwerveModule mFrontRight = SwerveModule.create(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset
  );

  private final SwerveModule mBackLeft = SwerveModule.create(
      DriveConstants.kBackLeftDrivingCanId,
      DriveConstants.kBackLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset
  );

  private final SwerveModule mBackRight = SwerveModule.create(
      DriveConstants.kBackRightDrivingCanId,
      DriveConstants.kBackRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset
  );

  private final List<SwerveModule> mModules = List.of(mFrontLeft, mFrontRight, mBackLeft, mBackRight);

  // The gyro sensor
  private final WPI_Pigeon2 mGyro = new WPI_Pigeon2(DriveConstants.kPigeon2CanId);
  private final BasePigeonSimCollection mGyroSim = mGyro.getSimCollection();

  // Slew rate filter variables for controlling lateral acceleration
  private double mCurrentRotation = 0.0;
  private double mCurrentTranslationDir = 0.0;
  private double mCurrentTranslationMag = 0.0;

  private SlewRateLimiter mMagLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter mRotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  // Pose Estimator class for tracking robot pose
  SwerveDrivePoseEstimator mPoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(mGyro.getAngle()),
    new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
    }, 
    new Pose2d(new Translation2d(4,4), new Rotation2d())
  );

  private final Field2d mField2d = new Field2d();
  private final FieldObject2d[] mModules2d = new FieldObject2d[mModules.size()];

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    for (int i = 0; i < mModules2d.length; i++) {
      mModules2d[i] = mField2d.getObject("module-" + i);
    }

    SmartDashboard.putData("Field", mField2d);

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {

    mPoseEstimator.resetPosition(
      Rotation2d.fromDegrees(mGyro.getAngle()),
      new SwerveModulePosition[] {
          mFrontLeft.getPosition(),
          mFrontRight.getPosition(),
          mBackLeft.getPosition(),
          mBackRight.getPosition()
      },
      pose
    );
  }

  public void updatePoseEstimator(){
    mPoseEstimator.update(
      Rotation2d.fromDegrees(mGyro.getAngle()),
      new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
      }
    );

    mField2d.setRobotPose(getPose());

    for (int i = 0; i < mModules.size(); i++) {
      var transform = new Transform2d(DriveConstants.kModuleOffset[i], mModules.get(i).getPosition().angle);
      mModules2d[i].setPose(getPose().transformBy(transform));
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    //This method is for joystick values, so scale values just incase.
    xSpeed = MathUtil.clamp(xSpeed, -1, 1);
    ySpeed = MathUtil.clamp(ySpeed, -1, 1);
    rot = MathUtil.clamp(rot, -1, 1);

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {

      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      //Rate Limit the Magnitude
      mCurrentTranslationMag = mMagLimiter.calculate(inputTranslationMag);
      mCurrentTranslationDir = inputTranslationDir;
      
      //Convert from polar back to XY
      xSpeedCommanded = mCurrentTranslationMag * Math.cos(mCurrentTranslationDir);
      ySpeedCommanded = mCurrentTranslationMag * Math.sin(mCurrentTranslationDir);

      mCurrentRotation = mRotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      mCurrentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = mCurrentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(mGyro.getAngle()))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    SmartDashboard.putNumber("Test", xSpeed);

    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mBackLeft.setDesiredState(swerveModuleStates[2]);
    mBackRight.setDesiredState(swerveModuleStates[3]);
  
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeft.setDesiredState(desiredStates[0]);
    mFrontRight.setDesiredState(desiredStates[1]);
    mBackLeft.setDesiredState(desiredStates[2]);
    mBackRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mBackLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mBackRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(mGyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return mGyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void simulate(){
    SmartDashboard.putData("Module 1", mFrontLeft);
    mGyroSim.addHeading(Units.radiansToDegrees(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) * 0.02);
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      mFrontLeft.getState(),
      mFrontRight.getState(),
      mBackLeft.getState(),
      mBackRight.getState(),
      
    };
  }

}
