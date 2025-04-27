// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinetics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Class for killough drive odometry. Odometry allows you to track the robot's position on
 * the field
 * over a course of a match using readings from your killough wheel encoders.
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 */
public class KilloughDriveOdometry extends Odometry<KilloughDriveWheelPositions> {
  private static final Pose2d Pose2dZero = new Pose2d();

  /**
   * Constructs a KilloughDriveOdometry object.
   *
   * @param kinematics The killough drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public KilloughDriveOdometry(
      KilloughDriveKinematics kinematics,
      Rotation2d gyroAngle,
      KilloughDriveWheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }

  /**
   * Constructs a KilloughDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The killough drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The distances driven by each wheel.
   */
  public KilloughDriveOdometry(
      KilloughDriveKinematics kinematics,
      Rotation2d gyroAngle,
      KilloughDriveWheelPositions wheelPositions) {
    this(kinematics, gyroAngle, wheelPositions, Pose2dZero);
  }
}
