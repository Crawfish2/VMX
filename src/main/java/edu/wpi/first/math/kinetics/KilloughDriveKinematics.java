// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinetics;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import org.ejml.simple.SimpleMatrix;

/**
 * Helper class that converts a chassis velocity (vx, vy, and omega components) into individual
 * wheel speeds for a killough drive.
 * <p>
 * The inverse kinematics (converting from a desired chassis velocity to individual wheel speeds)
 * uses the relative locations and orientations of the wheels with respect to the center of
 * rotation. The center of rotation for inverse kinematics is also variable. This means that you can
 * set your center of rotation in a corner of the robot to perform special evasion maneuvers.
 * <p>
 * Forward kinematics (converting an array of wheel speeds into the overall chassis motion) is
 * performs the exact opposite of what inverse kinematics does.
 * <p>
 * The inverse kinematics matrix is derived from the formula:
 * v_i = [ cos(theta_i) sin(theta_i) (x_i*cos(theta_i) + y_i*sin(theta_i)) ] * [vx vy omega]^T
 * where (x_i, y_i) is the position and theta_i is the angle of wheel i relative to the center of
 * rotation.
 * <p>
 * Forward kinematics is derived by taking the inverse of the inverse kinematics matrix.
 */
public class KilloughDriveKinematics
    implements Kinematics<KilloughDriveWheelSpeeds, KilloughDriveWheelPositions> {
  private final SimpleMatrix m_inverseKinematics;
  private final SimpleMatrix m_forwardKinematics;

  private final Translation2d m_forwardLeftWheelPos;
  private final Translation2d m_forwardRightWheelPos;
  private final Translation2d m_backWheelPos;
  private final double m_forwardLeftWheelAngleRad;
  private final double m_forwardRightWheelAngleRad;
  private final double m_backWheelAngleRad;

  private static final Translation2d Translation2dZero = new Translation2d();
  private Translation2d m_prevCoR = Translation2dZero;

  /**
   * Constructs a killough drive kinematics object.
   *
   * @param forwardLeftWheelPos The location of the forward-left wheel relative to the physical
   *        center of the robot.
   * @param forwardRightWheelPos The location of the forward-right wheel relative to the physical
   *        center of the robot.
   * @param backWheelPos The location of the back wheel relative to the physical center of the
   *        robot.
   * @param forwardLeftWheelAngleRad The angle of the forward-left wheel (radians).
   * @param forwardRightWheelAngleRad The angle of the forward-right wheel (radians).
   * @param backWheelAngleRad The angle of the back wheel (radians).
   */
  public KilloughDriveKinematics(
      Translation2d forwardLeftWheelPos,
      Translation2d forwardRightWheelPos,
      Translation2d backWheelPos,
      double forwardLeftWheelAngleRad,
      double forwardRightWheelAngleRad,
      double backWheelAngleRad) {
    m_forwardLeftWheelPos = forwardLeftWheelPos;
    m_forwardRightWheelPos = forwardRightWheelPos;
    m_backWheelPos = backWheelPos;

    m_forwardLeftWheelAngleRad = forwardLeftWheelAngleRad;
    m_forwardRightWheelAngleRad = forwardRightWheelAngleRad;
    m_backWheelAngleRad = backWheelAngleRad;

    m_inverseKinematics = new SimpleMatrix(3, 3);

    setInverseKinematics(forwardLeftWheelPos, forwardRightWheelPos, backWheelPos);
    m_forwardKinematics = m_inverseKinematics.pseudoInverse();
  }

  /**
   * Construct inverse kinematics matrix from wheel locations and angles relative to a center of
   * rotation.
   *
   * @param forwardLeftWheelPosCoR The location of the forward-left wheel relative to the center of
   *        rotation.
   * @param forwardRightWheelPosCoR The location of the forward-right wheel relative to the center
   *        of rotation.
   * @param backWheelPosCoR The location of the back wheel relative to the center of rotation.
   */
  private void setInverseKinematics(
      Translation2d forwardLeftWheelPosCoR,
      Translation2d forwardRightWheelPosCoR,
      Translation2d backWheelPosCoR) {
    // Forward Left Wheel
    double xFL = Math.cos(m_forwardLeftWheelAngleRad);
    double yFL = Math.sin(m_forwardLeftWheelAngleRad);
    double rVFL = forwardLeftWheelPosCoR.getNorm();
    m_inverseKinematics.setRow(0, 0, xFL, yFL, rVFL);

    // Forward Right Wheel
    double xFR = Math.cos(m_forwardRightWheelAngleRad);
    double yFR = Math.sin(m_forwardRightWheelAngleRad);
    double rVFR = forwardRightWheelPosCoR.getNorm();
    m_inverseKinematics.setRow(1, 0, xFR, yFR, rVFR);

    // Back Wheel
    double xB = Math.cos(m_backWheelAngleRad);
    double yB = Math.sin(m_backWheelAngleRad);
    double rVB = backWheelPosCoR.getNorm();
    m_inverseKinematics.setRow(2, 0, xB, yB, rVB);
  }

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired chassis velocity. This
   * method is often used to convert joystick values into wheel speeds.
   * <p>
   * This function also supports variable centers of rotation. During normal operations, the
   * center of rotation is usually the same as the physical center of the robot; therefore, the
   * argument is defaulted to that use case. However, if you wish to change the center of rotation
   * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *        rotation at one corner of the robot and provide a chassis speed that only has an omega
   *        component, the robot will rotate around that corner.
   * @return The wheel speeds. Use caution because they are not normalized. Use the {@link
   *         KilloughDriveWheelSpeeds#desaturate(double)} function to rectify this issue.
   */
  public KilloughDriveWheelSpeeds toWheelSpeeds(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
    // We have a new center of rotation. We need to compute the matrix again.
    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      // Calculate wheel positions relative to the new center of rotation
      Translation2d fl = m_forwardLeftWheelPos.minus(centerOfRotationMeters);
      Translation2d fr = m_forwardRightWheelPos.minus(centerOfRotationMeters);
      Translation2d bk = m_backWheelPos.minus(centerOfRotationMeters);

      setInverseKinematics(fl, fr, bk);
      m_prevCoR = centerOfRotationMeters;
      // Forward kinematics is invalid with custom CoR, but difficult to recalculate
      // m_forwardKinematics = m_inverseKinematics.invert(); // Consider if needed
    }

    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    chassisSpeedsVector.setColumn(
        0,
        0,
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);

    var wheelsVector = m_inverseKinematics.mult(chassisSpeedsVector);
    return new KilloughDriveWheelSpeeds(
        wheelsVector.get(0, 0),
        wheelsVector.get(1, 0),
        wheelsVector.get(2, 0));
  }

  /**
   * Performs forward kinematics to return the resulting chassis state from the given wheel speeds.
   * This method is often used for odometry -- determining the robot's position on the field using
   * data from the real-world speed of each wheel on the robot.
   *
   * @param wheelSpeeds The current killough drive wheel speeds.
   * @return The resulting chassis speed.
   */
  @Override
  public ChassisSpeeds toChassisSpeeds(KilloughDriveWheelSpeeds wheelSpeeds) {
    var wheelSpeedsVector = new SimpleMatrix(3, 1);
    wheelSpeedsVector.setColumn(
        0,
        0,
        wheelSpeeds.forwardLeftMetersPerSecond,
        wheelSpeeds.forwardRightMetersPerSecond,
        wheelSpeeds.backMetersPerSecond);
    var chassisSpeedsVector = m_forwardKinematics.mult(wheelSpeedsVector);

    return new ChassisSpeeds(
        chassisSpeedsVector.get(0, 0),
        chassisSpeedsVector.get(1, 0),
        chassisSpeedsVector.get(2, 0));
  }

  /**
   * Performs inverse kinematics. See {@link #toWheelSpeeds(ChassisSpeeds, Translation2d)} for more
   * information.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @return The wheel speeds.
   */
  @Override
  public KilloughDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return toWheelSpeeds(chassisSpeeds, Translation2dZero);
  }

  /**
   * Performs forward kinematics to return the resulting Twist2d from the given change in wheel
   * positions. This method is often used for odometry -- determining the robot's position on the
   * field using changes in the distance driven by each wheel on the robot.
   *
   * @param start The starting distances driven by the wheels.
   * @param end The ending distances driven by the wheels.
   * @return The resulting Twist2d in the robot's movement.
   */
  @Override
  public Twist2d toTwist2d(
      KilloughDriveWheelPositions start, KilloughDriveWheelPositions end) {
    var wheelDeltasVector = new SimpleMatrix(3, 1);
    wheelDeltasVector.setColumn(
        0,
        0,
        end.forwardLeftMeters - start.forwardLeftMeters,
        end.forwardRightMeters - start.forwardRightMeters,
        end.backMeters - start.backMeters);
    var twist = m_forwardKinematics.mult(wheelDeltasVector);
    return new Twist2d(twist.get(0, 0), twist.get(1, 0), twist.get(2, 0));
  }

  /**
   * Performs forward kinematics to return the resulting Twist2d from the given wheel deltas. This
   * method is often used for odometry -- determining the robot's position on the field using
   * changes in the distance driven by each wheel on the robot.
   *
   * @param wheelDeltas The distances driven by each wheel.
   * @return The resulting Twist2d.
   */
  public Twist2d toTwist2d(KilloughDriveWheelPositions wheelDeltas) {
    var wheelDeltasVector = new SimpleMatrix(3, 1);
    wheelDeltasVector.setColumn(
        0,
        0,
        wheelDeltas.forwardLeftMeters,
        wheelDeltas.forwardRightMeters,
        wheelDeltas.backMeters);
    var twist = m_forwardKinematics.mult(wheelDeltasVector);
    return new Twist2d(twist.get(0, 0), twist.get(1, 0), twist.get(2, 0));
  }

  @Override
  public KilloughDriveWheelPositions copy(KilloughDriveWheelPositions positions) {
    return new KilloughDriveWheelPositions(
        positions.forwardLeftMeters,
        positions.forwardRightMeters,
        positions.backMeters);
  }

  @Override
  public void copyInto(KilloughDriveWheelPositions positions,
      KilloughDriveWheelPositions output) {
    output.forwardLeftMeters = positions.forwardLeftMeters;
    output.forwardRightMeters = positions.forwardRightMeters;
    output.backMeters = positions.backMeters;
  }

  @Override
  public KilloughDriveWheelPositions interpolate(
      KilloughDriveWheelPositions startValue, KilloughDriveWheelPositions endValue, double t) {
    return startValue.interpolate(endValue, t);
  }
}
