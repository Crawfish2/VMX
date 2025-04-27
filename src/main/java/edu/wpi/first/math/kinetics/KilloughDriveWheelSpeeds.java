// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinetics;

/** Represents the wheel speeds for a killough drive drivetrain. */
public class KilloughDriveWheelSpeeds {
  /** Speed of the forward left wheel. */
  public double forwardLeftMetersPerSecond;

  /** Speed of the forward right wheel. */
  public double forwardRightMetersPerSecond;

  /** Speed of the back wheel. */
  public double backMetersPerSecond;

  /** Constructs a KilloughDriveWheelSpeeds with zeros for all member fields. */
  public KilloughDriveWheelSpeeds() {}

  /**
   * Constructs a KilloughDriveWheelSpeeds.
   *
   * @param forwardLeftMetersPerSecond Speed of the forward left wheel.
   * @param forwardRightMetersPerSecond Speed of the forward right wheel.
   * @param backMetersPerSecond Speed of the back wheel.
   */
  public KilloughDriveWheelSpeeds(
      double forwardLeftMetersPerSecond,
      double forwardRightMetersPerSecond,
      double backMetersPerSecond) {
    this.forwardLeftMetersPerSecond = forwardLeftMetersPerSecond;
    this.forwardRightMetersPerSecond = forwardRightMetersPerSecond;
    this.backMetersPerSecond = backMetersPerSecond;
  }

  /**
   * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
   * <p>
   * Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
   * above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
   * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
   * absolute threshold, while maintaining the ratio of speeds between wheels.
   *
   * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a wheel can reach.
   */
  public void desaturate(double attainableMaxSpeedMetersPerSecond) {
    double realMaxSpeed =
        Math.max(Math.abs(forwardLeftMetersPerSecond), Math.abs(forwardRightMetersPerSecond));
    realMaxSpeed = Math.max(realMaxSpeed, Math.abs(backMetersPerSecond));

    if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
      forwardLeftMetersPerSecond =
          forwardLeftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      forwardRightMetersPerSecond =
          forwardRightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
      backMetersPerSecond =
          backMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
    }
  }

  /**
   * Adds two KilloughDriveWheelSpeeds and returns the sum.
   * <p>
   * For example, KilloughDriveWheelSpeeds{1.0, 0.5, 2.0} + KilloughDriveWheelSpeeds{2.0, 1.5,
   * 0.5}
   * = KilloughDriveWheelSpeeds{3.0, 2.0, 2.5}
   *
   * @param other The KilloughDriveWheelSpeeds to add.
   * @return The sum of the KilloughDriveWheelSpeeds.
   */
  public KilloughDriveWheelSpeeds plus(KilloughDriveWheelSpeeds other) {
    return new KilloughDriveWheelSpeeds(
        forwardLeftMetersPerSecond + other.forwardLeftMetersPerSecond,
        forwardRightMetersPerSecond + other.forwardRightMetersPerSecond,
        backMetersPerSecond + other.backMetersPerSecond);
  }

  /**
   * Subtracts the other KilloughDriveWheelSpeeds from the current KilloughDriveWheelSpeeds and
   * returns the difference.
   * <p>
   * For example, KilloughDriveWheelSpeeds{5.0, 4.0, 6.0} - KilloughDriveWheelSpeeds{1.0, 2.0,
   * 3.0}
   * = KilloughDriveWheelSpeeds{4.0, 2.0, 3.0}
   *
   * @param other The KilloughDriveWheelSpeeds to subtract.
   * @return The difference between the two KilloughDriveWheelSpeeds.
   */
  public KilloughDriveWheelSpeeds minus(KilloughDriveWheelSpeeds other) {
    return new KilloughDriveWheelSpeeds(
        forwardLeftMetersPerSecond - other.forwardLeftMetersPerSecond,
        forwardRightMetersPerSecond - other.forwardRightMetersPerSecond,
        backMetersPerSecond - other.backMetersPerSecond);
  }

  /**
   * Returns the inverse of the current KilloughDriveWheelSpeeds. This is equivalent to negating
   * all
   * components of the KilloughDriveWheelSpeeds.
   *
   * @return The inverse of the current KilloughDriveWheelSpeeds.
   */
  public KilloughDriveWheelSpeeds unaryMinus() {
    return new KilloughDriveWheelSpeeds(
        -forwardLeftMetersPerSecond, -forwardRightMetersPerSecond, -backMetersPerSecond);
  }

  /**
   * Multiplies the KilloughDriveWheelSpeeds by a scalar and returns the new
   * KilloughDriveWheelSpeeds.
   * <p>
   * For example, KilloughDriveWheelSpeeds{2.0, 2.5, 3.0} * 2 = KilloughDriveWheelSpeeds{4.0, 5.0,
   * 6.0}
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled KilloughDriveWheelSpeeds.
   */
  public KilloughDriveWheelSpeeds times(double scalar) {
    return new KilloughDriveWheelSpeeds(
        forwardLeftMetersPerSecond * scalar,
        forwardRightMetersPerSecond * scalar,
        backMetersPerSecond * scalar);
  }

  /**
   * Divides the KilloughDriveWheelSpeeds by a scalar and returns the new
   * KilloughDriveWheelSpeeds.
   * <p>
   * For example, KilloughDriveWheelSpeeds{2.0, 2.5, 1.5} / 2 = KilloughDriveWheelSpeeds{1.0,
   * 1.25,
   * 0.75}
   *
   * @param scalar The scalar to divide by.
   * @return The scaled KilloughDriveWheelSpeeds.
   */
  public KilloughDriveWheelSpeeds div(double scalar) {
    return new KilloughDriveWheelSpeeds(
        forwardLeftMetersPerSecond / scalar,
        forwardRightMetersPerSecond / scalar,
        backMetersPerSecond / scalar);
  }

  @Override
  public String toString() {
    return String.format(
        "KilloughDriveWheelSpeeds(Forward Left: %.2f m/s, Forward Right: %.2f m/s, "
            + "Back: %.2f m/s)",
        forwardLeftMetersPerSecond, forwardRightMetersPerSecond, backMetersPerSecond);
  }
}
