// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

// 元のコード: Original source:
// https://github.com/wpilibsuite/allwpilib/blob/031962608bfac046c5ce7ecde8a31501f9a015c5/wpilibj/src/main/java/edu/wpi/first/wpilibj/PS4Controller.java

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

/**
 * Handle input from PS4 controllers connected to the Driver Station.
 * <p>
 * This class handles PS4 input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 */
public class PS4Controller extends GenericHID {
  /**
   * Construct an instance of a device.
   *
   * @param port The port index on the Driver Station that the device is plugged into.
   */
  public PS4Controller(int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  /** Represents a digital button on a PS4Controller. */
  public enum Button {
    kSquare(1), kCross(2), kCircle(3), kTriangle(4), kL1(5), kR1(6), kL2(7), kR2(8), kShare(
        9), kOptions(10), kL3(11), kR3(12), kPS(13), kTouchpad(14);

    public final int value;

    Button(int index) {
      this.value = index;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and if not the touchpad append `Button`.
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (this == kTouchpad) {
        return name;
      }
      return name + "Button";
    }
  }

  /** Represents an axis on a PS4Controller. */
  public enum Axis {
    kLeftX(0), kLeftY(1), kRightX(2), kRightY(5), kL2(3), kR2(4);

    public final int value;

    Axis(int index) {
      value = index;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if one of L2/R2 append `Axis`.
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("2")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * @deprecated 代わりに、getLeftX()か、getRightX()を使うこと
   */
  @Override
  @Deprecated
  public double getX(Hand hand) {
    return hand == Hand.kLeft ? getLeftX() : getRightX();
  }

  /**
   * @deprecated 代わりに、getLeftY()か、getRightY()を使うこと
   */
  @Override
  @Deprecated
  public double getY(Hand hand) {
    return hand == Hand.kLeft ? getLeftY() : getRightY();
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return the axis value.
   */
  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return the axis value.
   */
  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return the axis value.
   */
  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return the axis value.
   */
  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  /**
   * Get the L2 axis value of the controller. Note that this axis is bound to the range of [0, 1]
   * as
   * opposed to the usual [-1, 1].
   *
   * @return the axis value.
   */
  public double getL2Axis() {
    return getRawAxis(Axis.kL2.value);
  }

  /**
   * Get the R2 axis value of the controller. Note that this axis is bound to the range of [0, 1]
   * as
   * opposed to the usual [-1, 1].
   *
   * @return the axis value.
   */
  public double getR2Axis() {
    return getRawAxis(Axis.kR2.value);
  }

  /**
   * Read the value of the left trigger button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL2Button() {
    return getRawButton(Button.kL2.value);
  }

  /**
   * Read the value of the right trigger button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getR2Button() {
    return getRawButton(Button.kR2.value);
  }

  /**
   * Whether the L2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL2ButtonPressed() {
    return getRawButtonPressed(Button.kL2.value);
  }

  /**
   * Whether the R2 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getR2ButtonPressed() {
    return getRawButtonPressed(Button.kR2.value);
  }

  /**
   * Whether the L2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL2ButtonReleased() {
    return getRawButtonReleased(Button.kL2.value);
  }

  /**
   * Whether the R2 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getR2ButtonReleased() {
    return getRawButtonReleased(Button.kR2.value);
  }

  /**
   * Read the value of the L1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL1Button() {
    return getRawButton(Button.kL1.value);
  }

  /**
   * Read the value of the R1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getR1Button() {
    return getRawButton(Button.kR1.value);
  }

  /**
   * Whether the L1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL1ButtonPressed() {
    return getRawButtonPressed(Button.kL1.value);
  }

  /**
   * Whether the R1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getR1ButtonPressed() {
    return getRawButtonPressed(Button.kR1.value);
  }

  /**
   * Whether the L1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL1ButtonReleased() {
    return getRawButtonReleased(Button.kL1.value);
  }

  /**
   * Whether the R1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getR1ButtonReleased() {
    return getRawButtonReleased(Button.kR1.value);
  }

  /**
   * Read the value of the L3 button (pressing the left analog stick) on the controller.
   *
   * @return The state of the button.
   */
  public boolean getL3Button() {
    return getRawButton(Button.kL3.value);
  }

  /**
   * Read the value of the R3 button (pressing the right analog stick) on the controller.
   *
   * @return The state of the button.
   */
  public boolean getR3Button() {
    return getRawButton(Button.kR3.value);
  }

  /**
   * Whether the L3 (left stick) button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getL3ButtonPressed() {
    return getRawButtonPressed(Button.kL3.value);
  }

  /**
   * Whether the R3 (right stick) button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getR3ButtonPressed() {
    return getRawButtonPressed(Button.kR3.value);
  }

  /**
   * Whether the L3 (left stick) button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getL3ButtonReleased() {
    return getRawButtonReleased(Button.kL3.value);
  }

  /**
   * Whether the R3 (right stick) button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getR3ButtonReleased() {
    return getRawButtonReleased(Button.kR3.value);
  }

  /**
   * Read the value of the Square button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getSquareButton() {
    return getRawButton(Button.kSquare.value);
  }

  /**
   * Whether the Square button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getSquareButtonPressed() {
    return getRawButtonPressed(Button.kSquare.value);
  }

  /**
   * Whether the Square button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getSquareButtonReleased() {
    return getRawButtonReleased(Button.kSquare.value);
  }

  /**
   * Read the value of the Cross button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getCrossButton() {
    return getRawButton(Button.kCross.value);
  }

  /**
   * Whether the Cross button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getCrossButtonPressed() {
    return getRawButtonPressed(Button.kCross.value);
  }

  /**
   * Whether the Cross button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getCrossButtonReleased() {
    return getRawButtonReleased(Button.kCross.value);
  }

  /**
   * Read the value of the Triangle button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getTriangleButton() {
    return getRawButton(Button.kTriangle.value);
  }

  /**
   * Whether the Triangle button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getTriangleButtonPressed() {
    return getRawButtonPressed(Button.kTriangle.value);
  }

  /**
   * Whether the Triangle button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getTriangleButtonReleased() {
    return getRawButtonReleased(Button.kTriangle.value);
  }

  /**
   * Read the value of the Circle button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getCircleButton() {
    return getRawButton(Button.kCircle.value);
  }

  /**
   * Whether the Circle button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getCircleButtonPressed() {
    return getRawButtonPressed(Button.kCircle.value);
  }

  /**
   * Whether the Circle button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getCircleButtonReleased() {
    return getRawButtonReleased(Button.kCircle.value);
  }

  /**
   * Read the value of the share button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getShareButton() {
    return getRawButton(Button.kShare.value);
  }

  /**
   * Whether the share button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getShareButtonPressed() {
    return getRawButtonPressed(Button.kShare.value);
  }

  /**
   * Whether the share button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getShareButtonReleased() {
    return getRawButtonReleased(Button.kShare.value);
  }

  /**
   * Read the value of the PS button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getPSButton() {
    return getRawButton(Button.kPS.value);
  }

  /**
   * Whether the PS button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getPSButtonPressed() {
    return getRawButtonPressed(Button.kPS.value);
  }

  /**
   * Whether the PS button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getPSButtonReleased() {
    return getRawButtonReleased(Button.kPS.value);
  }

  /**
   * Read the value of the options button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getOptionsButton() {
    return getRawButton(Button.kOptions.value);
  }

  /**
   * Whether the options button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getOptionsButtonPressed() {
    return getRawButtonPressed(Button.kOptions.value);
  }

  /**
   * Whether the options button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getOptionsButtonReleased() {
    return getRawButtonReleased(Button.kOptions.value);
  }

  /**
   * Read the value of the touchpad on the controller.
   *
   * @return The state of the touchpad.
   */
  public boolean getTouchpad() {
    return getRawButton(Button.kTouchpad.value);
  }

  /**
   * Whether the touchpad was pressed since the last check.
   *
   * @return Whether the touchpad was pressed since the last check.
   */
  public boolean getTouchpadPressed() {
    return getRawButtonPressed(Button.kTouchpad.value);
  }

  /**
   * Whether the touchpad was released since the last check.
   *
   * @return Whether the touchpad was released since the last check.
   */
  public boolean getTouchpadReleased() {
    return getRawButtonReleased(Button.kTouchpad.value);
  }
}
