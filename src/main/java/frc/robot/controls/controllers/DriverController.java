package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public class DriverController extends FilteredController {
  
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public double getForwardAxis() {
    return -this.getFilteredAxis(Axis.kLeftY);
  }

  public double getLeftAxis() {
    return -this.getFilteredAxis(Axis.kLeftX);
  }

  public double getTurnAxis() {
    return this.getFilteredAxis(Axis.kRightX);
  }

  public boolean getWantsGyroReset() {
    return this.getButton(Button.kX);
  }
}
