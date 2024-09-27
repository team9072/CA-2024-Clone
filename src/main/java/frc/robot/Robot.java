// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Controller
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Intake m_intake = Intake.getInstance();
  private final Compressor m_compressor = Compressor.getInstance();
  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();

  // Auto stuff
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();
  private AutoChooser m_autoChooser = new AutoChooser();

  // Simulation stuff
  private final Field m_field = Field.getInstance();

  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static boolean isBlueAlliance() {
    return getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return getAlliance() == Alliance.Red;
  }

  public static double invertIfRed(double n) {
    return n * (isRedAlliance() ? -1 : 1);
  }

  /**
   * This function is run when the robot is first started up.
   */
  @Override
  public void robotInit() {
    setupLogging();

    // Add all subsystems to the list
    m_allSubsystems.add(m_intake);
    m_allSubsystems.add(m_compressor);
    m_allSubsystems.add(m_drive);
    m_allSubsystems.add(m_shooter);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    updateSim();

    // Used by sysid
    // CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autoRunner.setAutoMode(m_autoChooser.getSelectedAuto());
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    m_intake.resetPivotMotion();
  }

  private double m_shooterSpeed = 0;

  @Override
  public void teleopPeriodic() {
    m_drive.drive(invertIfRed(m_driverController.getForwardAxis()), invertIfRed(m_driverController.getLeftAxis()), m_driverController.getTurnAxis(), true, true);

    // X to reset gyro
    if (m_driverController.getWantsGyroReset()) {
      m_drive.resetGyro();
    }

    // Shooter: Hold RightBumper to spin, release to stop.
    // Then, you can easily tap B to score.
    if (m_operatorController.getWantsSpinShooter()) {
      // Hold Back to slow down the shooter
      if (m_operatorController.getWantsLessSpeed()) {
        m_shooterSpeed = Constants.kShooterSlowSpeed;
      } else {
        m_shooterSpeed = Constants.kShooterSpeed;
      }
    } else {
      m_shooterSpeed = 0;
    }
    
    m_shooter.setSpeed(m_shooterSpeed);

    // Scoring process
    // A to go down and intake, beam break detects and goes up, right trigger to spin shooter, B to launch

    // Intake commands
    if (m_operatorController.getWantsIntakeDown()) {
      m_intake.goToGround();
    } else if (m_operatorController.getWantsIntakeEject()) {
      m_intake.eject();
    } else if (m_operatorController.getWantsIntakeStow()) {
      m_intake.goToStow();
    } else if (m_intake.getIntakeState() != IntakeState.INTAKE) {
      m_intake.stopIntake();
    }
  }

  @Override
  public void disabledInit() {
    m_shooterSpeed = 0;
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_field.setRobotPose(m_drive.getPose());
  }

  private void setupLogging() {
    Logger.recordMetadata("ProjectName", "Flipside"); // Set a metadata value

    if (isReal()) {
      // Disable advantagekit logger as we dont have a usb in the rio
      //Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(RobotConstants.kPdhId, ModuleType.kRev); // Enables power distribution logging
    }

    Logger.start();
  }
}
