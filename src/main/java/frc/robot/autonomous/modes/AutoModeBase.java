package frc.robot.autonomous.modes;

import java.util.ArrayList;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract void queueTasks();

  public void setStartingPose() {
    // Figure out the first PathPlanner path
    Pose2d startingPose = null;

    // Loop over the m_tasks and find the first DriveTrajectoryTask
    for (Task task : m_tasks) {
      if (task instanceof DriveTrajectoryTask) {
        // Set the starting pose to the starting pose of the first DriveTrajectoryTask
        startingPose = ((DriveTrajectoryTask) task).getStartingPose();

        if (Robot.isRedAlliance()) {
          startingPose = GeometryUtil.flipFieldPose(startingPose);
        }
        break;
      }
    }

    // If there isn't one, default to something visible
    if (startingPose == null) {
      startingPose = Robot.isBlueAlliance() ? Constants.Field.blueCenterPose2d : Constants.Field.redCenterPose2d;
    }

    Drivetrain m_drive = Drivetrain.getInstance();
    m_drive.resetOdometry(startingPose);
  };

  /* Premade task sequences */

  protected Task shooterSequence() {
    return new SequentialTask(
        new ParallelTask(
            new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
            new ShooterTask(Constants.kShooterSpeed),
            new WaitTask(1.0)),

        new ParallelTask(
            new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
            new WaitTask(1.0)),

        new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
        new ShooterTask(0));
  }

  protected Task intakeDown() {
    return new IntakeTask(IntakeState.INTAKE, PivotTarget.GROUND);
  }

  protected Task intakeUp() {
    return new IntakeTask(IntakeState.NONE, PivotTarget.STOW);
  }
}
