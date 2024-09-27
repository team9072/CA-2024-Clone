package frc.robot.autonomous.tasks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectoryTask extends Task {
  private Drivetrain m_drive;
  private boolean m_isFinished = false;
  private PathPlannerPath m_autoPath = null;

  private Command m_pathCommand;

  public DriveTrajectoryTask(String pathName) {
    m_drive = Drivetrain.getInstance();

    try {
      System.out.println("Loading path: " + pathName);
      m_autoPath = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception ex) {
      DriverStation.reportError("Unable to load PathPlanner trajectory: " + pathName, ex.getStackTrace());
      m_isFinished = true;
    }

    m_pathCommand = AutoBuilder.followPath(m_autoPath);
  }

  @Override
  public void start() {
    DriverStation.reportWarning("Running path for " + Robot.getAlliance().toString(), false);
    //TODO: m_pathCommand.initialize();
  }

  @Override
  public void update() {
    /* TODO: m_pathCommand.execute();
    if (m_pathCommand.isFinished()) {
      m_pathCommand.end(false);
      m_isFinished = true;
    }*/
  }

  @Override
  public void updateSim() {
    
  }

  public Pose2d getStartingPose() {
    return m_autoPath.getPreviewStartingHolonomicPose();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Auto trajectory done", false);
    m_drive.stop();
  }
}
