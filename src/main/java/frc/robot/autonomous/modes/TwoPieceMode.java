package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TwoPieceMode extends AutoModeBase {
    private final String k_firstPath;
    private final String k_secondPath;

    public TwoPieceMode(String firstPath, String secondPath) {
        k_firstPath = firstPath;
        k_secondPath = secondPath;
    }

    @Override
    public void queueTasks() {
        queueTask(shooterSequence());

        queueTask(intakeDown());
        queueTask(new DriveTrajectoryTask(k_firstPath));

        queueTask(
            new ParallelTask(
                new DriveTrajectoryTask(k_secondPath),
                new SequentialTask(new WaitTask(2), intakeUp())));

        queueTask(shooterSequence());
    }

}
