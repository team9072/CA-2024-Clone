package frc.robot.autonomous.modes;

import frc.robot.Constants;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class ShootPreloadedMode extends AutoModeBase {

    @Override
    public void queueTasks() {
        queueShooterTask();
    }

    protected void queueShooterTask() {
        queueTask(new ParallelTask(
            new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
            new ShooterTask(Constants.kShooterSpeed),
            new WaitTask(1.0)));

        queueTask(new ParallelTask(
            new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
            new WaitTask(1.0)));

        queueTask(new IntakeTask(IntakeState.NONE, PivotTarget.STOW));
        queueTask(new ShooterTask(0));
    }
}
