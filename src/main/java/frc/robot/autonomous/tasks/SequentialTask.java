package frc.robot.autonomous.tasks;

public class SequentialTask extends Task {
    private final Task[] k_tasks;
    private Task m_currentTask = null;
    private int m_index;

    public SequentialTask(Task... tasks) {
        this.k_tasks = tasks;
    }

    @Override
    public void start() {
        m_index = 0;
        m_currentTask = null;
    }

    @Override
    public void update() {
        if (m_currentTask == null) {
            if (m_index > k_tasks.length) return;

            m_currentTask = k_tasks[m_index];
            m_currentTask.start();
        }

        m_currentTask.update();
        if (m_currentTask.isFinished()) {
            m_currentTask.done();
            
            m_currentTask = null;
            m_index++;
        }
    }

    @Override
    public void updateSim() {
        if (m_currentTask == null) return;

        m_currentTask.updateSim();
    }

    @Override
    public boolean isFinished() {
        return m_index >= k_tasks.length;
    }
    
}
