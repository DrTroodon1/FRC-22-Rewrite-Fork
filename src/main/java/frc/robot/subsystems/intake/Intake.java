package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;

    private boolean isExtended;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    /** Run the roller at the specified percentage. */
    public void runIntakeVelocity(double velocity) {
        intakeIO.runIntakeVelocity(velocity);
    }

    public void runIntakePercent(double percent) {
        intakeIO.runIntakePercent(percent);
    }

    public void runIntakeVoltage(double voltage) {
        intakeIO.runIntakeVoltage(voltage);
    }

    public void getIntakeVelocity() {
        intakeIO.getIntakeVelocity();
    }

    public void stop() {
        intakeIO.stopIntake();
    }

    public void extend() {
        if (isExtended) {
            return;
        }

        intakeIO.setIntakeSolenoid(true);
        isExtended = false;
    }

    public void retract() {
        if (!isExtended) {
            return;
        }

        intakeIO.setIntakeSolenoid(false);
        isExtended = true;
    }

    public boolean getState() {
        return intakeIO.getState();
    }
}
