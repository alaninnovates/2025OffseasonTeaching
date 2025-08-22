package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import badgerlog.Dashboard;
import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Arm extends SubsystemBase {
    private final TalonFX motor;
    private boolean isEnabled = false;

    private double desiredPositionCache = 0.0;

    @Entry(EntryType.Subscriber)
    private static double desiredPosition = 0.25;

    @Entry(EntryType.Publisher)
    private static double currentPos = 0;

    // for pid:
    @Entry(EntryType.Subscriber)
    private static double kP = 1.5;

    @Entry(EntryType.Subscriber)
    private static double kI = 0;

    @Entry(EntryType.Subscriber)
    private static double kD = 0.1;

    @Entry(EntryType.Publisher)
    private static double error = 0;

    private double iAccum = 0.0, previousError = 0.0;

    public Arm() {
        motor = new TalonFX(11);
        RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> {
            isEnabled = true;
        })).onTrue(Commands.runOnce(() -> {
            isEnabled = false;
        }));

        EventLoop buttonLoop = CommandScheduler.getInstance().getActiveButtonLoop();

        Dashboard.getAutoResettingButton("Zero", buttonLoop)
                .onTrue(Commands.runOnce(() -> {
                    System.out.println("Zeroing arm");
                    motor.setPosition(0);
                }));

        Dashboard.getAutoResettingButton("Go To Zero", buttonLoop)
                .onTrue(Commands.runOnce(() -> {
                    desiredPositionCache = desiredPosition;
                    Dashboard.putValue("Arm/desiredPosition", 0.0);
                }));
        Dashboard.getAutoResettingButton("Go To Desired", buttonLoop)
                .onTrue(Commands.runOnce(() -> {
                    Dashboard.putValue("Arm/desiredPosition", desiredPositionCache);
                }));
    }

    public void bangBang() {
        if (currentPos < desiredPosition) {
            motor.setVoltage(0.3);
        } else if (currentPos > desiredPosition) {
            motor.setVoltage(-0.3);
        } else {
            motor.setVoltage(0.0);
        }
    }

    public void pid() {
        error = desiredPosition - currentPos;
        double pTerm = kP * error;
        iAccum += error;
        double iTerm = kI * iAccum;
        double dTerm = kD * (error - previousError) / 0.02;
        previousError = error;
        double output = pTerm + iTerm + dTerm;
        motor.setVoltage(output > 1 ? 1 : (output < -1 ? -1 : output));
    }

    @Override
    public void periodic() {
        if (!isEnabled) {
            return;
        }
        pid();
        currentPos = motor.getPosition().getValueAsDouble();
    }
}
