package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeHoldCMD extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeHoldCMD(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize() {
        intakeSubsystem.Hold();
    }



    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}
