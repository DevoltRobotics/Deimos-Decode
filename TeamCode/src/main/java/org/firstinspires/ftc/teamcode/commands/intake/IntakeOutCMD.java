package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeOutCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeOutCMD(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.Out();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
