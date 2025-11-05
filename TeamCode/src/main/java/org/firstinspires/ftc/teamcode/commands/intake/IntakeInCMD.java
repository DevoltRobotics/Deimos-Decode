package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeInCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeInCMD(IntakeSubsystem intakeSubsystem){

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        intakeSubsystem.In();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
