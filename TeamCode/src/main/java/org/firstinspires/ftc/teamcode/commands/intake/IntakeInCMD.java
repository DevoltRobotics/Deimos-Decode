package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class IntakeInCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    private final SpindexSubsystem spindex;

    public IntakeInCMD(IntakeSubsystem intakeSubsystem,SpindexSubsystem spindexSubsystem){

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        spindex = spindexSubsystem;

    }


    @Override
    public void execute() {
            intakeSubsystem.In();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
