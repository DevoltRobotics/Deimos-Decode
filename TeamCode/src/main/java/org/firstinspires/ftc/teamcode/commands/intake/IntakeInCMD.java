package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class IntakeInCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    private final SpindexSubsystem spindexSubsystem;

    public IntakeInCMD(IntakeSubsystem intakeSubsystem,SpindexSubsystem spindexSubsystem){

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        this.spindexSubsystem = spindexSubsystem;

    }

    @Override
    public void initialize(){
        intakeSubsystem.In();
    }

    @Override
    public void execute(){
        if (spindexSubsystem.getBPresence()){
            intakeSubsystem.Hold();
        }else {
            intakeSubsystem.In();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
