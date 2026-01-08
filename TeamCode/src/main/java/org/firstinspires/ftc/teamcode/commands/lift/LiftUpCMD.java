package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftUpCMD extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public LiftUpCMD(LiftSubsystem liftSubsystem){

        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);

    }

    @Override
    public void initialize(){
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
