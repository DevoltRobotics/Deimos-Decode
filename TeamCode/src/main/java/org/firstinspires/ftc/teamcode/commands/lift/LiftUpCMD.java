package org.firstinspires.ftc.teamcode.commands.lift;

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
        liftSubsystem.liftUp();
    }

    @Override
    public void end(boolean interrupted) {
    liftSubsystem.liftStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
