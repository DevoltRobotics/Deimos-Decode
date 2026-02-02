package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftDownCMD extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public LiftDownCMD(LiftSubsystem liftSubsystem){

        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);

    }

    @Override
    public void initialize(){
        liftSubsystem.liftDown();
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
