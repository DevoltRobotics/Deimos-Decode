package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftHoldCMD extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public LiftHoldCMD(LiftSubsystem liftSubsystem){

        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);

    }

    @Override
    public void initialize(){
        liftSubsystem.hold();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
