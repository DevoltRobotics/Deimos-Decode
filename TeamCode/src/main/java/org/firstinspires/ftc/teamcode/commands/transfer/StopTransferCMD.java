package org.firstinspires.ftc.teamcode.commands.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class StopTransferCMD extends CommandBase {


    private TransferSubsystem transferSubsystem;

    public StopTransferCMD(TransferSubsystem transferSubsystem){

        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);

    }
    @Override
    public void initialize(){
        transferSubsystem.Stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
