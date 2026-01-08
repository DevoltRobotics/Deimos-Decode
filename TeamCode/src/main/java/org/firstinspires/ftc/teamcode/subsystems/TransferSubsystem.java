package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.transfer.StopTransferCMD;

public class TransferSubsystem extends SubsystemBase {

    CRServo transfer1;
    CRServo transfer2;


    public TransferSubsystem(HardwareMap hMap){
        transfer1 = hMap.get(CRServo.class,"tarnsfer");
        transfer2 = hMap.get(CRServo.class,"tarnsfer2");
        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);
        setDefaultCommand(new StopTransferCMD(this));

    }

    public void In(){
        transfer1.setPower(-1);
        transfer2.setPower(-1);

    }

    public void Out(){
        transfer1.setPower(1);
        transfer2.setPower(1);
    }

    public void Stop(){
        transfer1.setPower(0);
        transfer2.setPower(0);
    }



}
