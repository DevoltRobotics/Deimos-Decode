package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;

public class IntakeSubsystem extends SubsystemBase {

    DcMotor intake;


    public IntakeSubsystem(HardwareMap hMap){
        intake = hMap.get(DcMotor.class,"intake");

        setDefaultCommand(new IntakeHoldCMD(this));
    }

    public void In(){
        intake.setPower(-1);
    }

    public void Out(){
        intake.setPower(1);
    }

    public void Stop(){
        intake.setPower(0);
    }

    public void Hold(){
        intake.setPower(-0.2);
    }
}
