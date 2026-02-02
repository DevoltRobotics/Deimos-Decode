package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {


    CRServo lift;
    public LiftSubsystem(HardwareMap hMap){

        lift = hMap.get(CRServo.class,"lift");
    }

    public void liftUp(){
        lift.setPower(1);
    }

    public void liftDown(){
        lift.setPower(-1);
    }

    public void liftStop(){
        lift.setPower(0);
    }




}
