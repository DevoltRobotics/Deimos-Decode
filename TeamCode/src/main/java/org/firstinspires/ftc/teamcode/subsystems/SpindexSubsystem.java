package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public static double IntakePos = 0.02;
    public static double ShootPos = 0.5;

    public static double IntakePos2 = 0.245;

    public static double ShootPos2 = 0.73;

    public static double IntakePos3 = 0.475;

    public static double ShootPos3 = 0.965;


    public static double SpindexDelayFactorSeconds = 0.6;

    DigitalChannel laser;
    DigitalChannel home;

    DcMotor encoder;
    CRServo spindex1;
    CRServo spindex2;

    public boolean BPresence = false;

    public NormalizedColorSensor colorSensor;
    public double SpindexPos;


    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(
            0.5,   // P
            0.00,   // I
            0.015,  // D
            0.00    // F
    );

    public PIDFController SPID = new PIDFController(SPcoeffs);


    public static double TRIGGER_COOLDOWN_MS = 250;             // anti rebote tiempo
    public static double MIN_ADVANCE_RAD = Math.toRadians(110); // avance mínimo para indexar

    private int encoderOffset = 0;
    private double targetPos = IntakePos;

    private boolean Shootmode = false;     // TRUE = modo manual/tiro; FALSE = modo indexar auto
    public static int nBalls = 0; // CONTAODR INTERNO

    public boolean FirstInitIn = true;

    public boolean FirstInitSho = true;

    public double power = 0;

    //funciones para CMDS


    public SpindexSubsystem(HardwareMap hMap) {
        laser = hMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);

        home = hMap.get(DigitalChannel.class, "home");
        home.setMode(DigitalChannel.Mode.INPUT);

        spindex1 = hMap.get(CRServo.class, "spindex1");
        spindex2 = hMap.get(CRServo.class, "spindex2");

        encoder = hMap.get(DcMotor.class, "intake");

        colorSensor = hMap.get(NormalizedColorSensor.class, "colorL");
        colorSensor.setGain(8);
    }

    @Override
    public void periodic() {
        if (nBalls == 3) {
            Shootmode = true;
        } else if (nBalls == 0) {
            Shootmode = false;
        }

        nBalls = Range.clip(nBalls, 0, 3);

        SpindexPos = encoder.getCurrentPosition() - encoderOffset;

        SPID.setCoefficients(SPcoeffs);
        SPID.setSetPoint(targetPos);

        power = SPID.calculate(SpindexPos);

        spindex1.setPower(power);
        spindex2.setPower(power);


        FtcDashboard.getInstance().getTelemetry().addData("magnetico", home.getState());
        FtcDashboard.getInstance().getTelemetry().addData("shootMode", Shootmode);
        FtcDashboard.getInstance().getTelemetry().addData("spindex pos", SpindexPos);
        FtcDashboard.getInstance().getTelemetry().addData("target pos", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("angleerr", SPID.getPositionError());
        FtcDashboard.getInstance().getTelemetry().addData("presencia", BPresence);
        FtcDashboard.getInstance().getTelemetry().addData("nBalls", nBalls);
    }

    public void setnBalls(int nBalls){
        SpindexSubsystem.nBalls = nBalls;
    }

    public boolean getFirstInitIn() {
        return FirstInitIn;
    }

    public void setFirstInitIn(boolean FirstInitIn) {
        this.FirstInitIn = FirstInitIn;
    }

    public boolean getFirstInitSho() {
        return FirstInitSho;
    }

    public void setFirstInitsho(boolean FirstInitSho) {
        this.FirstInitSho = FirstInitSho;
    }

    public void setShootmode(boolean Shootmode) {
        this.Shootmode = Shootmode;
    }

    public void holdShootmode() {
        Shootmode = true;
    }

    public boolean getShootmode() {
        return Shootmode;
    }

    public double advanceOneIndex() {
        double lastTargetPos = targetPos;

        if (!Shootmode) {
            if (targetPos == IntakePos) {
                targetPos = IntakePos2;
            } else if (targetPos == IntakePos2) {
                targetPos = IntakePos3;
            } else if (targetPos == IntakePos3) {
                targetPos = IntakePos;
            }
        } else {
            if (targetPos == ShootPos) {
                targetPos = ShootPos2;
            } else if (targetPos == ShootPos2) {
                targetPos = ShootPos3;
            } else if (targetPos == ShootPos3) {
                targetPos = ShootPos;
            }

        }

        return targetPos - lastTargetPos;
    }

    public void returnOneIndex() {
        if (!Shootmode) {
            if (targetPos == IntakePos) {
                targetPos = IntakePos3;
            } else if (targetPos == IntakePos2) {
                targetPos = IntakePos;
            } else if (targetPos == IntakePos3) {
                targetPos = IntakePos2;
            }
        } else {
            if (targetPos == ShootPos) {
                targetPos = ShootPos3;
            } else if (targetPos == ShootPos2) {
                targetPos = ShootPos;
            } else if (targetPos == ShootPos3) {
                targetPos = ShootPos2;
            }

        }
    }

    public int getnBalls() {
        return nBalls;
    }

    public void addnBalls() {
        nBalls++;
    }

    public void lessBalls() {
        nBalls--;
    }

    public void resetnBalls() {
        nBalls = 0;
    }

    public boolean getBPresence() {
        return laser.getState();
    }
    public boolean isAtHome() {
        return !home.getState();
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }
    public double getTargetPos() {
        return targetPos;
    }

    public void resetRelativePos() {
        encoderOffset = encoder.getCurrentPosition();
    }

}
