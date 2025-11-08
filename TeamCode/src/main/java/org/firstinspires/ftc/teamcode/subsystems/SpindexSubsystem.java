package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class SpindexSubsystem extends SubsystemBase {

    public com.qualcomm.robotcore.hardware.CRServo spindex;

    public NormalizedColorSensor colorSensorL;
    public NormalizedColorSensor colorSensorR;
    private DetectedColor currentColorL = DetectedColor.UNKNOWN;


    double offset = 217.85;

    double targetPos = offset;

    public double FPos = offset;

    public double SPos = (FPos + 120);

   public double TPos = (SPos + 120);

    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }


    public AnalogInput SpindexPos;

    public PIDFController SpindexPID;
    public static PIDFCoefficients SPcoeffs = new PIDFCoefficients(0.006,0,0.00017,0);

    float normRed,normGreen,normBlue;
    NormalizedRGBA colorsL;

    public double Spindexp;

    public static double ENTER_DISTANCE_CM = 2.5;   // umbral para DISPARAR
    public static double EXIT_DISTANCE_CM  = 3.5;   // umbral mayor para SOLTAR (histeresis)
    public static double TRIGGER_COOLDOWN_MS = 250; // tiempo mínimo entre disparos
    public static double MIN_ADVANCE_DEG = 110;      // avance mínimo del spindex para poder rearmar

    // ====== NUEVO: estado para edge-detect/debouncing ======
    private boolean objectLatched = false;
    private double lastTriggerTimeMs = -9999;
    private double lastTriggerPosDeg = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private static double circularDeltaDeg(double a, double b) {
        double d = Math.abs(a - b) % 360.0;
        return d > 180 ? 360.0 - d : d;
    }

    public SpindexSubsystem (HardwareMap hMap){

        spindex = hMap.get(CRServo.class,"spindex");

        SpindexPos = hMap.get(AnalogInput.class,"Spencoder");

        colorSensorL = hMap.get(NormalizedColorSensor.class,"colorL");
        colorSensorR = hMap.get(NormalizedColorSensor.class,"colorR");


        colorSensorL.setGain(8);

        SpindexPID = new PIDFController(SPcoeffs);

    }

    @Override
    public void periodic(){
        if (targetPos > 360){
            targetPos -= 360;
        }
        SpindexPID.setCoefficients(SPcoeffs);
        Spindexp = SpindexPos.getVoltage() / 3.3 * 360;
        double distCm = ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM);
        double nowMs = timer.milliseconds();

       /* if (((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM) <= 2.5){
            targetPos += 120;
        }*/
        if (!objectLatched
                && distCm <= ENTER_DISTANCE_CM
                && (nowMs - lastTriggerTimeMs) >= TRIGGER_COOLDOWN_MS) {

            targetPos += 120;              // solo UNA vez por objeto
            objectLatched = true;          // quedamos “enganchados” hasta que se libere
            lastTriggerTimeMs = nowMs;
            lastTriggerPosDeg = Spindexp;  // recordamos la posición al disparar
        }

        // Liberar el latch solo cuando: a) se alejó (EXIT) y b) el spindex avanzó suficiente
        if (objectLatched) {
            boolean movedEnough = circularDeltaDeg(Spindexp, lastTriggerPosDeg) >= MIN_ADVANCE_DEG;
            if (distCm >= EXIT_DISTANCE_CM && movedEnough) {
                objectLatched = false;
            }
        }
        SpindexPID.setSetPoint(targetPos);
        double power = SpindexPID.calculate(Spindexp);

        if (SpindexPID.getPositionError() > 180){
        spindex.setPower(power);
        } else if (SpindexPID.getPositionError() < 180) {
            spindex.setPower(-power);
        }else {
            spindex.setPower(power);
        }

        colorsL = colorSensorL.getNormalizedColors();
         normRed = colorsL.red/ colorsL.alpha;
        normGreen = colorsL.green/ colorsL.alpha;
        normBlue = colorsL.blue/ colorsL.alpha;

        FtcDashboard.getInstance().getTelemetry().addData("Red",normRed);
        FtcDashboard.getInstance().getTelemetry().addData("Green",normGreen);
        FtcDashboard.getInstance().getTelemetry().addData("Blue",normBlue);
        FtcDashboard.getInstance().getTelemetry().addData("target",targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("Spindex_pos",Spindexp);
        FtcDashboard.getInstance().getTelemetry().addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM));



        if (normRed > 0.35 && normBlue > 0.35 && normGreen < 0.35){
            currentColorL = DetectedColor.PURPLE;
        } else if (normGreen > 0.35 && normBlue < 0.35 && normRed < 0.35) {
            currentColorL = DetectedColor.GREEN;
        }else {
            currentColorL = DetectedColor.UNKNOWN;
        }


    }

    public void setTargetPos(double targetPos){
        this.targetPos = targetPos;
    }




}
