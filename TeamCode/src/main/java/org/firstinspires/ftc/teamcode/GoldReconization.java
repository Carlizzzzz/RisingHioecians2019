package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Created by eitc on 25/1/2019.
 */

public class GoldReconization {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AVqo1bb/////AAABmVKiVARMvEt7t9+nrvcP4GQ364E8Re8prujYiXukla9CkvAhYd82qZigVI2LyXcgqS+/P21uz64+qkDt3/PyBNxTaziLm/fSKjSH/YIBwPUMqRXLsymB3ku/HPtFikIM8KQgTD3TzcEx7qQfikd5T9aHLwF2w/YBy9g7Cb/egNRQH1xMhO4tPnMu74U2iuMpxNIkf1p/Ek3W7nmuTSXtz66yA84Sknp+RlusehMB9yirtLggU3QxbwzQTmsG9OKAICyELhROsqen50XW4r1GrVl7YKLgyI5DOCUEpytl7kO9a/MWNDK5uZ+uYplfZjtmhm6xvQ7++OGSU331wZRAQvCxWfLEL4d0p+T/7LmxO0pj";
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    private String position = "Un-inited";

    public GoldReconization(HardwareMap hardwareMap) {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void endDetect(){
        tfod.deactivate();
        tfod.shutdown();
    }

    public String startReconizing(Telemetry telemetry) {

        /** Activate Tensor Flow Object Detection. */

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop();
                        } else {
                            silverMineral2X = (int) recognition.getTop();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            position = "Left";
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            position= "Right";
                        } else {
                            position = "Center";
                        }
                    }else
                        position="not found";
                }
            }
        }
        return position;
    }
}
