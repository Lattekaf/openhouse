package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class webcamExample extends OpMode {
    webcam aprilTagWebcam = new webcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap,telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        telemetry.addData("id20 String", id20.toString());
    }
}
