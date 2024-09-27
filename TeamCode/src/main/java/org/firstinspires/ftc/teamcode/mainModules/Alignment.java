package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Alignment {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DistanceSensor distanceSensor;

    private double distance1 = 404.0;
    private double distance2 = 1167.0; // 3 calculated distances in mm from sensor to wall to align centre of drivebase to goal
    private double distance3 = 1945.0;
    private double target = distance1; // TODO make it a list selectable via the secondary controller maybe
    private double tolerance = 10.0;

    private double Kp = 0.0019;

    private final boolean protect;

    private Gamepad gampepad1;
    private Gamepad gamepad2;

    public Alignment(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.protect = protect;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gampepad1 = gamepad1;
        this.gamepad2  = gamepad2;
        init();
    }

    private void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance"); // TODO add a red/blue toggle
    }
    public void addToTelemetry(){
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        telemetry.addData("Current distance", distance);
        if (distance < 80 && distance > 40){
            gampepad1. rumble(25);
            gamepad2.rumble(25);
        }
    }

    @Deprecated
    public double alignTarget(double target) {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.MM);
        double error = target - currentDistance;
        telemetry.addData("Error", error);

        if (Math.abs(error) > tolerance) {
            double speed = -(Kp * error); // Calculate proportional speed

            // Ensure the speed is within an acceptable range (-1.0 to 1.0)
            speed = Math.max(-1.0, Math.min(1.0, speed));
            return speed;
        } else {
            return 0.0;
        }
    }
}
