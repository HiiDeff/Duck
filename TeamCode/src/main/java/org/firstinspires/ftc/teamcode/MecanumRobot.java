package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MecanumRobot {

    // Order of wheels: LF, LR, RF, RR.
    private static final int[] DIRECTIONS = {-1, 1, -1, -1}; // -1 for REVERSE, 1 for FORWARDS.
    private final DcMotor[] wheels;
    private final Servo armLift;
    private final ColorSensor colorSensor;

    public MecanumRobot(HardwareMap map) {
        wheels = new DcMotor[4];
        wheels[0] = map.get(DcMotor.class, "leftFront");
        wheels[1] = map.get(DcMotor.class, "leftRear");
        wheels[2] = map.get(DcMotor.class, "rightFront");
        wheels[3] = map.get(DcMotor.class, "rightRear");
        armLift = map.get(Servo.class, "armLift");
        colorSensor = map.get(ColorSensor.class, "colorSensor");
    }

    public float[] getRGB() {
        return new float[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }

    public void setArmLiftPos(double pos) {
        armLift.setPosition(pos);
    }

    public double getArmLiftPos() {
        return armLift.getPosition();
    }

    // Setting the powers of wheels:
    public void setPowers(double... powers) {
        if (powers.length != 4) return;
        setPowersInternal(scaleDownToOnes(powers));
    }

    public double[] getPowers(double power, double angle) {
        power = capToOne(power);
        angle += Math.PI * 0.25;
        double xp = Math.cos(angle);
        double yp = Math.sin(angle);
        double factor = 0.98 / Math.max(Math.abs(xp), Math.abs(yp));
        xp *= factor * power;
        yp *= factor * power;
        // LF, LR, RF, RR:
        return new double[]{xp, yp, yp, xp};
    }

    // Private util:
    private void setPowersInternal(double...powers) {
        for (int i = 0; i < 4; ++i) {
            wheels[i].setPower(powers[i] * DIRECTIONS[i]);
        }
    }

    private double[] scaleDownToOnes(double[] powers) {
        double max = 0.0;
        for (double power : powers) {
            max = Math.max(max, power);
        }
        if (max < 1.0) {
            return powers;
        }
        for (int i = 0; i < powers.length; i++) {
            powers[i] /= max;
        }
        return powers;
    }

    private double capToOne(double power) {
        double abs = Math.abs(power);
        if (Math.abs(power) > 1.0) return power / abs;
        return power;
    }
}
