package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumBot
{
    private static final int MOTOR_COUNT = 4;

    public static final int FL = 0, FR = 1, BL = 2, BR = 3;
    public DcMotor[] motors = new DcMotor[MOTOR_COUNT];

    public void setMode(DcMotor.RunMode mode)
    {
        for (int i = 0; i < MOTOR_COUNT; ++i)
        {
            motors[i].setMode(mode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        for (int i = 0; i < MOTOR_COUNT; ++i)
        {
            motors[i].setZeroPowerBehavior(behavior);
        }
    }

    public void setPower(double power)
    {
        for (int i = 0; i < MOTOR_COUNT; ++i)
        {
            motors[i].setPower(power);
        }
    }

    public void setPowerLeft(double power)
    {
        motors[FL].setPower(power);
        motors[BL].setPower(power);
    }

    public void setPowerRight(double power)
    {
        motors[FR].setPower(power);
        motors[BR].setPower(power);
    }

    private double bias(double val)
    {
        final double factor = 3.0;
        final double calculated = ((Math.exp(Math.abs(val) * factor) - 1.0) / (Math.exp(factor) - 1.0));
        //return Math.copysign(val, calculated);
        return (val > 0.0) ? calculated : -calculated;
    }

    public void onUpdate(Gamepad gp1, Gamepad gp2)
    {
        //unfortnately, everything is bugged
        //left and right stick X are switched

        final double magnitude = Math.hypot(-gp1.right_stick_x, gp1.left_stick_y);
        final double robotAngle = Math.atan2(gp1.left_stick_y, -gp1.right_stick_x) - Math.PI / 4;
        final double rightX = 0.7 * bias(-gp1.left_stick_x);
        final double ROOT2 = Math.sqrt(2.0);

        motors[FL].setPower((magnitude * Math.cos(robotAngle) + rightX) * ROOT2 * 0.6);
        motors[FR].setPower((magnitude * Math.sin(robotAngle) - rightX) * ROOT2 * 0.6);
        motors[BL].setPower((magnitude * Math.cos(robotAngle) - rightX) * ROOT2 * 0.6);
        motors[BR].setPower((magnitude * Math.sin(robotAngle) + rightX) * ROOT2 * 0.6);
    }

    public MecanumBot(HardwareMap map)
    {
        motors[FL] = map.get(DcMotor.class, "frontLeftDrive");
        motors[FR] = map.get(DcMotor.class, "frontRightDrive");
        motors[BL] = map.get(DcMotor.class, "backLeftDrive");
        motors[BR] = map.get(DcMotor.class, "backRightDrive");

        motors[FR].setDirection(DcMotor.Direction.REVERSE);
        motors[BR].setDirection(DcMotor.Direction.REVERSE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPower(0.0);
    }
}
