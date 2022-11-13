package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.CameraOpenCV.Position;

@Autonomous(name = "Auto")
public class Auto extends AutoAbstract
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        onInit();

        forward(1.5, 2.0);

        switch (camera.scanner.position)
        {
            case RED:
            {
                left(24.0, 5.0);
            } break;

            case GREEN:
            {
                // no movement for now
            } break;

            case BLUE:
            {
                right(25.0, 5.0);
            } break;
        }

        forward(30.0, 10.0);
    }
}
