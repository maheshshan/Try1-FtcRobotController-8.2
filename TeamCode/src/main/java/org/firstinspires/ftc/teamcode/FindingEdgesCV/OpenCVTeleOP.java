package org.firstinspires.ftc.teamcode.FindingEdgesCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (group = "OpenCv")
public class OpenCVTeleOP extends LinearOpMode {
     private ImageChannels channel;
    @Override
    public void runOpMode() throws InterruptedException {
        //        initialize camera and pipeline
        CVMaster2SetPipeline cv = new CVMaster2SetPipeline(this);
        //      call the function to startStreaming
        cv.observeObjects();

        if (isStopRequested()) return;
        telemetry.addData("Image Channel", "Input_Image");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
             if(gamepad1.x)
             {
                 channel = cv.getImageChannel();

                 switch(channel) {
                     case Input_Image:
                         channel = ImageChannels.Color_RGB_TO_HSV;
                     break;
                     case Color_RGB_TO_HSV:
                         channel = ImageChannels.Initial_Color_Mask;
                         break;
                     case Initial_Color_Mask:
                         channel = ImageChannels.Merged_Mask_To_Input;
                         break;
                     case Merged_Mask_To_Input:
                         channel = ImageChannels.Scaled_Mask_To_FullColor;
                         break;
                     case Scaled_Mask_To_FullColor:
                         channel = ImageChannels.Fully_Masked_Image;
                         break;
                     case Fully_Masked_Image:
                         channel = ImageChannels.Show_Contour_Edges;
                         break;
                     case Show_Contour_Edges:
                         channel = ImageChannels.Show_Simplified_Edge;
                         break;
                     case Show_Simplified_Edge:
                         channel = ImageChannels.Input_Image;
                         break;
                 }
                 telemetry.addData("Image Channel:", channel);
                 telemetry.update();
                 sleep(250);
                 cv.putImageChannel(channel);
            }
        }
        //        stopStreaming
        cv.stopCamera();
    }
}