#include <opencv2/calib3d/calib3d.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string> 
#include <sstream> 

const int FALSE = 0;
const int TRUE = 1;

using namespace std;
using namespace cv;

enum Processor { cl, gl, cpu };

void CallBackFunc(int event, int x, int y, int flags, void* params)
{

        cv::Mat *image = (Mat*)params;

        if  ( event == EVENT_LBUTTONDOWN )
        {
                //int pixelValue = static_cast<int>(image->at<float>(cv::Point(x,y)));
                cout << "The pixel value at (" << x << ", " << y << ") : " << endl;
        }
        // else if  ( event == EVENT_RBUTTONDOWN )
        // {
        //         cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        // }
        // else if  ( event == EVENT_MBUTTONDOWN )
        // {
        //         cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        // }
        else if ( event == EVENT_MOUSEMOVE )
        {
                std::stringstream text;
                text << "Pos.(" << x << ", " << y << ")";
                //cv::putText(image, text.str(), cv::Point(x,y), FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0,255,255), 1,8);
                cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        }
}

int MenuSelect()
{
        int menu_item;
        std::cout << "#################"<<std::endl;
        std::cout << "1) Camara output"<<std::endl;
        std::cout << "2) Calibration"<<std::endl;
        std::cout << "3) End program"<<std::endl;
        std::cout << "#################"<<std::endl;
        std::cin >> menu_item;
        return menu_item;
}

void KeyOptions()
{
        std::cout << "#############################################"<<std::endl;
        std::cout << "'s' Depth with RGB alignment"<<std::endl;
        std::cout << "'d' Draw RGB & IR chessboard lines"<<std::endl;
        std::cout << "'f' Save RGB & IR frame"<<std::endl;
        std::cout << "'c' Apply to RGB & IR calibration parameters"<<std::endl;
        std::cout << "'other key' Reset to default"<<std::endl;
        std::cout << "'esc' End frame stream"<<std::endl;
        std::cout << "#############################################"<<std::endl;
}

static void calcChessboardCorners(Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
        corners.clear();
        for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float(j*squareSize),float(i*squareSize), 0)); 
}

int main(int argc, char *argv[])
{
        //! [context] Definir variables
        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = nullptr;
        libfreenect2::PacketPipeline *pipeline = nullptr;
         //! [context]

         //! [discovery]Buscar e inicializar el sensor
        if(freenect2.enumerateDevices() == 0)
        {
                std::cout << "no device connected!" << std::endl;
                return -1;
        }
       string serial = freenect2.getDefaultDeviceSerialNumber();
        if(serial == "")  return -1;
        // cout<<"The serial number is :"<<serial<<endl;
        //! [discovery]

        //! [Configuration] Configurar el formato de transmision
        int depthProcessor = Processor::cl;
        if(depthProcessor == Processor::cpu)
        {
                if(!pipeline)
                {
                     //! [pipeline]
                    pipeline = new libfreenect2::CpuPacketPipeline();
                    //! [pipeline]
                }
        } 
        else 
                if (depthProcessor == Processor::gl) 
                {
                        #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
                        if(!pipeline)
                                pipeline = new libfreenect2::OpenGLPacketPipeline();
                        #else
                                std::cout << "OpenGL pipeline is not supported!" << std::endl;
                        #endif
                } 
                else 
                        if (depthProcessor == Processor::cl) 
                        {
                                #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
                                 if(!pipeline)
                                        pipeline = new libfreenect2::OpenCLPacketPipeline();
                                #else
                                        std::cout << "OpenCL pipeline is not supported!" << std::endl;
                                #endif
                        }
        //! [Configuration]

        //! [Start device] Inicio de dispositivo
        if(pipeline)
        {
                //! [open]
                dev = freenect2.openDevice(serial, pipeline);
                //! [open]
        } 
        else 
        {
                dev = freenect2.openDevice(serial);
        }
        if(dev == 0)
        {
                std::cout << "failure opening device!" << std::endl;
                return -1;
        }
        //! [Start device]

        //! [listeners]Configuracion del formato de imagen
        libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |libfreenect2::Frame::Depth |libfreenect2::Frame::Ir);
        libfreenect2::FrameMap frames;
        dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);
        //! [listeners]

        //! [start] Iniciar transferencia de datos
        dev->start();
        // std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
        // std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
        //! [start]

        /// [registration setup]
        libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
        Mat rgbmat, depthmat, irmat;
        Mat map1x, map1y, map2x, map2y;//output map
        Mat RGB_out, IR_out;//Destination image//INTER_LINEAR - bilinear interpolation//BORDER_CONSTANT - iiiiii|abcdefgh|iiiiiii with some specified i
        bool end_flag = 1, remap_flag = 0;
        /// [registration setup]

        //! [image loop] recepcion ciclica
        while(end_flag)
        {
                switch(MenuSelect())
                {
                        case 1:
                        {
                                cv::Size patternsize = cv::Size(9,6);//esquinas interiores del tablero de ajedrez
                                std::vector<cv::Point2f> corners2D;//findChessboardCorners guarda los puntos del tablero aqui
                                bool IRfound, RGBfound;
                                double min, max;
                                Mat imgGrayDrawCal, irmatBuffer, RGBimgGrayDrawCal;
                                int counter_img_save = 0, key_press = 0, key_press_buffer;
                                std::string RGBdireccion = "/home/fernando/Pictures/Calibracion RGB/RGBcal";
                                std::string IRdireccion = "/home/fernando/Pictures/Calibracion IR/IRcal";
                                std::stringstream RGBimgs;
                                std::stringstream IRimgs;

                                KeyOptions();
                                do{
                                        key_press_buffer = cv::waitKey(1) & 0xEFFFFF;
                                        key_press = (key_press_buffer != 0xEFFFFF)?key_press_buffer : key_press;

                                        listener.waitForNewFrame(frames);
                                        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
                                        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
                                        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
                                        //! [loop start]

                                        Mat rgbmat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
                                        Mat depthmat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
                                        Mat irmat = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data);
                                        
                                        ///ir image filter [start]
                                        irmat = (irmat / 4096.0f)/16;
                                        cvtColor(irmat, irmatBuffer, COLOR_GRAY2BGR);
                                        irmatBuffer.convertTo(irmatBuffer, CV_8UC1, 255.0/1);
                                        ///ir image filter [end]

                                        cv::setMouseCallback("depth", CallBackFunc, (void*)&depthmat);///callback for mouse position

                                        cv::resize(rgbmat,rgbmat,Size(512,424),0,0,INTER_AREA); //resize rgbmat from 1920x1080 to 512x424

                                        if(key_press == 100){
                                                cvtColor(irmatBuffer, imgGrayDrawCal, COLOR_BGR2GRAY);
                                                IRfound = findChessboardCorners(imgGrayDrawCal,patternsize,corners2D,
                                                                                CALIB_CB_ADAPTIVE_THRESH +
                                                                                CALIB_CB_NORMALIZE_IMAGE);
                                                if(IRfound){
                                                        drawChessboardCorners(irmatBuffer,patternsize,Mat(corners2D),IRfound);  
                                                }

                                                cvtColor(rgbmat, RGBimgGrayDrawCal, COLOR_BGR2GRAY);
                                                RGBfound = findChessboardCorners(RGBimgGrayDrawCal,patternsize,corners2D,
                                                                                CALIB_CB_ADAPTIVE_THRESH +
                                                                                CALIB_CB_NORMALIZE_IMAGE);
                                                if(RGBfound){
                                                        drawChessboardCorners(rgbmat,patternsize,Mat(corners2D),RGBfound);  
                                                }
                                        }

                                        if(key_press == 115){      //115 igual a 's' en el teclado (no esta ligado a ascii)
                                                for (int x = 0; x < depth->width; x++)
                                                {
                                                        for (int y = 0; y < depth->height; y++)
                                                        {
                                                                if(depthmat.at<float>(y,x) > 1000) //Change rgb mat to black (NULL) between range
                                                                {
                                                                        depthmat.at<float>(y,x) = (float)NULL;
                                                                        rgbmat.at<cv::Vec4b>(cv::Point(x,y))[0] = (uchar)NULL;
                                                                        rgbmat.at<cv::Vec4b>(cv::Point(x,y))[1] = (uchar)NULL;
                                                                        rgbmat.at<cv::Vec4b>(cv::Point(x,y))[2] = (uchar)NULL;   
                                                                }
                                                        }
                                                }
                                        }

                                        if(key_press == 99){
                                                if(remap_flag){
                                                        remap(rgbmat, rgbmat, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
                                                        remap(irmatBuffer, irmatBuffer, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
                                                }
                                        }


                                        if(key_press == 102){      //100 igual a 'd' en el teclado (no esta ligado a ascii)
                                                RGBimgs << RGBdireccion << counter_img_save << ".png";
                                                IRimgs << IRdireccion << counter_img_save << ".png";
                                                cv::imwrite(RGBimgs.str().c_str(), rgbmat);
                                                cv::imwrite(IRimgs.str().c_str(), irmatBuffer);
                                                counter_img_save++;
                                                RGBimgs = std::stringstream();
                                                IRimgs = std::stringstream();
                                                cout << "#################" << endl;
                                                cout << "#  Images saved #" << endl;
                                                cout << "#################" << endl;
                                        }

                                        cv::imshow("rgb", rgbmat);
                                        //cv::imshow("ir", irmat);
                                        cv::imshow("ir", irmatBuffer);//Se divide entre un valor para modificar el valor final del pixel mat
                                        cv::imshow("depth", depthmat / 4096.0f);

                                        //! [loop end]
                                        listener.release(frames);
                                }while(key_press != 27);
                                //! [loop end]
                        break;
                        }
        ////////////////////////////////////////////////// RGB / IR ///////////////////////////////////////////////////////////////////////////
                        case 2:
                        {
                                cv::Size patternsize = cv::Size(9,6);//esquinas interiores del tablero de ajedrez
                                std::vector<cv::Point3f> RGBcorners3D;//obj
                                std::vector<cv::Point2f> RGBcorners2D, IRcorners2D;//corners1, corners2 //findChessboardCorners guarda los puntos del tablero aqui
                                std::vector<std::vector<cv::Point2f>> RGBcoord2D, IRcoord2D;//imagePoints1, imagePoints2 //Ubicacion de las esquinas detectadas en la imagen
                                std::vector<std::vector<cv::Point3f>> RGBcoord3D;//object_points //Ubicacion real de los puntos 3D
                                std::vector<std::vector<cv::Point2f>> left_img_points, right_img_points;

                                std::string RGBdireccion = "/home/fernando/Pictures/Calibracion RGB/RGBcal";
                                std::string IRdireccion = "/home/fernando/Pictures/Calibracion IR/IRcal";
                                std::stringstream RGBimgs;
                                std::stringstream IRimgs;

                                Mat RGBimg, RGBimgGray, IRimg, IRimgGray;
                                bool RGBfound, IRfound;

                                for (int  i = 0; i < 11; i++){

                                        RGBimgs << RGBdireccion << i << ".png";
                                        IRimgs << IRdireccion << i << ".png";
                                        RGBimg = imread(RGBimgs.str().c_str());
                                        IRimg = imread(IRimgs.str().c_str());

                                        cvtColor(RGBimg, RGBimgGray, COLOR_BGR2GRAY);
                                        cvtColor(IRimg, IRimgGray, COLOR_BGR2GRAY);

                                        RGBimgs = std::stringstream();
                                        IRimgs = std::stringstream();

                                        RGBfound = findChessboardCorners(RGBimgGray,patternsize,RGBcorners2D,
                                                                        CALIB_CB_ADAPTIVE_THRESH + 
                                                                        CALIB_CB_NORMALIZE_IMAGE +
                                                                        CV_CALIB_CB_FILTER_QUADS);
                                        IRfound = findChessboardCorners(IRimgGray,patternsize,IRcorners2D,
                                                                        CALIB_CB_ADAPTIVE_THRESH + 
                                                                        CALIB_CB_NORMALIZE_IMAGE +
                                                                        CV_CALIB_CB_FILTER_QUADS);
                                        if(RGBfound){
                                                cornerSubPix(RGBimgGray, RGBcorners2D, Size(11, 11), Size(-1, -1),
                                                        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
                                                drawChessboardCorners(RGBimg,patternsize,Mat(RGBcorners2D),RGBfound);
                                        }
                                        if(IRfound){
                                                cornerSubPix(IRimgGray, IRcorners2D, Size(11, 11), Size(-1, -1),
                                                        TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
                                                drawChessboardCorners(IRimg,patternsize,Mat(IRcorners2D),IRfound);
                                        }

                                        calcChessboardCorners(patternsize,18,RGBcorners3D);//(x,18,x)tamano en milimetros de los cuadrados


                                        if(RGBfound && IRfound){
                                                RGBcoord2D.push_back(RGBcorners2D);
                                                IRcoord2D.push_back(IRcorners2D);
                                                RGBcoord3D.push_back(RGBcorners3D);
                                        }

                                        namedWindow(("RGBcal"+std::to_string(i)).c_str(),WINDOW_AUTOSIZE);
                                        moveWindow(("RGBcal"+std::to_string(i)).c_str(), 40, 30);
                                        imshow(("RGBcal"+std::to_string(i)).c_str(), RGBimg);

                                        namedWindow(("IRcal"+std::to_string(i)).c_str(),WINDOW_AUTOSIZE);
                                        moveWindow(("IRcal"+std::to_string(i)).c_str(), 600, 30);
                                        imshow(("IRcal"+std::to_string(i)).c_str(), IRimg);

                                        waitKey(1000);

                                        cvDestroyWindow(("RGBcal"+std::to_string(i)).c_str());
                                        cvDestroyWindow(("IRcal"+std::to_string(i)).c_str());
                                }

                                for (int i = 0; i < RGBcoord2D.size(); i++) {
                                        vector< Point2f > v1, v2;
                                        for (int j = 0; j < RGBcoord2D[i].size(); j++) {
                                                v1.push_back(Point2f((double)RGBcoord2D[i][j].x, (double)RGBcoord2D[i][j].y));
                                                v2.push_back(Point2f((double)IRcoord2D[i][j].x, (double)IRcoord2D[i][j].y));
                                        }
                                        left_img_points.push_back(v1);
                                        right_img_points.push_back(v2);
                                }

                                cout << "Starting Calibration" << endl;
                                Mat K1, K2, R, F, E, D1, D2;
                                Vec3d T;

                                cout << "Read intrinsics" << endl;

                                stereoCalibrate(RGBcoord3D, left_img_points, right_img_points, K1, D1, K2, D2, IRimg.size(), R, T, E, F);

                                cout << "K1:\n" << K1 << endl;// intrinsic matrix
                                cout << "K2:\n" << K2 << endl;// intrinsic matrix
                                cout << "D1:\n" << D1 << endl;// vector of distortion coefficients
                                cout << "D2:\n" << D2 << endl;// vector of distortion coefficients
                                cout << "R:\n" << R << endl;//Output rotation matrix between the 1st and the 2nd camera coordinate systems.
                                cout << "T:\n" << T << endl;//Output translation vector between the coordinate systems of the cameras.
                                cout << "E:\n" << E << endl;//Output essential matrix.
                                cout << "F:\n" << F << endl;//Output fundamental matrix.
                                cout << "Done Calibration" << endl;

                                cout << "Starting Rectification" << endl;
                                cv::Mat R1, R2, P1, P2, Q;
                                stereoRectify(K1, D1, K2, D2, IRimg.size(), R, T, R1, R2, P1, P2, Q);

                                cout << "R1:\n" << R1 << endl;//Rotation Matrix
                                cout << "R2:\n" << R2 << endl;//Rotation Matrix
                                cout << "P1:\n" << P1 << endl;//3x4Projection matrix
                                cout << "P2:\n" << P2 << endl;//3x4Projection matrix
                                cout << "Q:\n" << Q << endl;//4x4 disparity-to-depth mapping matrix
                                cout << "Done Rectification" << endl;

                                cout << "Starting Undistord" << endl;
                                cv::initUndistortRectifyMap(K1, D1, R1, P1, IRimg.size(), CV_32FC1, map1x, map1y);
                                cv::initUndistortRectifyMap(K2, D2, R2, P2, IRimg.size(), CV_32FC1, map2x, map2y);
                                cout << "Done Undistortion" << endl;

                                cout << "Starting remmaping" << endl;
                                remap(RGBimg, RGB_out, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
                                remap(IRimg, IR_out, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
                                cout << "Done remmaping" << endl;
                                remap_flag = 1;


                                // namedWindow("RGB original", WINDOW_AUTOSIZE);
                                // moveWindow("RGB original", 40, 30);
                                // imshow("RGB original", RGBimg);

                                // namedWindow("IR original",WINDOW_AUTOSIZE);
                                // moveWindow("IR original", 600, 30);
                                // imshow("IR original", IRimg);

                                namedWindow("RGB rectification", WINDOW_AUTOSIZE);
                                moveWindow("RGB rectification", 1000, 10);
                                imshow("RGB rectification", RGB_out);

                                namedWindow("IR rectification",WINDOW_AUTOSIZE);
                                moveWindow("IR rectification", 40, 30);
                                imshow("IR rectification", IR_out);

                                waitKey(0);
                        break;
                        }
                        default:
                        {
                                end_flag = 0;
                        break;
                        }
                }
                cv::destroyAllWindows();
                waitKey(100);
        }
        //! [stop]
        dev->stop();
        dev->close();
        //! [stop]
        //! [image loop]
        delete registration;
        std::cout << "Goodbye World!" << std::endl;
return 0;
}
