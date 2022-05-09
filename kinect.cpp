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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string> 
#include <sstream> 
#include <thread>

const int FALSE = 0;
const int TRUE = 1;

using namespace std;
using namespace cv;

enum Processor { cl, gl, cpu };

void CallBackFunc(int event, int x, int y, int flags, void* params)
{

        cv::Mat *image = (Mat*)params;

        // if  ( event == EVENT_LBUTTONDOWN )
        // {
        //         //int pixelValue = static_cast<int>(image->at<float>(cv::Point(x,y)));
        //         cout << "The pixel value at (" << x << ", " << y << ") : " << endl;
        // }
        // else if  ( event == EVENT_RBUTTONDOWN )
        // {
        //         cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        // }
        // else if  ( event == EVENT_MBUTTONDOWN )
        // {
        //         cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        // }
        // else if ( event == EVENT_MOUSEMOVE )
        // {
        //         std::stringstream text;
        //         text << "Pos.(" << x << ", " << y << ")";
        //         //cv::putText(image, text.str(), cv::Point(x,y), FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0,255,255), 1,8);
        //         cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        // }
}

int MenuSelect()
{
        int menu_item;
        std::cout << "#################################"<<std::endl;
        std::cout << "1) Camara output"<<std::endl;
        std::cout << "2) Calibration"<<std::endl;
        std::cout << "3) End program"<<std::endl;
        std::cout << "#################################"<<std::endl;
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
        std::cout << "'a' Apply to RGB calibration with RGB alignment"<<std::endl;
        std::cout << "'p' Point cloud"<<std::endl;
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

                                        cv::resize(rgbmat,rgbmat,Size(512,424),0,0, INTER_AREA); //resize rgbmat from 1920x1080 to 512x424

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


                                        if((key_press == 99) || (key_press == 97) || key_press == 112){
                                                if(remap_flag){
                                                        remap(depthmat, depthmat, map1x, map1y, 
                                                                cv::INTER_NEAREST);
                                                                //INTER_AREA, BORDER_TRANSPARENT, Scalar());
                                                        remap(rgbmat, rgbmat, map2x, map2y, 
                                                                cv::INTER_NEAREST);
                                                                //INTER_AREA, BORDER_TRANSPARENT, Scalar());
                                                }
                                        }

                                        if((key_press == 115) || (key_press == 97)||(key_press == 112)){      //115 igual a 's' en el teclado (no esta ligado a ascii)
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
                                        if(key_press == 112){//create pointcloud with calibration
                                                const float badPoint = std::numeric_limits<float>::quiet_NaN();
                                                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

                                                for (int i = 0; i<depthmat.rows; i++)
                                                {
                                                        for (int j = 0; j < depthmat.cols; j++)
                                                        {
                                                                pcl::PointXYZRGBA point;
                                                                point.z =  depthmat.at<float>(i,j);
                                                                point.x = i;
                                                                point.y = j;
                                                                uint32_t rgb = (static_cast<uint32_t>(rgbmat.at<cv::Vec4b>(cv::Point(j,i))[2]) << 16
                                                                 | static_cast<uint32_t>(rgbmat.at<cv::Vec4b>(cv::Point(j,i))[1]) << 8
                                                                  | static_cast<uint32_t>(rgbmat.at<cv::Vec4b>(cv::Point(j,i))[0]));
                                                                point.rgb = *reinterpret_cast<float*>(&rgb);
                                                                point.a = badPoint;
                                                                ptCloud->points.push_back(point);
                                                                //std::cout <<" z: "<<point.z<<" x: "<<point.x<<" y: "<<point.y<<" rgb: "<<point.rgb<<" a: "<<point.a<< std::endl;
                                                        }
                                                }
                                                ptCloud->height = rgbmat.cols;
                                                ptCloud->width = rgbmat.rows;
                                                ptCloud->is_dense = FALSE;
                                                
                                                // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
                                                pcl::visualization::CloudViewer viewer("Cloud Viewer");
                                                // const std::string cloudName = "3D Image Calibrated";
                                                // viewer->initCameraParameters();
                                                // viewer->setBackgroundColor(0, 0, 0);
                                                // viewer->addPointCloud(ptCloud, cloudName);
                                                // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
                                                // viewer->addCoordinateSystem(1.0);
                                                // viewer->initCameraParameters();
                                                viewer.showCloud(ptCloud);

                                                while (!viewer.wasStopped ())
                                                {
                                                        //viewer->spinOnce (100);
                                                }
                                                //viewer->close();
                                                key_press = 0;
                                        }

                                        if(key_press == 102){      //102 igual a 's' en el teclado (no esta ligado a ascii)
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
                                                cout << "        " << counter_img_save << endl;
                                                cout << "#################" << endl;
                                                key_press = 0;
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
                                std::vector<cv::Point3f> Board3D;//obj guarda las cordenadas reales de los puntos del tablero z es igual a 0 x y son calculados
                                std::vector<cv::Point2f> RGBcorners2D, IRcorners2D;//corners1, corners2 //findChessboardCorners guarda los puntos encontrados en la imagen por eso son 2d
                                std::vector<std::vector<cv::Point2f>> points2DRGB, points2DIr;//imagePoints1, imagePoints2 //vector de vectores de los puntos encontrados, un vector para cada imagen y un vector para los puntos
                                std::vector<std::vector<cv::Point3f>> Boardcoord3D;//object_points //vector de vectores de los puntos reales de cada imagen

                                std::string RGBdireccion = "/home/fernando/Pictures/Calibracion RGB/RGBcal";
                                std::string IRdireccion = "/home/fernando/Pictures/Calibracion IR/IRcal";
                                std::stringstream RGBimgs;
                                std::stringstream IRimgs;

                                Mat RGBimg, RGBimgGray, IRimg, IRimgGray;
                                bool RGBfound, IRfound;

                                calcChessboardCorners(patternsize,18,Board3D);//(x,18,x)tamano en milimetros de los cuadrados

                                for (int  i = 0; i < 49; i++){

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
                                                        TermCriteria(TermCriteria::COUNT + TermCriteria::COUNT, 100, DBL_EPSILON));
                                                drawChessboardCorners(RGBimg,patternsize,Mat(RGBcorners2D),RGBfound);
                                        }
                                        if(IRfound){
                                                cornerSubPix(IRimgGray, IRcorners2D, Size(11, 11), Size(-1, -1),
                                                        TermCriteria(TermCriteria::COUNT + TermCriteria::COUNT, 100, DBL_EPSILON));
                                                drawChessboardCorners(IRimg,patternsize,Mat(IRcorners2D),IRfound);
                                        }

                                        if(RGBfound && IRfound){
                                                points2DRGB.push_back(RGBcorners2D);
                                                points2DIr.push_back(IRcorners2D);
                                                Boardcoord3D.push_back(Board3D);
                                        }

                                        // namedWindow(("RGBcal"+std::to_string(i)).c_str(),WINDOW_AUTOSIZE);
                                        // moveWindow(("RGBcal"+std::to_string(i)).c_str(), 40, 30);
                                        // imshow(("RGBcal"+std::to_string(i)).c_str(), RGBimg);

                                        // namedWindow(("IRcal"+std::to_string(i)).c_str(),WINDOW_AUTOSIZE);
                                        // moveWindow(("IRcal"+std::to_string(i)).c_str(), 600, 30);
                                        // imshow(("IRcal"+std::to_string(i)).c_str(), IRimg);

                                        // waitKey(500);

                                        // cvDestroyWindow(("RGBcal"+std::to_string(i)).c_str());
                                        // cvDestroyWindow(("IRcal"+std::to_string(i)).c_str());
                                }
                                cv::destroyAllWindows();

                                Mat cameraMatrix_RGB = Mat::eye(3, 3, CV_64F);
                                Mat cameraMatrix_IR = Mat::eye(3, 3, CV_64F);
                                Mat distCoeffs_RGB = Mat::zeros(8, 1, CV_64F);
                                Mat distCoeffs_IR = Mat::zeros(8, 1, CV_64F);
                                std::vector<Mat> rvecs_RGB;
                                std::vector<Mat> rvecs_IR;
                                std::vector<Mat> tvecs_RGB;
                                std::vector<Mat> tvecs_IR;
                           
                                double rms_RGB = calibrateCamera(Boardcoord3D, points2DRGB, RGBimg.size(), cameraMatrix_RGB,
                                          distCoeffs_RGB, rvecs_RGB, tvecs_RGB, 
                                                // CALIB_FIX_PRINCIPAL_POINT +
                                                // CALIB_FIX_ASPECT_RATIO +
                                                // CALIB_ZERO_TANGENT_DIST 
                                                CALIB_RATIONAL_MODEL
                                                ,TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, DBL_EPSILON));
                                double rms_IR = calibrateCamera(Boardcoord3D, points2DIr, IRimg.size(), cameraMatrix_IR,
                                          distCoeffs_IR, rvecs_IR, tvecs_IR,
                                                // CALIB_FIX_PRINCIPAL_POINT +
                                                // CALIB_FIX_ASPECT_RATIO +
                                                // CALIB_ZERO_TANGENT_DIST
                                                CALIB_RATIONAL_MODEL
                                                ,TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 50, DBL_EPSILON));

                                std::cout << "///////////////////////////" << std::endl;
                                std::cout << "RGB RMS: " << rms_RGB << std::endl;
                                std::cout << "RGB Camera matrix: " << cameraMatrix_RGB << std::endl;
                                std::cout << "RGB Distortion _coefficients: " << distCoeffs_RGB << std::endl;
                                std::cout << "///////////////////////////" << std::endl;
                                std::cout << "IR RMS: " << rms_IR << std::endl;
                                std::cout << "IR Camera matrix: " << cameraMatrix_IR << std::endl;
                                std::cout << "IR Distortion _coefficients: " << distCoeffs_IR << std::endl;
                                std::cout << "///////////////////////////" << std::endl;

                                cout << "Starting Calibration" << endl;
                                Mat cameraMatrixIr, cameraMatrixColor, rotation, fundamental, essential, distortionIr, distortionColor, translation;

                                cameraMatrixIr =cameraMatrix_IR;
                                cameraMatrixColor =cameraMatrix_RGB;

                                distortionIr =distCoeffs_IR;
                                distortionColor =distCoeffs_RGB;

                                cout << "Read intrinsics"<< endl;
                                double rms_stereo = stereoCalibrate(Boardcoord3D, points2DIr, points2DRGB, cameraMatrix_IR, distCoeffs_IR, cameraMatrix_RGB, distCoeffs_RGB,
                                                 RGBimg.size(), rotation, translation, essential, fundamental,
                                                 TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 100, DBL_EPSILON), 
                                                 //CV_CALIB_FIX_FOCAL_LENGTH +
                                                 CV_CALIB_USE_INTRINSIC_GUESS +
                                                 CV_CALIB_FIX_INTRINSIC);

                                cout << "RMS Stereo:\n" << rms_stereo << endl;// rms
                                cout << "CameraMatrixIr:\n" << cameraMatrix_IR << endl;// intrinsic matrix
                                cout << "CameraMatrixColor:\n" << cameraMatrix_RGB << endl;// intrinsic matrix
                                cout << "DistortionIr:\n" << distCoeffs_IR << endl;// vector of distortion coefficients
                                cout << "DistortionColor:\n" << distCoeffs_RGB << endl;// vector of distortion coefficients
                                cout << "Rotation:\n" << rotation << endl;//Output rotation matrix between the 1st and the 2nd camera coordinate systems.
                                cout << "Translation:\n" << translation << endl;//Output translation vector between the coordinate systems of the cameras.
                                cout << "Essential:\n" << essential << endl;//Output essential matrix.
                                cout << "fundamental:\n" << fundamental << endl;//Output fundamental matrix.
                                cout << "Done Calibration" << endl;

                                cout << "Starting Rectification" << endl;
                                cv::Mat R1, R2, new_cameraMatrix_IR, new_cameraMatrix_RGB, Q;
                                stereoRectify(cameraMatrix_IR, distCoeffs_IR, cameraMatrix_RGB, distCoeffs_RGB, IRimg.size(),
                                                rotation, translation,
                                                R1, R2, new_cameraMatrix_IR, new_cameraMatrix_RGB, Q
                                                ,CALIB_ZERO_DISPARITY, 0, IRimg.size()
                                                );

                                cout << "R1:\n" << R1 << endl;//Rotation Matrix
                                cout << "R2:\n" << R2 << endl;//Rotation Matrix
                                cout << "P1:\n" << new_cameraMatrix_IR << endl;//3x4Projection matrix
                                cout << "P2:\n" << new_cameraMatrix_RGB << endl;//3x4Projection matrix
                                cout << "Q:\n" << Q << endl;//4x4 disparity-to-depth mapping matrix
                                cout << "Done Rectification" << endl;

                                cout << "Starting Undistord" << endl;
                                cv::Rect rect( 0, 0, 3, 3);
                                new_cameraMatrix_IR = new_cameraMatrix_IR(rect);
                                new_cameraMatrix_RGB = new_cameraMatrix_RGB(rect);

                                cv::initUndistortRectifyMap(cameraMatrix_IR, distCoeffs_IR, R1, new_cameraMatrix_IR, IRimg.size(), CV_32FC1, map1x, map1y);
                                cv::initUndistortRectifyMap(cameraMatrix_RGB, distCoeffs_RGB, R2, new_cameraMatrix_RGB, IRimg.size(), CV_32FC1, map2x, map2y);

                                // cv::initUndistortRectifyMap(cameraMatrix_IR, distCoeffs_IR, cv::Mat(), cameraMatrix_IR, IRimg.size(), CV_32FC1, map1x, map1y);
                                // cv::initUndistortRectifyMap(cameraMatrix_RGB, distCoeffs_RGB, cv::Mat(), cameraMatrix_RGB, IRimg.size(), CV_32FC1, map2x, map2y);
                                cout << "Done Undistortion" << endl;

                                cout << "Starting remmaping" << endl;
                                remap(IRimg, IR_out, map1x, map1y,
                                        cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
                                remap(RGBimg, RGB_out, map2x, map2y,
                                        cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
                                cout << "Done remmaping" << endl;
                                remap_flag = 1;

                                ////////// Draw lines horizontal and verticaly///////////
                                Mat result_h, result_v;
                                hconcat(RGB_out, IR_out, result_h);
                                vconcat(RGB_out, IR_out, result_v);

                                // draw horizontal line
                                for(int j = 0; j < result_h.rows; j += 16 ) {
                                        line(result_h, Point(0, j), Point(result_h.cols, j), Scalar(0, 255, 0), 1, 8);
                                }
                                for(int j = 0; j < result_v.cols; j += 16 ) {
                                        line(result_v, Point(j, 0), Point(j,result_v.rows), Scalar(0, 255, 0), 1, 8);
                                }

                                namedWindow("Result_h", WINDOW_AUTOSIZE);
                                moveWindow("Result_h", 40, 30);
                                imshow("Result_h", result_h);
                                namedWindow("Result_v", WINDOW_AUTOSIZE);
                                moveWindow("Result_v", 40, 30);
                                imshow("Result_v", result_v);
                                //////////////////////

                                // namedWindow("RGB original", WINDOW_AUTOSIZE);
                                // moveWindow("RGB original", 1000, 10);
                                // imshow("RGB original", RGBimg);

                                // namedWindow("IR original",WINDOW_AUTOSIZE);
                                // moveWindow("IR original", 40, 30);
                                // imshow("IR original",  IRimg);

                                // namedWindow("RGB rectification", WINDOW_AUTOSIZE);
                                // moveWindow("RGB rectification", 1000, 10);
                                // imshow("RGB rectification", RGB_out);

                                // namedWindow("IR rectification",WINDOW_AUTOSIZE);
                                // moveWindow("IR rectification", 40, 30);
                                // imshow("IR rectification", IR_out);

                                waitKey(0);
                                cv::destroyAllWindows();
                        break;
                        }
                        case 3:
                        {
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
