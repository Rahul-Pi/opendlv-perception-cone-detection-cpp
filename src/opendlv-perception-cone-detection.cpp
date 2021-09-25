/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
float aim_angle_kiwi_prev{1.57f};
int intersection_detected{0};
cv::Point2f aim_point_prev = cv::Point2f(0,0);
float aim_angle_kiwi{};

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
            std::mutex distancesMutex;
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
                auto senderStamp = env.senderStamp();
                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

                // Store distance readings.
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;
                

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                cv::Mat original = img.clone();
                
                //Polygon fill top
                int width = img.size().width;
                int height = img.size().height;
                cv::Point rook_points[1][4];
                rook_points[0][0]  = cv::Point(0,0);
                rook_points[0][1]  = cv::Point(0,height * 0.55);
                rook_points[0][2]  = cv::Point(width,height * 0.55);
                rook_points[0][3]  = cv::Point(width,0);
                
                const cv::Point* ppt[1] = { rook_points[0] };
                cv::Scalar const poly_fill_color(225, 225, 225);
                int npt[] = {4};
                int lineType = cv::LINE_8;
                cv::fillPoly(img, ppt, npt, 1, poly_fill_color, lineType);
                
                
                // Polygon fill kiwi
                cv::Point rook_points_new[1][6];
                rook_points_new[0][0]  = cv::Point(0,height);
                rook_points_new[0][1]  = cv::Point(width*1/5,4.65/5*height);
                rook_points_new[0][2]  = cv::Point(width*1/5,height*4.25/5);
                rook_points_new[0][3]  = cv::Point(width*4/5,height*4.25/5);
                rook_points_new[0][4]  = cv::Point(width*4/5,4.65/5*height);
                rook_points_new[0][5]  = cv::Point(width,height);
                
                const cv::Point* ppt_new[1] = { rook_points_new[0] };
                int npt_new[] = {6};
                cv::fillPoly(img, ppt_new, npt_new, 1, poly_fill_color, lineType);
                
                
                // image transformation starts
                cv::Mat img_copy = img.clone();
                cv::Mat img_edited;
                cv::Mat blueMask;
                cv::Mat yellowMask;
                cv::Mat redMask;
                
                // Yellow Cone Masking
                cv::cvtColor(img, img_edited, cv::COLOR_BGR2HSV);
                cv::Scalar const lower_yellow(15, 5, 5);
                cv::Scalar const upper_yellow(45, 255, 255);
                cv::inRange(img_edited, lower_yellow, upper_yellow, yellowMask);
                
                
                // Red Cone Masking
                cv::Scalar const lower_red(170, 110, 95);
                cv::Scalar const upper_red(180, 255, 255);
                cv::inRange(img_edited, lower_red, upper_red, redMask);
                
                
                // Blue cone masking
                cv::cvtColor(img_copy, img_edited, cv::COLOR_BGR2HSV);
                cv::bitwise_not(img_edited, img_edited);
                cv::Scalar const lower_blue(135, 0, 0);
                cv::Scalar const upper_blue(170, 225, 225);
                cv::inRange(img_edited, lower_blue, upper_blue, blueMask);
                

                // Morphing both the blue and yellow masks
                cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(-1,1));
                
                for (int i = 0; i < 3; i++){
                
                   // Yellow Mask Morphing
                   cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_OPEN, element, cv::Point(-1,-1), 1, 1, 1);
                   cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 3, 1, 1);
                   cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_DILATE, element, cv::Point(-1, -1), 2, 1, 1);
                   
                   // Red Mask Morphing
                   cv::morphologyEx(redMask, redMask, cv::MORPH_OPEN, element, cv::Point(-1,-1), 1, 1, 1);
                   cv::morphologyEx(redMask, redMask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 3, 1, 1);
                   cv::morphologyEx(redMask, redMask, cv::MORPH_DILATE, element, cv::Point(-1, -1), 2, 1, 1);
                   
                   cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, element, cv::Point(-1,-1), 1, 1, 1);
                   cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 3, 1, 1);
                   cv::morphologyEx(blueMask, blueMask, cv::MORPH_DILATE, element, cv::Point(-1, -1), 2, 1, 1);
                }
                
                img_edited=yellowMask+blueMask;
                   
            
                // Yellow cone detection and marking
                cv::Mat yellowMask_copy = yellowMask.clone();
                std::vector<std::vector<cv::Point>> yellow_cnts;
                cv::findContours(yellowMask_copy, yellow_cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                
                std::vector<cv::Moments> M_yellow;
                
                for(uint32_t i = 0; i < yellow_cnts.size(); i++){
                   cv::RotatedRect fullRect = cv::minAreaRect(yellow_cnts[i]); 
                   cv::Rect boxrect = fullRect.boundingRect();
                   if ((boxrect.area() > 800 && boxrect.area() < 4000) && boxrect.width < 0.88*boxrect.height) {
                      M_yellow.push_back(cv::moments(yellow_cnts[i], false));
                   }
                }
                std::vector<cv::Point2f> center_circle(M_yellow.size());
                for(uint32_t i = 0; i < M_yellow.size(); i++){
                   center_circle[i]=cv::Point2f((float)(M_yellow[i].m10/M_yellow[i].m00) , (float)(M_yellow[i].m01/M_yellow[i].m00));
                   cv::Scalar color = cv::Scalar(255,0,0);
                   circle(original, center_circle[i], 4, color, -1, 8, 0 );
                }
                
                
                // Red cone detection and marking
                cv::Mat redMask_copy = redMask.clone();
                std::vector<std::vector<cv::Point>> red_cnts;
                cv::findContours(redMask_copy, red_cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                
                std::vector<cv::Moments> M_red;
                
                for(uint32_t i = 0; i < red_cnts.size(); i++){
                   cv::RotatedRect fullRect = cv::minAreaRect(red_cnts[i]); 
                   cv::Rect boxrect = fullRect.boundingRect();
                   if ((boxrect.area() > 1000 && boxrect.area() < 4250) && boxrect.width < 0.88*boxrect.height ) {
                      M_red.push_back(cv::moments(red_cnts[i], false));
                   }
                }
                std::vector<cv::Point2f> center_circle_red(M_red.size());
                for(uint32_t i = 0; i < M_red.size(); i++){
                   center_circle_red[i]=cv::Point2f((float)(M_red[i].m10/M_red[i].m00) , (float)(M_red[i].m01/M_red[i].m00));
                   cv::Scalar color_red = cv::Scalar(0,255,255);
                   circle(original, center_circle_red[i], 4, color_red, -1, 8, 0 );
                }
                
                
                // Blue cone detection and marking
                cv::Mat blueMask_copy = blueMask.clone();
                std::vector<std::vector<cv::Point>> blue_cnts;
                cv::findContours(blueMask_copy, blue_cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                
                std::vector<cv::Moments> M_blue;
                
                for(uint32_t i = 0; i < blue_cnts.size(); i++){
                      cv::RotatedRect fullRect = cv::minAreaRect(blue_cnts[i]); 
                      cv::Rect boxrect = fullRect.boundingRect();
                      if ((boxrect.area() > 700 && boxrect.area() < 4000)  && boxrect.width < 0.88*boxrect.height) {                   
                          M_blue.push_back(cv::moments(blue_cnts[i], false));
                      }
                   
                }
                std::vector<cv::Point2f> center_circle_blue(M_blue.size());
                for(uint32_t i = 0; i < M_blue.size(); i++){
                   center_circle_blue[i]=cv::Point2f((float)(M_blue[i].m10/M_blue[i].m00) , (float)(M_blue[i].m01/M_blue[i].m00));
                   cv::Scalar color_blue = cv::Scalar(0,0,255);
                   circle(original, center_circle_blue[i], 4, color_blue, -1, 8, 0 );
                }
                
                
                // Aim point determination
                for(uint32_t i = 0; i <= std::max(center_circle_blue.size(),center_circle.size()); i++){
                   cv::Scalar color_c = cv::Scalar(255,255,255);
                   cv::Point2f aim_point_init;
                   
                   if (center_circle_blue.size()>0 && center_circle.size()>0){
                      if((center_circle_blue[0].y+center_circle[0].y)/2<4.1/5*height){
                         aim_point_init = cv::Point2f((center_circle_blue[0].x+center_circle[0].x)/2, (center_circle_blue[0].y+center_circle[0].y)/2);
                      }
                   }
                   else if(center_circle_blue.size()>0){
                       cv::Point2f point_left = cv::Point2f(200,600);
                       aim_point_init = cv::Point2f((center_circle_blue[0].x+point_left.x)/2, (center_circle_blue[0].y+point_left.y)/2);
                   }
                   else if(center_circle.size()>0){
                       cv::Point2f point_right = cv::Point2f(width-200,600);
                       aim_point_init = cv::Point2f((center_circle[0].x+point_right.x)/2, (center_circle[0].y+point_right.y)/2);
                   }
                   else{
                       aim_point_init = cv::Point2f(width/2,height*4.5/5);
                   }
                   
                   
                   // Filtering based on the rate of change... Note: The rate of change beyond 1 radian is ignored and the previous aimpoint is considered.
                   aim_angle_kiwi = atan2((height-aim_point_init.y),(width/2-aim_point_init.x));
                   if (aim_angle_kiwi-aim_angle_kiwi_prev>-1 && aim_angle_kiwi-aim_angle_kiwi_prev<1){
                      aim_angle_kiwi_prev=aim_angle_kiwi;
                      aim_point_prev = cv::Point2f(aim_point_init.x, aim_point_init.y);
                      circle(original, aim_point_init, 7, color_c, -1, 8, 0 );
                   }
                   else{
                      aim_angle_kiwi = aim_angle_kiwi_prev;
                      aim_point_init = cv::Point2f(aim_point_prev.x, aim_point_prev.y);
                      circle(original, aim_point_prev, 7, color_c, -1, 8, 0 );
                   }
                }
                {
                   // Changing the Aim angle from a global frame to a kiwi frame
                   aim_angle_kiwi = 3.14159265f/2 - aim_angle_kiwi;
                }
                
                // Task 3 Finding the intersection
                
                if (std::max(center_circle_blue.size(),center_circle.size())==0 && center_circle_red.size()>3){
                   intersection_detected++;
                   if (intersection_detected>1){
                      intersection_detected = 1;
                      std::cout<<intersection_detected<<"INTERSECTION"<<center_circle_red.size()<<std::endl;
                   }
                }

                // Display image.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), original);
                    cv::waitKey(1);
                }

                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                {
                    std::lock_guard<std::mutex> lck(distancesMutex);
                    std::cout << "front = " << front << ", "
                              << "rear = " << rear << ", "
                              << "left = " << left << ", "
                              << "right = " << right << "." << std::endl;
                }

                ////////////////////////////////////////////////////////////////
                // Sending aim angle message to other microservices;
                opendlv::proxy::AngleReading ar;
                ar.angle(aim_angle_kiwi);
                od4.send(ar);
                
                //Sending intersection message to other microservices;
                opendlv::proxy::SwitchStateRequest intersection_kiwi;
                intersection_kiwi.state(intersection_detected);
                od4.send(intersection_kiwi);

                ////////////////////////////////////////////////////////////////
                // Steering and acceleration/decelration.
                //
                // Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
                // Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
                //opendlv::proxy::GroundSteeringRequest gsr;
                //gsr.groundSteering(0);
                //od4.send(gsr);

                // Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
                // Be careful!
                //opendlv::proxy::PedalPositionRequest ppr;
                //ppr.position(0);
                //od4.send(ppr);
            }
        }
        retCode = 0;
    }
    return retCode;
}

