#include "Anglesolve.h"
#include "usb_capture_with_thread.h"
#include <opencv2/opencv.hpp>
#include "time.h"
#include "UART.h"
#include <iostream>
#include <queue>

using namespace cv;
using namespace std;



#define  lightHM_max   6.7
#define  lightHW_min   2.3
#define  lightPar_angle  5
#define  rotate_angle  17
#define  pi            3.14
#define  horizontal_angle  5
#define  small_width  141
#define  small_high  55
#define  big_width 230
#define  big_high 55


double hor_angle(Point p1, Point p2)
{
    int a, b;
    double rad, angle;
    a = abs(p1.x - p2.x);
    b = abs(p1.y - p2.y);
    if (a != 0)//
    {
        rad = atan(b / a);
        angle = rad * 180 / pi;
    }
    else
        angle = 90;
    return angle;
}

void drawBox(RotatedRect box, Mat img)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }
    box.points(pt); //计算二维盒子顶点
    line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 2, 8, 0);
}

Mat ToHSV(Mat image)
{
    Mat result;
    Mat hsv_image;        //转HSV
    hsv_image.create(image.size(), image.type());
    cvtColor(image, hsv_image, CV_BGR2HSV);

    Mat mask1 = Mat(image.size(), CV_8UC1);
    Mat mask2 = Mat(image.size(), CV_8UC1);

    //result = Mat(image.size(), CV_8UC1);
    //cout << hsv_image << endl;


    //red se diao -6
//    inRange(hsv_image, Scalar(155, 43, 46), Scalar(180, 255, 255), mask1);
//    inRange(hsv_image, Scalar(0, 43, 46), Scalar(10, 255, 255), mask2);
//    result = mask1 + mask2;

    //blue //se diao 40
    inRange(hsv_image, Scalar(100, 43, 46), Scalar(124, 255, 255), mask2);
    result = mask2;
    return result;
}



Mat pretrat(Mat src)
{
    Mat dst;
    Mat element1, element2;
    //cvtColor(src, dst, CV_BGR2GRAY);
    //threshold(dst, dst, 34, 255, CV_THRESH_BINARY);
    /*imshow("提取颜色后", src);
    waitKey(30);*/
    element1 = getStructuringElement(MORPH_RECT, Size(11, 11));
    dilate(src, dst, element1);//腐蚀
    element2 = getStructuringElement(MORPH_RECT, Size(11, 11));
    erode(dst, dst, element2);//膨胀
//    imshow("腐蚀膨胀后", dst);
//    waitKey(30);
    return dst;
}

Point calc_coordinate(vector<RotatedRect> finally)
{
    Point centre;
    if (finally.size() == 0)
    {
        centre.x = 319;
        centre.y = 239;
    }
    else
    {
        centre.x = (finally[0].center.x + finally[1].center.x) / 2;
        centre.y = (finally[0].center.y + finally[1].center.y) / 2;
    }
    return centre;
}

/*Mat track(Mat image, Point centre)
{
Mat track_image;

if (centre.x > 160 && (640 - centre.x) > 160 && centre.y > 120 && (480 - centre.y) > 120)
{
track_image = image(Rect(centre.x - 160, centre.y - 120, 320, 240));

}
else if (centre.x < 160 && (640 - centre.x) > 160 && centre.y > 120 && (480 - centre.y) > 120)
{
track_image = image(Rect(0, centre.y - 120, 320, 240));

}
else if (centre.x > 160 && (640 - centre.x) < 160 && centre.y > 120 && (480 - centre.y) > 120)
{
track_image = image(Rect(centre.x - 160, centre.y - 120, 640 - centre.x, 240));
}
else if (centre.x > 160 && (640 - centre.x) > 160 && centre.y < 120 && (480 - centre.y) > 120)
{
track_image = image(Rect(centre.x - 160, 0, 320, 240));
}
else if (centre.x > 160 && (640 - centre.x) > 160 && centre.y > 120 && (480 - centre.y) < 120)
{
track_image = image(Rect(centre.x, centre.y, 320, 480 - centre.y));
}
else
{
track_image = image;
}

imshow("轨迹预测",track_image);
waitKey(30);

return track_image;
}*/

vector<RotatedRect> Armordetection(Mat image)
{
    //imshow("cap", image);
    //waitKey(30);
    Rect  external_rect;
    float area;
    Mat temp_image = image.clone();
    vector<vector<Point> >  contour;
    findContours(temp_image, contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<RotatedRect>  ellipsee(contour.size());
    vector<RotatedRect>  ellipsee1, ellipsee2, ellipsee3, ellipsee4, finally;
    int count1 = 0;
    for (int i0 = 0; i0 < contour.size(); i0++)
    {
        if (contour[i0].size()>10)
        {
            ellipsee[i0] = fitEllipse(contour[i0]);
            external_rect = ellipsee[i0].boundingRect();
            area = external_rect.area();
            if ((area>170) && external_rect.height>0 && external_rect.width>0 && (((double)external_rect.height / (double)external_rect.width)<lightHM_max &&
                ((double)external_rect.height / (double)external_rect.width)>2) && (ellipsee[i0].angle<rotate_angle || ellipsee[i0].angle>(180 - rotate_angle)))
            {
                ellipsee1.push_back(ellipsee[i0]);
                count1++;
            }
        }
    }

    int i2, j2;
    int flag = 1;
    RotatedRect temp;
    for (i2 = 1; i2 < ellipsee1.size() && flag == 1; i2++)
    {
        flag = 0;
        for (j2 = 0; j2 < ellipsee1.size() - i2; j2++)
        {
            if (ellipsee1[j2].size.height < ellipsee1[j2 + 1].size.height)
            {
                flag = 1;
                temp = ellipsee1[j2];
                ellipsee1[j2] = ellipsee1[j2 + 1];
                ellipsee1[j2 + 1] = temp;
            }
        }
    }




    int count2 = 0;
    int horizontal_angle_real;
    RotatedRect temp1;
    for (int i1 = 0; i1 < count1; i1++)
    {
        for (int j1 = i1 + 1; j1 < count1; j1++)
        {
            horizontal_angle_real = hor_angle(ellipsee1[i1].center, ellipsee1[j1].center);
            if ((abs((ellipsee1[i1].angle - ellipsee1[j1].angle))<lightPar_angle || abs(abs(180 - ellipsee1[i1].angle) - ellipsee1[j1].angle)<lightPar_angle || abs(abs(180 - ellipsee1[j1].angle) - ellipsee1[i1].angle)<lightPar_angle)
                && horizontal_angle_real < horizontal_angle && (abs((ellipsee1[i1].center.x - ellipsee1[j1].center.x))<150) && abs((ellipsee1[i1].center.y - ellipsee1[j1].center.y))<50)
            {
                if (abs((ellipsee1[i1].size.width*ellipsee1[i1].size.height) - (ellipsee1[j1].size.width*ellipsee1[j1].size.height)) <700 && (abs(ellipsee1[i1].center.x - ellipsee1[j1].center.x) / ellipsee1[i1].size.height) < 3)
                {
                    ismall = true;
                    ellipsee2.push_back(ellipsee1[i1]);
                    ellipsee2.push_back(ellipsee1[j1]);
                    count2++;
                    if (ellipsee2.size() == 2)
                    {
                        break;
                    }
                }
            }//小装甲
             if ((abs((ellipsee1[i1].angle - ellipsee1[j1].angle)) < lightPar_angle || abs(abs(180 - ellipsee1[i1].angle) - ellipsee1[j1].angle) < lightPar_angle || abs(abs(180 - ellipsee1[j1].angle) - ellipsee1[i1].angle) < lightPar_angle)
                && horizontal_angle_real < horizontal_angle && (abs((ellipsee1[i1].center.x - ellipsee1[j1].center.x)) < 350) && (abs((ellipsee1[i1].center.x - ellipsee1[j1].center.x)) >100 ) && abs((ellipsee1[i1].center.y - ellipsee1[j1].center.y)) < 30)
            {
                if (abs((ellipsee1[i1].size.width*ellipsee1[i1].size.height) - (ellipsee1[j1].size.width*ellipsee1[j1].size.height)) <1600 && (abs(ellipsee1[i1].center.x - ellipsee1[j1].center.x) / ellipsee1[i1].size.height) > 1.98/* &&
                    abs(ellipsee1[i1].size.height-ellipsee1[j1].size.height)<100*/)
                {

                    ismall = false;
                    ellipsee2.push_back(ellipsee1[i1]);
                    ellipsee2.push_back(ellipsee1[j1]);
                    count2++;
                    if (ellipsee2[0].center.x > ellipsee2[1].center.x)
                    {
                        temp1 = ellipsee2[0];
                        ellipsee2[0] = ellipsee2[1];
                        ellipsee2[1] = temp1;
                    }
                    if ( ellipsee2.size()==2 && (ellipsee2[0].angle > 5 && ellipsee2[1].angle<175)&& (!(ellipsee2[0].angle > 50 && ellipsee2[1].angle>50)|| (ellipsee2[0].angle < 50 && ellipsee2[1].angle < 50)) /*&& abs(ellipsee2[0].size.width*ellipsee2[0].size.height- ellipsee2[1].size.width*ellipsee2[1].size.height)<700*/)
                    {
                        ellipsee2.pop_back();
                        ellipsee2.pop_back();
                    }
                    if (ellipsee2.size() == 2)
                    {
                        break;
                    }
                }
            }//大装甲
        }
        if (ellipsee2.size() == 2)
        {
            break;
        }
    }
    finally = ellipsee2;
    return finally;
}


vector<Point2f>  Four_peak_coordinate(vector<RotatedRect> result_rect, Mat image)
{
    RotatedRect left_light, right_light;
    Point2f  pt1[4], pt2[4];
    vector<Point2f>  pt3;
    if (result_rect.size() != 2)
    {
        return pt3;
    }
    else
    {
        if (result_rect[0].center.x > result_rect[1].center.x)
        {
            left_light = result_rect[1];
            right_light = result_rect[0];
            left_light.points(pt1);
            right_light.points(pt2);

            int ii, jj;
            int flag1 = 1;
            Point2f temp1;

            for (ii = 1; ii < 4 && flag1 == 1; ii++)
            {
                flag1 = 0;
                for (jj = 0; jj < 4 - ii; jj++)
                {
                    if (pt1[jj].x > pt1[jj + 1].x)
                    {
                        flag1 = 1;
                        temp1 = pt1[jj];
                        pt1[jj] = pt1[jj + 1];
                        pt1[jj + 1] = temp1;
                    }
                }
            }

            if (pt1[0].y < pt1[1].y)
            {
                pt3.push_back(pt1[0]);//左上角顶点
                pt3.push_back(pt1[1]);//左下角顶点
            }
            else
            {
                pt3.push_back(pt1[1]);
                pt3.push_back(pt1[0]);
            }

            int ii2, jj2;
            int flag2 = 1;
            Point2f temp2;

            for (ii2 = 1; ii2 < 4 && flag2 == 1; ii2++)
            {
                flag2 = 0;
                for (jj2 = 0; jj2 < 4 - ii2; jj2++)
                {
                    if (pt2[jj2].x < pt2[jj2 + 1].x)
                    {
                        flag2 = 1;
                        temp2 = pt2[jj2];
                        pt2[jj2] = pt2[jj2 + 1];
                        pt2[jj2 + 1] = temp2;
                    }
                }
            }

            if (pt2[0].y > pt2[1].y)
            {
                pt3.push_back(pt2[0]);
                pt3.push_back(pt2[1]);
            }
            else
            {
                pt3.push_back(pt2[1]);
                pt3.push_back(pt2[0]);
            }



        }
        else
        {
            left_light = result_rect[0];
            right_light = result_rect[1];
            left_light.points(pt1);
            right_light.points(pt2);

            int ii, jj;
            int flag1 = 1;
            Point2f temp1;

            for (ii = 1; ii < 4 && flag1 == 1; ii++)
            {
                flag1 = 0;
                for (jj = 0; jj < 4 - ii; jj++)
                {
                    if (pt1[jj].x > pt1[jj + 1].x)
                    {
                        flag1 = 1;
                        temp1 = pt1[jj];
                        pt1[jj] = pt1[jj + 1];
                        pt1[jj + 1] = temp1;
                    }
                }
            }

            if (pt1[0].y < pt1[1].y)
            {
                pt3.push_back(pt1[0]);//左上角顶点
                pt3.push_back(pt1[1]);//左下角顶点
            }
            else
            {
                pt3.push_back(pt1[1]);
                pt3.push_back(pt1[0]);
            }

            int ii2, jj2;
            int flag2 = 1;
            Point2f temp2;

            for (ii2 = 1; ii2 < 4 && flag2 == 1; ii2++)
            {
                flag2 = 0;
                for (jj2 = 0; jj2 < 4 - ii2; jj2++)
                {
                    if (pt2[jj2].x < pt2[jj2 + 1].x)
                    {
                        flag2 = 1;
                        temp2 = pt2[jj2];
                        pt2[jj2] = pt2[jj2 + 1];
                        pt2[jj2 + 1] = temp2;
                    }
                }
            }

            if (pt2[0].y > pt2[1].y)
            {
                pt3.push_back(pt2[0]);
                pt3.push_back(pt2[1]);
            }
            else
            {
                pt3.push_back(pt2[1]);
                pt3.push_back(pt2[0]);
            }


        }
        line(image, pt3[0], pt3[1], CV_RGB(255, 0, 0), 2, 8, 0);
        line(image, pt3[1], pt3[2], CV_RGB(255, 0, 0), 2, 8, 0);
        line(image, pt3[2], pt3[3], CV_RGB(255, 0, 0), 2, 8, 0);
        line(image, pt3[3], pt3[0], CV_RGB(255, 0, 0), 2, 8, 0);

        return pt3;
    }
}

Point2d add_speedforecast(Point2d d_angle,queue<Point2d> *add_speed)
{
    Point2d d_angle_1,d_angle_2,d_add_speed;
    if (add_speed->size()< 2)
     {
         add_speed->push(d_angle);
     }
     else
    {
        d_angle_1=add_speed->front();
        d_angle_2=add_speed->back();
        d_add_speed.x=d_angle_2.x-d_angle_1.x;
        d_add_speed.y=d_angle_2.y-d_angle_1.y;
        add_speed->push(d_angle);
        add_speed->pop();
        cout << "差差值角度：" << "  " << d_add_speed.x << "," << d_add_speed.y << endl;
    }
    return d_add_speed;
}


Point2d Forecast(double angle_x, double angle_y, queue<Point2d> *forecast)
{
     Point2d d_angle;
     Point2d angle;
     int number = forecast->size();
    //cout << "队列中个数：" << "  " << number << endl;
        if (forecast->size()< 2)
        {
            angle.x = angle_x;
            angle.y = angle_y;
            forecast->push(angle);
        }
        else
        {
            Point2d angle_1, angle_2;
            angle.x = angle_x;
            angle.y = angle_y;
            angle_1 = forecast->front();
            angle_2 = forecast->back();
            //cout << "初始角度：" << "  " << angle_x << "," << angle_y << endl;
            d_angle.x = angle_2.x - angle_1.x;
            d_angle.y = angle_2.y - angle_1.y;
//            dd_angle=add_speedforecast(d_angle);
            cout << "差值角度：" << "  " << d_angle.x << "," << d_angle.y << endl;
//            cout << "差差值角度：" << "  " << dd_angle.x << "," << dd_angle.y << endl;
            forecast->push(angle);
            forecast->pop();
//            angle.x = angle.x + k1*d_angle.x+kk1*dd_angle.x;
//            angle.y = angle.y + 0*d_angle.y+kk2*dd_angle.y;
            //cout << "预测角度：" << "  " << angle.x << "," << angle.y << endl;
        }
        return d_angle;
}



int main()
{
    //image = imread("cap2.png");

    UsbCaptureWithThread *cap=new UsbCaptureWithThread();
    cap->init("/dev/video0",1280,720);
    queue<Point2d> forecast;
    queue<Point2d> forecast_2;
    //capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);//宽度
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//高度
    //capture.set(CV_CAP_PROP_FPS, 120);//帧率 帧/秒
    //capture.set(CV_CAP_PROP_BRIGHTNESS, -64);//亮度
    //capture.set(CV_CAP_PROP_CONTRAST, 0);//对比度
    //capture.set(CV_CAP_PROP_SATURATION, 128);//饱和度
    //capture.set(CV_CAP_PROP_HUE, -6);//色调 50
    //capture.set(CV_CAP_PROP_EXPOSURE, -9);//曝光

    while (1)
    {
//        clock_t start, finish;
//        double  duration;
//        start = clock();

        //打印件摄像头参数640*480
//        Mat cam_matrix = (Mat_<double>(3, 3) << 912.1696, 0, 337.7947, 0 , 918.0548, 212.0634, 0, 0, 1);
//        Mat distortion_coeff = (Mat_<double>(1, 5) << -0.4006, 0.2815, 0, 0, 0);

        //ptint cam 1280*720
        Mat cam_matrix = (Mat_<double>(3, 3) << 1385.83, 0, 661.0446, 0 , 1407.8, 296.6695, 0, 0, 1);
        Mat distortion_coeff = (Mat_<double>(1, 5) << -0.4451, 0.6231, 0, 0, 0);

        //ov2710摄像头参数
//        Mat cam_matrix = (Mat_<double>(3, 3) << 612.8550, 0, 341.5985, 0, 612.4596, 226.1547, 0, 0, 1);
//        Mat distortion_coeff = (Mat_<double>(1, 5) << -0.4315, 0.1799, 0, 0, 0);

        vector<RotatedRect>  result;
        //Point centre;
        Mat image;
        double distance;
        cap->getImg(image);
        //image = imread("红色的灯柱.png");
        //image = caliberation(image);
        if (!image.empty())
        {
            Size img_size;
            img_size = image.size();
            Mat threshold = Mat(img_size, CV_8UC1);
            threshold=ToHSV(image);
            threshold = pretrat(threshold);
            result = Armordetection(threshold);
            for (int nI = 0; nI < result.size(); nI++)
            {
                drawBox(result[nI], image);
            }
            //       final_peak=Four_peak_coordinate(result, image);
            if(ismall==true)
            {
                AngleSolver solver(cam_matrix, distortion_coeff, small_width, small_high, 1.0);
                const double barrel_ptz_offset_y = 0;
                const double ptz_camera_y = 0;
                const double ptz_camera_z = 0;
                double t_data[] = { 0, ptz_camera_y, ptz_camera_z }; // ptz org position in camera coodinate system
                Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
                solver.setRelationPoseCameraPTZ(t_camera_ptz, barrel_ptz_offset_y);
                double angle_y = 0.0, angle_x = 0.0;
                if (true == solver.getAngle(result, angle_x, angle_y, distance, 40.00))
                {

                    //cout << angle_x << "," << angle_y << endl;
                    Point2d d_angle = Forecast(angle_x, angle_y, &forecast);
                    Point2d dd_angle = add_speedforecast(d_angle,&forecast_2);
                    angle_x = ((angle_x+1.5)+(1*d_angle.x)+3*dd_angle.x);
                    angle_y = angle_y+(1*d_angle.y)+3*dd_angle.y;
                    Sendata((int)(angle_x*100), (int)(angle_y*100));
                    cout << (int)(angle_x*100) << "," << (int)(angle_y*100)<< endl;
                }
                else
                {
                    Sendata(0, 0);
                    while (!forecast.empty())
                    {
                        forecast.pop();
                    }

                }
            }
            if(ismall==false)
            {
                AngleSolver solver(cam_matrix, distortion_coeff, big_width, big_high, 1.0);
                const double barrel_ptz_offset_y = 0;
                const double ptz_camera_y = 0;
                const double ptz_camera_z = 0;
                double t_data[] = { 0, ptz_camera_y, ptz_camera_z }; // ptz org position in camera coodinate system
                Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
                solver.setRelationPoseCameraPTZ(t_camera_ptz, barrel_ptz_offset_y);
                double angle_y = 0.0, angle_x = 0.0;
                if (true == solver.getAngle(result, angle_x, angle_y, distance, 30.00))
                {

                    //cout << angle_x << "," << angle_y << endl;
                    Point2d d_angle = Forecast(angle_x, angle_y, &forecast);
                    Point2d dd_angle = add_speedforecast(d_angle,&forecast_2);
                    angle_x = ((angle_x+1.5)+(2*d_angle.x)+3*dd_angle.x);
                    angle_y = angle_y+(2*d_angle.y)+3*dd_angle.y;
                    Sendata((int)(angle_x*100), (int)(angle_y*100));
                    cout << (int)(angle_x*100) << "," << (int)(angle_y*100)<< endl;
                }
                else
                {
                    Sendata(0, 0);
                    while (!forecast.empty())
                    {
                        forecast.pop();
                    }

                }
            }
//centre = calc_coordinate(result);
//            finish = clock();
//            duration = (double)(finish - start) / CLOCKS_PER_SEC;
//            cout << duration << "秒" << endl;
//            imshow("最终图", image);
//            waitKey(30);

        }
    }
    return 0;

}

