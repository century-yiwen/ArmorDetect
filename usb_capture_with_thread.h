////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture_with_thread.h
///@brief: 由于采集图像，摄像头传输数据需要一定时间，单独开启线程采集图像，采用生产者模式，
/// 进行图像采集。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-6
///修订历史：
///2017.12.29 ：解决帧延迟，，即获取的图片为最新的。可实时调整分辨率，
/// 曝光时间参数。(参考2016DJI开源代码)
///2018.3.25：可实时调整相机参数：分辨率，曝光时间等。（新增部分参数调整接口）
/// 增加相机相关参数接口，如相机内参，相机畸变参数，相机云台坐标系偏移量，相机枪管安装偏
/// 移量。(新增)
///2018.3.26:相机参数采用yaml配置文件进行读取。完成该版本的开发。
///2018.5.1:增加相机号遍历重连方法，解决usb突然松动导致相机号变动问题。（只适应只有一个
/// 相机的情况在getImg方法中失败时调用）
///2018.5.17:取消机相关参数接口（由于用处不大，直接读取配置文件就可以了），
/// 取消相机重连接口（udev固定设备路径，图片获取失败，重新初始化即可）
///2018.6.10:采用多线程方式采集图像，与图像处理部分并行，参考robomaster DJI2016开源代码
///接口基本上兼容之前相机版本。
///2018.6.11:取消自动曝光参数调节（实现过程中出现一些问题），目前只能手动曝光调节。
///2018.6.22:采用线程互斥锁，以及更新标志位，解决线程之间变量访问问题，主要是采集的图片数据
/// 的读写互斥，同时采用中间变量mImgImp,以保证程序稳定性。
///2018.7.15:部分细节优化，改变分辨率先检测当前分辨率，增加采集图像流开关控制。
////////////////////////////////////////////////////////////////////////////////


#ifndef LIB_USB_CAPTURE_WITH_THREAD_H
#define LIB_USB_CAPTURE_WITH_THREAD_H

#include <opencv2/opencv.hpp>
#include "base_thread.h"


class UsbCaptureWithThread:public BaseThread{
public:
    UsbCaptureWithThread();
    ~UsbCaptureWithThread();
private:
    std::string mVideoPath;
    int mVideofd;//设备文件描述符
    bool mIsInit;
    bool mIsOpen;
    //内存映射buffer
    struct MapBuffer {
        uint8_t * pBuffer;
        unsigned int len;
        int width;
        int height;
    };
    MapBuffer mBuffer;//缓存区
    cv::Mat mImgTmp,mImg;//解码后的图像
    //线程标志，消费者和生产者模型
    bool mIsUpdate;
    pthread_mutex_t imgMutex = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁
    pthread_mutex_t SigMutex = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁
    //相机图像分辨率
    int mCaptureWidth;
    int mCaptureHeight;
    //unsigned int mImgFormat;//获取图像格式，和摄像头相关
    bool mIsChangeFormat;
    bool mIsCapture;

private:
    int setVideoFormat();
    int init_mmap();
    int startStream();
    int closeStream();
    void cvtRaw2Mat(const void * data, cv::Mat & image);
    int refreshVideoFormat();

    /** 相机初始化函数(内部使用，外部禁止调用)*/
    int init();
    int GetImgRaw();
    void run();
public:

    /** 相机初始化函数(不包括相机参数初始化)
     * @param:const char * device,相机设备名
    *  @param: int width,相机分辨率的宽度值。
    *  @param: int height,相机分辨率的高度值。
    *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
    */
    int init(std::string videoPath, int width, int height);

    /** 图片获取函数
     *  @param: Mat &img,　Mat类的引用，目标图像存放。
     *  @return: int ,错误码，0代表无错误。
     */
    int getImg(cv::Mat &img);


    /** 相机信息打印函数
     *  @param: void
     *  @return: int ,错误码，0代表无错误。
     *  @note:  包括相机支持的功能，以及当前分辨率，格式，FPS.
     */
    int infoPrint();

    /** 检测相机是否打开函数
     *  @param: void
     *  @return: bool ,ture代表正常打开，false代表未打开或异常关闭
     */
    bool isOpen();

    /** 相机曝光设置函数
     *  @param: bool auto_exp,是否为自动曝光模式。
     *  @param: int t,手动曝光模式的曝光时间设置。
     *  @return: int ,错误码，0代表无错误。
     *  @note:  如果auto_exp为true，为自动曝光模式，则第二个参数t无效。
     */
    int setExposureTime(int value);

    //设置色彩饱和度0-128
    int setSaturation(int value);

    //设置白平衡
    int setWhiteBalance(int value);

    //设置亮度
    int setBrightness(int value);

    //设置伽玛值
    int setGamma(int value);

    //设置对比度
    int setContrast(int value);

    /** 相机格式改变函数
     *  @param: int width,相机分辨率的宽度值。
     *  @param: int height,相机分辨率的高度值。
     *  @param: bool mjpeg,是否设置为MJPEG格式。默认为true。
     *  @return: int ,错误码，0代表无错误。
     */
    int changeVideoFormat(int width, int height);
    void setCaptureState(bool isCapture);
};

#endif //LIB_USB_CAPTURE_WITH_THREAD_H
