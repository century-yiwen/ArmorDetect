////////////////////////////////////////////////////////////////////////////////
///Copyright(c)     UESTC ROBOMASTER2018      UsbCapture Code for robot
///ALL RIGHTS RESERVED
///@file:usb_capture_with_thread.cpp
///@brief: 无。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 18-6-6
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include "usb_capture_with_thread.h"
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"

using namespace std;
using namespace cv;

static int xioctl(int fd, int request, void *arg)
{
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

UsbCaptureWithThread::UsbCaptureWithThread(){
    mVideofd=-1;
    mIsInit= false;
    mIsOpen=false;
    mIsChangeFormat= false;
}

UsbCaptureWithThread::~UsbCaptureWithThread(){
    if(mVideofd!=-1){
        close(mVideofd);
    }
}

int UsbCaptureWithThread::init(){
    if(mIsOpen){
        return 0;
    }
/***1.open device*****/
    mVideofd = open(mVideoPath.c_str(), O_RDWR);
    if (mVideofd == -1)
    {
        printf("Opening video device error\n");
        return 1;
    }
/***2.set format*****/
    if(setVideoFormat()!=0){
        return 2;
    }
    refreshVideoFormat();
/***3.mmap******/
    if(init_mmap()!=0){
        return 3;
    }
    mIsOpen=true;
    return 0;
}



int UsbCaptureWithThread::init(std::string videoPath, int width, int height) {
    if(mIsOpen){
        return -1;
    }
    mVideoPath=videoPath;
    mCaptureWidth=width;
    mCaptureHeight=height;
    mIsInit= true;
    mIsCapture=true;
    int ret=init();
    //启动线程
    start();
    return ret;
}

bool UsbCaptureWithThread::isOpen() {
    return mIsOpen;
}


int UsbCaptureWithThread::setVideoFormat(){
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<__u32>(mCaptureWidth);
    fmt.fmt.pix.height = static_cast<__u32>(mCaptureHeight);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (-1 == xioctl(mVideofd, VIDIOC_S_FMT, &fmt)){
        printf("Setting Pixel Format\n");
        return 1;
    }
    return 0;
}

int UsbCaptureWithThread::init_mmap()
{
    //申请内部缓存数量，1个缓存
    struct v4l2_requestbuffers req = {0};
    req.count =1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(mVideofd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
    //内核空间内存映射到用户空间(0号 buffer)
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index =0;
    if(-1 == xioctl(mVideofd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 2;
    }
    mBuffer.pBuffer = (uint8_t *)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, mVideofd, buf.m.offset);
    mBuffer.len=buf.length;
    mBuffer.width=mCaptureWidth;
    mBuffer.height=mCaptureHeight;
    if(mBuffer.pBuffer == MAP_FAILED){
        perror("MAP_FAILED");
        return 3;
    }
    return 0;
}

int UsbCaptureWithThread::startStream() {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(mVideofd, VIDIOC_STREAMON, &buf.type))
    {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    return 0;
}

int UsbCaptureWithThread::closeStream() {
    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //关闭采集流
    if(ioctl(mVideofd, VIDIOC_STREAMOFF, &type) < 0){
        perror("VIDIOC_STREAMOFF");
        return -1;
    }
    //解除内存映射
    munmap(mBuffer.pBuffer, mBuffer.len);
    return 0;
}

int UsbCaptureWithThread::refreshVideoFormat(){
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(mVideofd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return 1;
    }
    mCaptureWidth = fmt.fmt.pix.width;
    mCaptureHeight = fmt.fmt.pix.height;
    return 0;
}

int UsbCaptureWithThread::changeVideoFormat(int width, int height){
    if((width!=mCaptureWidth)||(height!=mCaptureHeight)){
        mCaptureWidth=width;
        mCaptureHeight=height;
        mIsChangeFormat=true;
    }
    return 0;
}

void UsbCaptureWithThread::cvtRaw2Mat(const void *data, cv::Mat &image){
    Mat src(mBuffer.height, mBuffer.width, CV_8UC1, (void*)data);
    imdecode(src, 1).copyTo(image);
}

int UsbCaptureWithThread::GetImgRaw() {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(mVideofd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        //设备松动重连
        if(mIsOpen){
            closeStream();
            close(mVideofd);
        }
        usleep(50000);//50ms
        mIsOpen=false;
        if(init()==0){
            printf("usb cap reConnect success!\n");
            usleep(50000);//50ms
        } else{
            printf("usb cap reConnect fail!\n");
            usleep(500000);//500ms
        };

        return 2;
    }

    if(startStream()!=0) {
        perror("Start Capture");
        return 3;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(mVideofd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(mVideofd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return 4;
    }
    if(-1 == xioctl(mVideofd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return 5;
    }
    //解码
    cvtRaw2Mat(mBuffer.pBuffer,mImgTmp);
    pthread_mutex_lock(&imgMutex);
    mImgTmp.copyTo(mImg);//写入mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    pthread_mutex_lock(&SigMutex);
    mIsUpdate=true;
    pthread_mutex_unlock(&SigMutex);
    return 0;
}

int UsbCaptureWithThread::getImg(Mat &img){
    if(!mIsUpdate){
        //等待100ms
        int timeCounter=0;
        while(!mIsUpdate&&timeCounter<100){
            usleep(1000);//1ms等待
            timeCounter++;
        }
        if(!mIsUpdate){
            return -3;//更新超时
        }
    }
    pthread_mutex_lock(&imgMutex);
    mImg.copyTo(img);//读mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    if(!img.empty()&&(img.cols==mCaptureWidth)&&(img.rows==mCaptureHeight)){
        pthread_mutex_lock(&SigMutex);
        mIsUpdate= false;
        pthread_mutex_unlock(&SigMutex);
        return 0;
    } else{
        return -1;
    }
}



int UsbCaptureWithThread::setSaturation(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_SATURATION;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Saturation error\n");
        return -1;
    }
    return 0;
}

int UsbCaptureWithThread::setWhiteBalance(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set White Balance error\n");
        return -1;
    }
    return 0;
}
int UsbCaptureWithThread::setBrightness(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_BRIGHTNESS;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Brightness error\n");
        return -1;
    }
    return 0;
}
int UsbCaptureWithThread::setGamma(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_GAMMA;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Gamma error\n");
        return -1;
    }
    return 0;
}

int UsbCaptureWithThread::setContrast(int value) {
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_CONTRAST;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Contrast error\n");
        return -1;
    }
    return 0;
}


int UsbCaptureWithThread::setExposureTime(int value){
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_EXPOSURE_AUTO;
    control_s.value = V4L2_EXPOSURE_MANUAL;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Close MANUAL Exposure error\n");
        return 3;
    }
    control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control_s.value = value;
    if( xioctl(mVideofd, VIDIOC_S_CTRL, &control_s) < 0){
        printf("Set Exposure Time error\n");
        return 1;
    }
    return 0;
}



int UsbCaptureWithThread::infoPrint(){
    struct v4l2_capability caps = {};
    if (-1 == xioctl(mVideofd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilities");
        return 1;
    }

    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);

    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (mVideofd, VIDIOC_CROPCAP, &cropcap))
    {
        perror("Querying Cropping Capabilities");
        return 1;
    }
    printf( "Camera Cropping:\n"
            "  Bounds: %dx%d+%d+%d\n"
            "  Default: %dx%d+%d+%d\n"
            "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    int support_grbg10 = 0;

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(mVideofd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(mVideofd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return 1;
    }
    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
            "  Width: %d\n"
            "  Height: %d\n"
            "  PixFmt: %s\n"
            "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);

    struct v4l2_streamparm streamparm = {0};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(mVideofd, VIDIOC_G_PARM, &streamparm)) {
        perror("Querying Frame Rate\n");
        return 1;
    }
    printf( "Frame Rate:  %f\n====================\n",
            (float)streamparm.parm.capture.timeperframe.denominator /
            (float)streamparm.parm.capture.timeperframe.numerator);

    struct v4l2_control control_g={0};

    //曝光时间
    control_g.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Exposure error\n");
    }else{
        printf("EXPOSURE MODE:%d\n",control_g.value);
    }
    //色彩饱和度
    control_g.id = V4L2_CID_SATURATION ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Saturation error\n");
    }else{
        printf("Saturation:%d\n",control_g.value);
    }
    //对比度
    control_g.id = V4L2_CID_CONTRAST ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Contrast error\n");
    }else{
        printf("Contrast:%d\n",control_g.value);
    }
    //增益
    control_g.id = V4L2_CID_GAIN;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get GAIN error\n");
    }else{
        printf("GAIN:%d\n",control_g.value);
    }
    //伽玛值
    control_g.id = V4L2_CID_GAMMA;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Gamma error\n");
    }else{
        printf("Gamma:%d\n",control_g.value);
    }
    //亮度
    control_g.id = V4L2_CID_BRIGHTNESS;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get Brightness error\n");
    }else{
        printf("Brightness:%d\n",control_g.value);
    }
    //白平衡
    control_g.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get White Balance error\n");
    }else{
        printf("White Balance:%d\n",control_g.value);
    }
    //色调
    control_g.id =V4L2_CID_HUE ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get HUE error\n");
    }else{
        printf("HUE:%d\n",control_g.value);
    }
    //逆光补偿
    control_g.id =V4L2_CID_BACKLIGHT_COMPENSATION ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get BACKLIGHT_COMPENSATION error\n");
    }else{
        printf("BACKLIGHT_COMPENSATION:%d\n",control_g.value);
    }
    //清晰度
    control_g.id =V4L2_CID_SHARPNESS ;
    if( xioctl(mVideofd, VIDIOC_G_CTRL, &control_g) < 0){
        printf("Get SHARPNESS error\n");
    }else{
        printf("SHARPNESS:%d\n",control_g.value);
    }

    return 0;

}



void UsbCaptureWithThread::run() {
    while(true){
        if(mIsChangeFormat){
            //更改图像分辨率
            closeStream();
            close(mVideofd);
            if(init()==0){
                mIsChangeFormat= false;
            }
        }
        //获取图片
        if(mIsCapture){
            GetImgRaw();
        } else{
            usleep(30000);//30ms
        }
    }
}

void UsbCaptureWithThread::setCaptureState(bool isCapture) {
    mIsCapture=isCapture;
}
