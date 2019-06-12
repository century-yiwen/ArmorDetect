#pragma once

#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int get_fd();
int Init_Uart();
void UART_SendData(int fd, unsigned int X, unsigned int Y, int Yes);
char UART_GetData(int fd);
void Sendata(int x,int y);
#endif
