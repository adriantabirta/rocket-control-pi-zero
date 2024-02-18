#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/fb.h>

void displayImage(std::ifstream imageFile) {
    const char* framebufferDevice = "/dev/fb0";

    // Open framebuffer device
    int fbfd = open(framebufferDevice, O_RDWR);
    if (fbfd == -1) {
        std::cerr << "Error opening framebuffer device" << std::endl;
        return 1;
    }

    // Get fixed screen information
    struct fb_fix_screeninfo finfo;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
        std::cerr << "Error reading fixed information" << std::endl;
        close(fbfd);
        return 1;
    }

    // Get variable screen information
    struct fb_var_screeninfo vinfo;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
        std::cerr << "Error reading variable information" << std::endl;
        close(fbfd);
        return 1;
    }

    // Open the image file
    if (!imageFile) {
        std::cerr << "Error opening image file" << std::endl;
        close(fbfd);
        return;
    }

    // Calculate image size
    unsigned int imageSize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
    char* imageData = new char[imageSize];

    // Read image data from file
    imageFile.read(imageData, imageSize);

    // Write image data to framebuffer
    lseek(fbfd, 0, SEEK_SET);
    write(fbfd, imageData, imageSize);

    // Clean up
    delete[] imageData;
    imageFile.close();
    close(fbfd);
}
