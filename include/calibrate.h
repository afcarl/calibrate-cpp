/// ---------------------------------------------------
/// (C) Jacobs University Bremen gGmbH 2012
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// * Redistributions of source code must retain the above copyright
///   notice, this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright
///   notice, this list of conditions and the following disclaimer in the
///   documentation and/or other materials provided with the distribution.
/// * Neither the name of the author nor the
///   names of its contributors may be used to endorse or promote products
///   derived from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY the author ''AS IS'' AND ANY
/// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL the author BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/// @file calibrate.h
/// @author Billy Okal <b.okal@jacobs-university.de>
/// ---------------------------------------------------

#ifndef CALIBRATE_H_
#define CALIBRATE_H_


#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/progress.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/opencv.hpp>


#define IMAGE_GRABBER_NAME "Calib Image Grabber"
#define ACQUIRE_IMAGE 32
#define DELETE_IMAGE 100
#define STOP_GRABBING 27


namespace fs = boost::filesystem;


/// @brief Calibrate is a class for performing camera calibration
/// Images are acquired via webcam or from a file
class Calibrate
{
public:
    Calibrate(bool interact, std::string imagepath, int xsize, int ysize, bool visual);
    virtual ~Calibrate();

    void doCalibration();
    void prepareImages();

private:
    bool interactive_;
    bool visualize_;
    int size_x_;
    int size_y_;
    std::string images_path_;

    std::vector<cv::Mat> image_queue_;

};



#endif
