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
/// @file calibrate.cc
/// @author Billy Okal <b.okal@jacobs-university.de>
/// ---------------------------------------------------


#include <calibrate.h>

using namespace std;
namespace po = boost::program_options;


Calibrate::Calibrate(bool interact, string imagepath, int xsize, int ysize, bool visual)
    : interactive_(interact), images_path_(imagepath), size_x_(xsize), size_y_(ysize), visualize_(visual)
{
    image_queue_.clear();
    cout << "[INFO] Intializing ..." << endl;
}
Calibrate::~Calibrate()
{
    image_queue_.clear();
    cout << "[INFO] ... Done." << endl;
}


/// ============================================================
/// prepare images (use only jpeg images)
/// ============================================================
void Calibrate::prepareImages()
{
    cout << "[INFO] Preparing images ..." << endl;
    int image_count = 0;

    /// -----------------------------------
    /// acquire the images from files
    /// -----------------------------------
    if (!interactive_)
    {
        boost::progress_timer t( std::clog );

        fs::path full_path( fs::initial_path<fs::path>() );
        full_path = fs::system_complete( fs::path( images_path_ ) );

        if (!fs::exists(full_path))
        {
            cout << "[ERROR] Image path Not found: " << full_path.filename() << endl;
            exit(-1);
        }

        if (fs::is_directory( full_path ))
        {
            fs::directory_iterator end_iter;
            cv::Mat frame;

            for ( fs::directory_iterator dir_itr( full_path );
                  dir_itr != end_iter;
                  ++dir_itr )
            {
                try
                {
                    /// skip all subdirectories
                    if ( fs::is_directory( dir_itr->status() ) )
                    {
                        continue;
                    }
                    else if ( fs::is_regular_file( dir_itr->status() ) )
                    {
                        /// check if it is indeed jpeg a  file, then read or skip
                        if ( dir_itr->path().extension() == ".jpg" ||
                             dir_itr->path().extension() == ".jpeg" )
                        {
                            cout << "Reading in: "<< dir_itr->path().filename() << endl;
                            /// read in the image
                            frame = cv::imread( dir_itr->path().string() );
                            image_queue_.push_back(frame);
                            image_count++;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else /// skip all irregular files
                    {
                        continue;
                    }

                }
                catch ( const std::exception & ex )
                {
                    std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
                }
            }
        }
        else
        {
            cout << "[WARN] You specified path to only one image. More images are needed for calibration. Aborting" << endl;
            exit(-1);
        }
    }
    /// -----------------------------------
    /// acquire the images from webcam
    /// -----------------------------------
    else
    {
        cv::VideoCapture camera(-1);
        if (!camera.isOpened())
        {
            cout << "[ERROR] Cannot initialize the webcam, cannot continue." << endl;
            exit(-1);
        }

        cv::namedWindow(IMAGE_GRABBER_NAME, CV_WINDOW_AUTOSIZE);
        cout << "[INFO] Load images for calibration from webcam via the following" << endl;
        cout << "\t Press {Spacebar} to grab image" << endl;
        cout << "\t Press {d} to discard the last grabbed image" << endl;
        cout << "\t Press {Esc} to stop grabbing images" << endl;

        cv::Mat frame;
        while(1)
        {
            camera >> frame;

            cv::imshow(IMAGE_GRABBER_NAME, frame);
            char key = cv::waitKey(10);
            if (key == ACQUIRE_IMAGE)
            {
                image_queue_.push_back(frame);
                cout << "No of images grabbed: " << image_queue_.size() << endl;
                image_count++;
            }
            else if (key == DELETE_IMAGE)
            {
                image_queue_.pop_back();
                image_count--;
                cout << "No of images grabbed: " << image_queue_.size() << endl;
            }
            else if (key == STOP_GRABBING)
            {
                break;
            }
        }

    }
}



/// ============================================================
/// Do the calibration
/// ============================================================
void Calibrate::doCalibration()
{
    cout << "[INFO] Performing calibration ..." << endl;
    cv::namedWindow("Calibration", CV_WINDOW_AUTOSIZE);

    cv::Size board_size(size_x_,size_y_);
    cout << "X=" << size_x_ << ", Y=" << size_y_ << endl;
    float sqsize = 20;  // size of each square in mm

    /// compute the object points
    /// make Z equal to zero since we use a planar pattern
    std::vector<std::vector<cv::Point3f> > object_points(1);
    for( int i = 0; i < board_size.height; ++i )
        for( int j = 0; j < board_size.width; ++j )
            object_points[0].push_back(cv::Point3f(float( j*sqsize ), float( i*sqsize ), 0));

    object_points.resize(image_queue_.size(), object_points[0]);


    /// compute the image points
    std::vector<std::vector<cv::Point2f> > image_points;
    std::vector<cv::Point2f> corners;
    cv::Mat gray_image;

    std::vector<cv::Mat>::iterator it = image_queue_.begin();

    for (; it != image_queue_.end(); it++)
    {
        cv::cvtColor((*it), gray_image, CV_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray_image, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK);

        if (found)
        {
            image_points.push_back(corners);

            if (visualize_)
            {
                cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                cv::drawChessboardCorners((*it), board_size, cv::Mat(corners), found);
                cv::imshow("Calibration", (*it));
                if (cv::waitKey(0) == 27)
                    continue;
            }
        }
    }

    /// Prepare Matrices for calibration parameters
    cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;


    /// Do the actual calibration and save the results
    double reproj_error = calibrateCamera(object_points, image_points, image_queue_[0].size(), cam_matrix, dist_coeffs, rvecs, tvecs, CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cv::FileStorage cfs("calibration_data.yml", cv::FileStorage::WRITE);
    time_t rawtime; time(&rawtime);
    cfs << "calibration_date" << asctime(localtime(&rawtime));
    cfs << "camera_matrix" << cam_matrix;
    cfs << "distortion_coefficients" << dist_coeffs;
    cfs.release();

    /// undistort the first image and save it. Using c++ version of cvUndistort2
    cv::Mat undistorted;
    cv::undistort(image_queue_[0], undistorted, cam_matrix, dist_coeffs);
    cv::imwrite("undistorted.jpg", undistorted);
}





/// ============================================================
/// Main
/// ============================================================
int main(int argc, const char *argv[])
{
    /// --------------------------------------------
    /// set up program options
    /// --------------------------------------------
    po::options_description calib_options("Usage: calibrate <options> where options are: ");
    calib_options.add_options()
            ("help,?", "Show this help message")
            ("visualize,v", "Visialize the calibration steps")
            ("webcam,c", "Use the default webcam to acquire calibration images")
            ("images_path,p", po::value<std::string>(), "Path to the calibration images (If using pre-existing images)")
            ("width,w", po::value<int>(), "Size (no. of corners) of checkerboard in the shorter direction")
            ("height,h", po::value<int>(), "Size (no. of corners) of checkerboard in the longer direction")
            ;

    po::variables_map vmap;
    po::store(po::parse_command_line(argc, argv, calib_options), vmap);
    po::notify(vmap);



    /// --------------------------------------------
    /// Load and parse the program options
    /// --------------------------------------------

    std::string path_to_images = "";
    bool interactive = false;
    bool visualize = false;
    int xsize = 0; int ysize = 0;

    if (vmap.count("visualize"))
    {
        cout << "Visualizing results" << endl;
        visualize = true;
    }


    if (vmap.count("help"))
    {
        cout << calib_options << endl;
        exit(-1);
    }
    else if ( vmap.count("images_path") && vmap.count("width") && vmap.count("height") )
    {
        path_to_images += (vmap["images_path"].as<std::string>());
        xsize = vmap["width"].as<int>();
        ysize = vmap["height"].as<int>();
    }
    else if (vmap.count("webcam") && vmap.count("width") && vmap.count("height") )
    {
        cout << "Using webcam" << endl;
        interactive = true;
        xsize = vmap["width"].as<int>();
        ysize = vmap["height"].as<int>();
    }
    else
    {
        cout << calib_options << endl;
        exit(-1);
    }


    /// --------------------------------------------
    /// Initialize and start calibration
    /// --------------------------------------------

    Calibrate calib(interactive, path_to_images, xsize, ysize, visualize);
    calib.prepareImages();
    calib.doCalibration();

    return EXIT_SUCCESS;
}
