#include "tools.h"


#define _USE_MATH_DEFINES
#define SHOW_UNDISTORTION 0

FeatureTracker trackerData[NUM_OF_CAM];
Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<pair<cv::Mat, double>> img0_buf;
queue<pair<cv::Mat, double>> img1_buf;

std::mutex m_buf;

// void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
void img0_callback(const cv::Mat &img_msg, const double &t)
{
    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img0_buf.push(tmp);
    m_buf.unlock();
}

void img1_callback(const cv::Mat &img_msg, const double &t)
{
    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img1_buf.push(tmp);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     cv_bridge::CvImageConstPtr ptr;
//     if (img_msg->encoding == "8UC1")
//     {
//         sensor_msgs::Image img;
//         img.header = img_msg->header;
//         img.height = img_msg->height;
//         img.width = img_msg->width;
//         img.is_bigendian = img_msg->is_bigendian;
//         img.step = img_msg->step;
//         img.data = img_msg->data;
//         img.encoding = "mono8";
//         ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//     }
//     else
//         ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

//     cv::Mat img = ptr->image.clone();
//     return img;
// }

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                // double time0 = img0_buf.front()->header.stamp.toSec();
                // double time1 = img1_buf.front()->header.stamp.toSec();
                double time0 = img0_buf.front().second;
                double time1 = img1_buf.front().second;
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    // time = img0_buf.front()->header.stamp.toSec();
                    time = img0_buf.front().second;
                    // header = img0_buf.front()->header;
                    // image0 = getImageFromMsg(img0_buf.front());
                    image0 = img0_buf.front().first;
                    img0_buf.pop();
                    // image1 = getImageFromMsg(img1_buf.front());
                    image1 = img1_buf.front().first;
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                // time = img0_buf.front()->header.stamp.toSec();
                time = img0_buf.front().second;
                // header = img0_buf.front()->header;
                // image = getImageFromMsg(img0_buf.front());
                image = img0_buf.front().first;
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
        vector<string> &strImagesFileNames, vector<double> &timeStamps)
{
    ifstream fTimes;
    fTimes.open(strTimesStampsPath.c_str());
    timeStamps.reserve(5000); //reserve vector space
    strImagesFileNames.reserve(5000); 
    while(!fTimes.eof())
    {
    string s;
    getline(fTimes,s);
    if(!s.empty())
    {
        stringstream ss;
        ss << s;
        strImagesFileNames.push_back(strImagePath + "/" + ss.str() + ".png");
        double t;
        ss >> t;
        timeStamps.push_back(t/1e9);
    }
    }
}
/******************* load image end ***********************/

/******************* load IMU begin ***********************/

void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp)
{

    while(!fImus.eof())
    {
    string s;
    getline(fImus,s);
    if(!s.empty())
    {
       char c = s.at(0);
       if(c<'0' || c>'9')      //remove first line in data.csv
               continue;       
        stringstream ss;
        ss << s;
        double tmpd;
        int cnt=0;
        double data[7];
        while(ss >> tmpd)
        {
        data[cnt] = tmpd;
        cnt++;
        if(cnt ==7)
          break;
        if(ss.peek() == ',' || ss.peek() == ' ')
          ss.ignore();
        }
        data[0] *=1e-9; //convert to second unit
        sensor_msgs::ImuPtr imudata(new sensor_msgs::Imu);
        imudata->angular_velocity.x = data[1];
        imudata->angular_velocity.y = data[2];
        imudata->angular_velocity.z = data[3];
        imudata->linear_acceleration.x = data[4];
        imudata->linear_acceleration.y = data[5];
        imudata->linear_acceleration.z = data[6];
        uint32_t  sec = data[0];
        uint32_t nsec = (data[0]-sec)*1e9;
        nsec = (nsec/1000)*1000+500;
        imudata->header.stamp = ros::Time(sec,nsec);
        imu_callback(imudata);
        if (imudata->header.stamp > imageTimestamp)       //load all imu data produced in interval time between two consecutive frams 
          break;
    }
    }
}

int main(int argc, char **argv)
{
  /******************* load image begin ***********************/
    if(argc != 5)
    {
    cerr << endl << "Usage: ./vins_estimator path_to_setting_file path_to_image_folder path_to_times_file path_to_imu_data_file" <<endl;
    return 1;
    }
    
    //imu data file 
    ifstream fImus;
    fImus.open(argv[4]);
    
    cv::Mat image;
    int ni;//num image
    
    //read parameters section
    readParameters(argv[1]);
    
    estimator.setParameter();
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]); //add
    
    vector<string> vStrImagesFileNames;
    vector<double> vTimeStamps;
    LoadImages(string(argv[2]),string(argv[3]),vStrImagesFileNames,vTimeStamps);
    
    int imageNum = vStrImagesFileNames.size();
    
    if(imageNum<=0)
    {
    cerr << "ERROR: Failed to load images" << endl;
    return 1;
    }
    
    std::thread measurement_process{sync_process};
    
     measurement_process.detach();
   
    for(ni=0; ni<imageNum; ni++)
    {
      
      double  tframe = vTimeStamps[ni];   //timestamp
      uint32_t  sec = tframe;
      uint32_t nsec = (tframe-sec)*1e9;
      nsec = (nsec/1000)*1000+500;
      ros::Time image_timestamp = ros::Time(sec, nsec);
       // read imu data
      LoadImus(fImus,image_timestamp);
       
    //read image from file
      image = cv::imread(vStrImagesFileNames[ni],CV_LOAD_IMAGE_UNCHANGED);
      
      if(image.empty())
      {
      cerr << endl << "Failed to load image: " << vStrImagesFileNames[ni] <<endl;
      return 1;
      }
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      img0_callback(image, image_timestamp.toSec());
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double timeSpent =std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
      
      //wait to load the next frame image
      double T=0;
      if(ni < imageNum-1)
    T = vTimeStamps[ni+1]-tframe; //interval time between two consecutive frames,unit:second
      else if(ni>0)    //lastest frame
    T = tframe-vTimeStamps[ni-1];
      
      if(timeSpent < T)
    usleep((T-timeSpent)*1e6); //sec->us:1e6
      else
    cerr << endl << "process image speed too slow, larger than interval time between two consecutive frames" << endl;
      
    }

    return 0;
}