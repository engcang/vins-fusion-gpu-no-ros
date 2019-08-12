#ifndef CAMERA_MODELS_H
#define CAMERA_MODELS_H



//// Spline.h
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <exception>

//// gpl.h, EigenQuaternion, EigenUtils
#include <algorithm>
#include <cmath>
#include "ceres/local_parameterization.h"

////calib, camera
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdint.h>
#include "ceres/rotation.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream> //// error occured...?

// Spline.h
namespace ublas = boost::numeric::ublas;

class Spline : private std::vector<std::pair<double, double> >
{
public:
  //The boundary conditions available
  enum BC_type {
    FIXED_1ST_DERIV_BC,
    FIXED_2ND_DERIV_BC,
    PARABOLIC_RUNOUT_BC
  };

  enum Spline_type {
    LINEAR,
    CUBIC
  };

  //Constructor takes the boundary conditions as arguments, this
  //sets the first derivative (gradient) at the lower and upper
  //end points
  Spline():
    _valid(false),
    _BCLow(FIXED_2ND_DERIV_BC), _BCHigh(FIXED_2ND_DERIV_BC),
    _BCLowVal(0), _BCHighVal(0),
    _type(CUBIC)
  {}

  typedef std::vector<std::pair<double, double> > base;
  typedef base::const_iterator const_iterator;

  //Standard STL read-only container stuff
  const_iterator begin() const { return base::begin(); }
  const_iterator end() const { return base::end(); }
  void clear() { _valid = false; base::clear(); _data.clear(); }
  size_t size() const { return base::size(); }
  size_t max_size() const { return base::max_size(); }
  size_t capacity() const { return base::capacity(); }
  bool empty() const { return base::empty(); }

  //Add a point to the spline, and invalidate it so its
  //recalculated on the next access
  inline void addPoint(double x, double y)
  {
    _valid = false;
    base::push_back(std::pair<double, double>(x,y));
  }

  //Reset the boundary conditions
  inline void setLowBC(BC_type BC, double val = 0)
  { _BCLow = BC; _BCLowVal = val; _valid = false; }

  inline void setHighBC(BC_type BC, double val = 0)
  { _BCHigh = BC; _BCHighVal = val; _valid = false; }

  void setType(Spline_type type) { _type = type; _valid = false; }

  //Check if the spline has been calculated, then generate the
  //spline interpolated value
  double operator()(double xval)
  {
    if (!_valid) generate();

    //Special cases when we're outside the range of the spline points
    if (xval <= x(0)) return lowCalc(xval);
    if (xval >= x(size()-1)) return highCalc(xval);

    //Check all intervals except the last one
    for (std::vector<SplineData>::const_iterator iPtr = _data.begin();
         iPtr != _data.end()-1; ++iPtr)
        if ((xval >= iPtr->x) && (xval <= (iPtr+1)->x))
          return splineCalc(iPtr, xval);

    return splineCalc(_data.end() - 1, xval);
  }

private:

  ///////PRIVATE DATA MEMBERS
  struct SplineData { double x,a,b,c,d; };
  //vector of calculated spline data
  std::vector<SplineData> _data;
  //Second derivative at each point
  ublas::vector<double> _ddy;
  //Tracks whether the spline parameters have been calculated for
  //the current set of points
  bool _valid;
  //The boundary conditions
  BC_type _BCLow, _BCHigh;
  //The values of the boundary conditions
  double _BCLowVal, _BCHighVal;

  Spline_type _type;

  ///////PRIVATE FUNCTIONS
  //Function to calculate the value of a given spline at a point xval
  inline double splineCalc(std::vector<SplineData>::const_iterator i, double xval)
  {
    const double lx = xval - i->x;
    return ((i->a * lx + i->b) * lx + i->c) * lx + i->d;
  }

  inline double lowCalc(double xval)
  {
    const double lx = xval - x(0);

    if (_type == LINEAR)
      return lx * _BCHighVal + y(0);

    const double firstDeriv = (y(1) - y(0)) / h(0) - 2 * h(0) * (_data[0].b + 2 * _data[1].b) / 6;

    switch(_BCLow)
      {
      case FIXED_1ST_DERIV_BC:
        return lx * _BCLowVal + y(0);
      case FIXED_2ND_DERIV_BC:
          return lx * lx * _BCLowVal + firstDeriv * lx + y(0);
      case PARABOLIC_RUNOUT_BC:
        return lx * lx * _ddy[0] + lx * firstDeriv  + y(0);
      }
    throw std::runtime_error("Unknown BC");
  }

  inline double highCalc(double xval)
  {
    const double lx = xval - x(size() - 1);

    if (_type == LINEAR)
      return lx * _BCHighVal + y(size() - 1);

    const double firstDeriv = 2 * h(size() - 2) * (_ddy[size() - 2] + 2 * _ddy[size() - 1]) / 6 + (y(size() - 1) - y(size() - 2)) / h(size() - 2);

    switch(_BCHigh)
      {
      case FIXED_1ST_DERIV_BC:
        return lx * _BCHighVal + y(size() - 1);
      case FIXED_2ND_DERIV_BC:
        return lx * lx * _BCHighVal + firstDeriv * lx + y(size() - 1);
      case PARABOLIC_RUNOUT_BC:
        return lx * lx * _ddy[size()-1] + lx * firstDeriv  + y(size() - 1);
      }
    throw std::runtime_error("Unknown BC");
  }

  //These just provide access to the point data in a clean way
  inline double x(size_t i) const { return operator[](i).first; }
  inline double y(size_t i) const { return operator[](i).second; }
  inline double h(size_t i) const { return x(i+1) - x(i); }

  //Invert a arbitrary matrix using the boost ublas library
  template<class T>
  bool InvertMatrix(ublas::matrix<T> A,
        ublas::matrix<T>& inverse)
  {
    using namespace ublas;

    // create a permutation matrix for the LU-factorization
    permutation_matrix<std::size_t> pm(A.size1());

    // perform LU-factorization
    int res = lu_factorize(A,pm);
        if( res != 0 ) return false;

    // create identity matrix of "inverse"
    inverse.assign(ublas::identity_matrix<T>(A.size1()));

    // backsubstitute to get the inverse
    lu_substitute(A, pm, inverse);

    return true;
  }

  //This function will recalculate the spline parameters and store
  //them in _data, ready for spline interpolation
  void generate()
  {
    if (size() < 2)
      throw std::runtime_error("Spline requires at least 2 points");

    //If any spline points are at the same x location, we have to
    //just slightly seperate them
    {
      bool testPassed(false);
      while (!testPassed)
        {
          testPassed = true;
          std::sort(base::begin(), base::end());

          for (base::iterator iPtr = base::begin(); iPtr != base::end() - 1; ++iPtr)
        if (iPtr->first == (iPtr+1)->first)
          {
            if ((iPtr+1)->first != 0)
              (iPtr+1)->first += (iPtr+1)->first
            * std::numeric_limits<double>::epsilon() * 10;
            else
              (iPtr+1)->first = std::numeric_limits<double>::epsilon() * 10;
            testPassed = false;
            break;
          }
        }
    }

    const size_t e = size() - 1;

    switch (_type)
      {
      case LINEAR:
        {
          _data.resize(e);
          for (size_t i(0); i < e; ++i)
        {
          _data[i].x = x(i);
          _data[i].a = 0;
          _data[i].b = 0;
          _data[i].c = (y(i+1) - y(i)) / (x(i+1) - x(i));
          _data[i].d = y(i);
        }
          break;
        }
      case CUBIC:
        {
          ublas::matrix<double> A(size(), size());
          for (size_t yv(0); yv <= e; ++yv)
        for (size_t xv(0); xv <= e; ++xv)
          A(xv,yv) = 0;

          for (size_t i(1); i < e; ++i)
        {
          A(i-1,i) = h(i-1);
          A(i,i) = 2 * (h(i-1) + h(i));
          A(i+1,i) = h(i);
        }

          ublas::vector<double> C(size());
          for (size_t xv(0); xv <= e; ++xv)
        C(xv) = 0;

          for (size_t i(1); i < e; ++i)
        C(i) = 6 *
          ((y(i+1) - y(i)) / h(i)
           - (y(i) - y(i-1)) / h(i-1));

          //Boundary conditions
          switch(_BCLow)
        {
        case FIXED_1ST_DERIV_BC:
          C(0) = 6 * ((y(1) - y(0)) / h(0) - _BCLowVal);
          A(0,0) = 2 * h(0);
          A(1,0) = h(0);
          break;
        case FIXED_2ND_DERIV_BC:
          C(0) = _BCLowVal;
          A(0,0) = 1;
          break;
        case PARABOLIC_RUNOUT_BC:
          C(0) = 0; A(0,0) = 1; A(1,0) = -1;
          break;
        }

          switch(_BCHigh)
        {
        case FIXED_1ST_DERIV_BC:
          C(e) = 6 * (_BCHighVal - (y(e) - y(e-1)) / h(e-1));
          A(e,e) = 2 * h(e - 1);
          A(e-1,e) = h(e - 1);
          break;
        case FIXED_2ND_DERIV_BC:
          C(e) = _BCHighVal;
          A(e,e) = 1;
          break;
        case PARABOLIC_RUNOUT_BC:
          C(e) = 0; A(e,e) = 1; A(e-1,e) = -1;
          break;
        }

          ublas::matrix<double> AInv(size(), size());
          InvertMatrix(A,AInv);

          _ddy = ublas::prod(C, AInv);

          _data.resize(size()-1);
          for (size_t i(0); i < e; ++i)
        {
          _data[i].x = x(i);
          _data[i].a = (_ddy(i+1) - _ddy(i)) / (6 * h(i));
          _data[i].b = _ddy(i) / 2;
          _data[i].c = (y(i+1) - y(i)) / h(i) - _ddy(i+1) * h(i) / 6 - _ddy(i) * h(i) / 3;
          _data[i].d = y(i);
        }
        }
      }
    _valid = true;
  }
};
// Spline.h

namespace ceres//CostFunctionFactory.h
{
    class CostFunction;
}

namespace camodocal
{

//Chessboard.h
class ChessboardCorner;
typedef boost::shared_ptr<ChessboardCorner> ChessboardCornerPtr;
class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class Chessboard
{
public:
    Chessboard(cv::Size boardSize, cv::Mat& image);

    void findCorners(bool useOpenCV = false);
    const std::vector<cv::Point2f>& getCorners(void) const;
    bool cornersFound(void) const;

    const cv::Mat& getImage(void) const;
    const cv::Mat& getSketch(void) const;

private:
    bool findChessboardCorners(const cv::Mat& image,
                               const cv::Size& patternSize,
                               std::vector<cv::Point2f>& corners,
                               int flags, bool useOpenCV);

    bool findChessboardCornersImproved(const cv::Mat& image,
                                       const cv::Size& patternSize,
                                       std::vector<cv::Point2f>& corners,
                                       int flags);

    void cleanFoundConnectedQuads(std::vector<ChessboardQuadPtr>& quadGroup, cv::Size patternSize);

    void findConnectedQuads(std::vector<ChessboardQuadPtr>& quads,
                            std::vector<ChessboardQuadPtr>& group,
                            int group_idx, int dilation);

//    int checkQuadGroup(std::vector<ChessboardQuadPtr>& quadGroup,
//                       std::vector<ChessboardCornerPtr>& outCorners,
//                       cv::Size patternSize);

    void labelQuadGroup(std::vector<ChessboardQuadPtr>& quad_group,
                        cv::Size patternSize, bool firstRun);

    void findQuadNeighbors(std::vector<ChessboardQuadPtr>& quads, int dilation);

    int augmentBestRun(std::vector<ChessboardQuadPtr>& candidateQuads, int candidateDilation,
                       std::vector<ChessboardQuadPtr>& existingQuads, int existingDilation);

    void generateQuads(std::vector<ChessboardQuadPtr>& quads,
                       cv::Mat& image, int flags,
                       int dilation, bool firstRun);

    bool checkQuadGroup(std::vector<ChessboardQuadPtr>& quads,
                        std::vector<ChessboardCornerPtr>& corners,
                        cv::Size patternSize);

    void getQuadrangleHypotheses(const std::vector< std::vector<cv::Point> >& contours,
                                 std::vector< std::pair<float, int> >& quads,
                                 int classId) const;

    bool checkChessboard(const cv::Mat& image, cv::Size patternSize) const;

    bool checkBoardMonotony(std::vector<ChessboardCornerPtr>& corners,
                            cv::Size patternSize);

    bool matchCorners(ChessboardQuadPtr& quad1, int corner1,
                      ChessboardQuadPtr& quad2, int corner2) const;

    cv::Mat mImage;
    cv::Mat mSketch;
    std::vector<cv::Point2f> mCorners;
    cv::Size mBoardSize;
    bool mCornersFound;
};
//Chessboard.h
//ChessboardCorner.h
class ChessboardCorner;
typedef boost::shared_ptr<ChessboardCorner> ChessboardCornerPtr;

class ChessboardCorner
{
public:
    ChessboardCorner() : row(0), column(0), needsNeighbor(true), count(0) {}

    float meanDist(int &n) const
    {
        float sum = 0;
        n = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (neighbors[i].get())
            {
                float dx = neighbors[i]->pt.x - pt.x;
                float dy = neighbors[i]->pt.y - pt.y;
                sum += sqrt(dx*dx + dy*dy);
                n++;
            }
        }
        return sum / std::max(n, 1);
    }

    cv::Point2f pt;                     // X and y coordinates
    int row;                            // Row and column of the corner
    int column;                         // in the found pattern
    bool needsNeighbor;                 // Does the corner require a neighbor?
    int count;                          // number of corner neighbors
    ChessboardCornerPtr neighbors[4];   // pointer to all corner neighbors
};
//ChessboardCorner.h
//ChessboardQuad.h
class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class ChessboardQuad
{
public:
    ChessboardQuad() : count(0), group_idx(-1), edge_len(FLT_MAX), labeled(false) {}

    int count;                         // Number of quad neighbors
    int group_idx;                     // Quad group ID
    float edge_len;                    // Smallest side length^2
    ChessboardCornerPtr corners[4];    // Coordinates of quad corners
    ChessboardQuadPtr neighbors[4];    // Pointers of quad neighbors
    bool labeled;                      // Has this corner been labeled?
};
//ChessboardQuad.h


//Transform.h
class Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Transform();
    Transform(const Eigen::Matrix4d& H);

    Eigen::Quaterniond& rotation(void);
    const Eigen::Quaterniond& rotation(void) const;
    double* rotationData(void);
    const double* const rotationData(void) const;

    Eigen::Vector3d& translation(void);
    const Eigen::Vector3d& translation(void) const;
    double* translationData(void);
    const double* const translationData(void) const;

    Eigen::Matrix4d toMatrix(void) const;

private:
    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;
};
//Transform.h

//gpl.h
template<class T>
const T clamp(const T& v, const T& a, const T& b)
{
    return std::min(b, std::max(a, v));
}

double hypot3(double x, double y, double z);
float hypot3f(float x, float y, float z);

template<class T>
const T normalizeTheta(const T& theta)
{
    T normTheta = theta;

    while (normTheta < - M_PI)
    {
        normTheta += 2.0 * M_PI;
    }
    while (normTheta > M_PI)
    {
        normTheta -= 2.0 * M_PI;
    }

    return normTheta;
}

double d2r(double deg);
float d2r(float deg);
double r2d(double rad);
float r2d(float rad);

double sinc(double theta);

template<class T>
const T square(const T& x)
{
    return x * x;
}

template<class T>
const T cube(const T& x)
{
    return x * x * x;
}

template<class T>
const T random(const T& a, const T& b)
{
    return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;
}

template<class T>
const T randomNormal(const T& sigma)
{
    T x1, x2, w;

    do
    {
        x1 = 2.0 * random(0.0, 1.0) - 1.0;
        x2 = 2.0 * random(0.0, 1.0) - 1.0;
        w = x1 * x1 + x2 * x2;
    }
    while (w >= 1.0 || w == 0.0);

    w = sqrt((-2.0 * log(w)) / w);

    return x1 * w * sigma;
}

unsigned long long timeInMicroseconds(void);

double timeInSeconds(void);

void colorDepthImage(cv::Mat& imgDepth,
                     cv::Mat& imgColoredDepth,
                     float minRange, float maxRange);

bool colormap(const std::string& name, unsigned char idx,
              float& r, float& g, float& b);

std::vector<cv::Point2i> bresLine(int x0, int y0, int x1, int y1);
std::vector<cv::Point2i> bresCircle(int x0, int y0, int r);

void fitCircle(const std::vector<cv::Point2d>& points,
               double& centerX, double& centerY, double& radius);

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2);

void LLtoUTM(double latitude, double longitude,
             double& utmNorthing, double& utmEasting,
             std::string& utmZone);
void UTMtoLL(double utmNorthing, double utmEasting,
             const std::string& utmZone,
             double& latitude, double& longitude);

long int timestampDiff(uint64_t t1, uint64_t t2);
//gpl.h

//EigenUtils.h
// Returns the 3D cross product skew symmetric matrix of a given 3D vector
template<typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1),
                                        vec(2), T(0), -vec(0),
                                        -vec(1), vec(0), T(0)).finished();
}

template<typename Derived>
typename Eigen::MatrixBase<Derived>::PlainObject sqrtm(const Eigen::MatrixBase<Derived>& A)
{
    Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> es(A);

    return es.operatorSqrt();
}

template<typename T>
Eigen::Matrix<T, 3, 3> AngleAxisToRotationMatrix(const Eigen::Matrix<T, 3, 1>& rvec)
{
    T angle = rvec.norm();
    if (angle == T(0))
    {
        return Eigen::Matrix<T, 3, 3>::Identity();
    }

    Eigen::Matrix<T, 3, 1> axis;
    axis = rvec.normalized();

    Eigen::Matrix<T, 3, 3> rmat;
    rmat = Eigen::AngleAxis<T>(angle, axis);

    return rmat;
}

template<typename T>
Eigen::Quaternion<T> AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec)
{
    Eigen::Matrix<T, 3, 3> rmat = AngleAxisToRotationMatrix<T>(rvec);

    return Eigen::Quaternion<T>(rmat);
}

template<typename T>
void AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec, T* q)
{
    Eigen::Quaternion<T> quat = AngleAxisToQuaternion<T>(rvec);

    q[0] = quat.x();
    q[1] = quat.y();
    q[2] = quat.z();
    q[3] = quat.w();
}

template<typename T>
Eigen::Matrix<T, 3, 1> RotationToAngleAxis(const Eigen::Matrix<T, 3, 3> & rmat)
{
    Eigen::AngleAxis<T> angleaxis; 
    angleaxis.fromRotationMatrix(rmat); 
    return angleaxis.angle() * angleaxis.axis(); 
    
}

template<typename T>
void QuaternionToAngleAxis(const T* const q, Eigen::Matrix<T, 3, 1>& rvec)
{
    Eigen::Quaternion<T> quat(q[3], q[0], q[1], q[2]);

    Eigen::Matrix<T, 3, 3> rmat = quat.toRotationMatrix();

    Eigen::AngleAxis<T> angleaxis;
    angleaxis.fromRotationMatrix(rmat);

    rvec = angleaxis.angle() * angleaxis.axis();
}

template<typename T>
Eigen::Matrix<T, 3, 3> QuaternionToRotation(const T* const q)
{
    T R[9];
    ceres::QuaternionToRotation(q, R);

    Eigen::Matrix<T, 3, 3> rmat;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rmat(i,j) = R[i * 3 + j];
        }
    }

    return rmat;
}

template<typename T>
void QuaternionToRotation(const T* const q, T* rot)
{
    ceres::QuaternionToRotation(q, rot);
}

template<typename T>
Eigen::Matrix<T,4,4> QuaternionMultMatLeft(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T,4,4>() << q.w(), -q.z(), q.y(), q.x(),
                                      q.z(), q.w(), -q.x(), q.y(),
                                      -q.y(), q.x(), q.w(), q.z(),
                                      -q.x(), -q.y(), -q.z(), q.w()).finished();
}

template<typename T>
Eigen::Matrix<T,4,4> QuaternionMultMatRight(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T,4,4>() << q.w(), q.z(), -q.y(), q.x(),
                                      -q.z(), q.w(), q.x(), q.y(),
                                      q.y(), -q.x(), q.w(), q.z(),
                                      -q.x(), -q.y(), -q.z(), q.w()).finished();
}

/// @param theta - rotation about screw axis
/// @param d - projection of tvec on the rotation axis
/// @param l - screw axis direction
/// @param m - screw axis moment
template<typename T>
void AngleAxisAndTranslationToScrew(const Eigen::Matrix<T, 3, 1>& rvec,
                                    const Eigen::Matrix<T, 3, 1>& tvec,
                                    T& theta, T& d,
                                    Eigen::Matrix<T, 3, 1>& l,
                                    Eigen::Matrix<T, 3, 1>& m)
{

    theta = rvec.norm();
    if (theta == 0)
    {
        l.setZero(); 
        m.setZero(); 
        std::cout << "Warning: Undefined screw! Returned 0. " << std::endl; 
    }

    l = rvec.normalized();

    Eigen::Matrix<T, 3, 1> t = tvec;

    d = t.transpose() * l;

    // point on screw axis - projection of origin on screw axis
    Eigen::Matrix<T, 3, 1> c;
    c = 0.5 * (t - d * l + (1.0 / tan(theta / 2.0) * l).cross(t));

    // c and hence the screw axis is not defined if theta is either 0 or M_PI
    m = c.cross(l);
}

template<typename T>
Eigen::Matrix<T, 3, 3> RPY2mat(T roll, T pitch, T yaw)
{
    Eigen::Matrix<T, 3, 3> m;

    T cr = cos(roll);
    T sr = sin(roll);
    T cp = cos(pitch);
    T sp = sin(pitch);
    T cy = cos(yaw);
    T sy = sin(yaw);

    m(0,0) = cy * cp;
    m(0,1) = cy * sp * sr - sy * cr;
    m(0,2) = cy * sp * cr + sy * sr;
    m(1,0) = sy * cp;
    m(1,1) = sy * sp * sr + cy * cr;
    m(1,2) = sy * sp * cr - cy * sr;
    m(2,0) = - sp;
    m(2,1) = cp * sr;
    m(2,2) = cp * cr;
    return m; 
}

template<typename T>
void mat2RPY(const Eigen::Matrix<T, 3, 3>& m, T& roll, T& pitch, T& yaw)
{
    roll = atan2(m(2,1), m(2,2));
    pitch = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
    yaw = atan2(m(1,0), m(0,0));
}

template<typename T>
Eigen::Matrix<T, 4, 4> homogeneousTransform(const Eigen::Matrix<T, 3, 3>& R, const Eigen::Matrix<T, 3, 1>& t)
{
    Eigen::Matrix<T, 4, 4> H;
    H.setIdentity();

    H.block(0,0,3,3) = R;
    H.block(0,3,3,1) = t;

    return H;
}

template<typename T>
Eigen::Matrix<T, 4, 4> poseWithCartesianTranslation(const T* const q, const T* const p)
{
    Eigen::Matrix<T, 4, 4> pose = Eigen::Matrix<T, 4, 4>::Identity();

    T rotation[9];
    ceres::QuaternionToRotation(q, rotation);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose(i,j) = rotation[i * 3 + j];
        }
    }

    pose(0,3) = p[0];
    pose(1,3) = p[1];
    pose(2,3) = p[2];

    return pose;
}

template<typename T>
Eigen::Matrix<T, 4, 4> poseWithSphericalTranslation(const T* const q, const T* const p, const T scale = T(1.0))
{
    Eigen::Matrix<T, 4, 4> pose = Eigen::Matrix<T, 4, 4>::Identity();

    T rotation[9];
    ceres::QuaternionToRotation(q, rotation);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose(i,j) = rotation[i * 3 + j];
        }
    }

    T theta = p[0];
    T phi = p[1];
    pose(0,3) = sin(theta) * cos(phi) * scale;
    pose(1,3) = sin(theta) * sin(phi) * scale;
    pose(2,3) = cos(theta) * scale;

    return pose;
}

// Returns the Sampson error of a given essential matrix and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 3, 3>& E,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    Eigen::Matrix<T, 3, 1> Ex1 = E * p1;
    Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * p2;

    T x2tEx1 = p2.dot(Ex1);

    // compute Sampson error
    T err = square(x2tEx1) / (square(Ex1(0,0)) + square(Ex1(1,0)) + square(Etx2(0,0)) + square(Etx2(1,0)));

    return err;
}

// Returns the Sampson error of a given rotation/translation and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 3, 3>& R,
               const Eigen::Matrix<T, 3, 1>& t,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    // construct essential matrix
    Eigen::Matrix<T, 3, 3> E = skew(t) * R;

    Eigen::Matrix<T, 3, 1> Ex1 = E * p1;
    Eigen::Matrix<T, 3, 1> Etx2 = E.transpose() * p2;

    T x2tEx1 = p2.dot(Ex1);

    // compute Sampson error
    T err = square(x2tEx1) / (square(Ex1(0,0)) + square(Ex1(1,0)) + square(Etx2(0,0)) + square(Etx2(1,0)));

    return err;
}

// Returns the Sampson error of a given rotation/translation and 2 image points
template<typename T>
T sampsonError(const Eigen::Matrix<T, 4, 4>& H,
               const Eigen::Matrix<T, 3, 1>& p1,
               const Eigen::Matrix<T, 3, 1>& p2)
{
    Eigen::Matrix<T, 3, 3> R = H.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> t = H.block(0, 3, 3, 1);

    return sampsonError(R, t, p1, p2);
}

template<typename T>
Eigen::Matrix<T, 3, 1>
transformPoint(const Eigen::Matrix<T, 4, 4>& H, const Eigen::Matrix<T, 3, 1>& P)
{
    Eigen::Matrix<T, 3, 1> P_trans = H.block(0, 0, 3, 3) * P + H.block(0, 3, 3, 1);

    return P_trans;
}

template<typename T>
Eigen::Matrix<T, 4, 4>
estimate3DRigidTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                         const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
{
    // compute centroids
    Eigen::Matrix<T, 3, 1> c1, c2;
    c1.setZero(); c2.setZero();

    for (size_t i = 0; i < points1.size(); ++i)
    {
        c1 += points1.at(i);
        c2 += points2.at(i);
    }

    c1 /= points1.size();
    c2 /= points1.size();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        X.col(i) = points1.at(i) - c1;
        Y.col(i) = points2.at(i) - c2;
    }

    Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

    Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<T, 3, 3> U = svd.matrixU();
    Eigen::Matrix<T, 3, 3> V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0.0)
    {
        V.col(2) *= -1.0;
    }

    Eigen::Matrix<T, 3, 3> R = V * U.transpose();
    Eigen::Matrix<T, 3, 1> t = c2 - R * c1;

    return homogeneousTransform(R, t);
}

template<typename T>
Eigen::Matrix<T, 4, 4>
estimate3DRigidSimilarityTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                                   const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
{
    // compute centroids
    Eigen::Matrix<T, 3, 1> c1, c2;
    c1.setZero(); c2.setZero();

    for (size_t i = 0; i < points1.size(); ++i)
    {
        c1 += points1.at(i);
        c2 += points2.at(i);
    }

    c1 /= points1.size();
    c2 /= points1.size();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        X.col(i) = points1.at(i) - c1;
        Y.col(i) = points2.at(i) - c2;
    }

    Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

    Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix<T, 3, 3> U = svd.matrixU();
    Eigen::Matrix<T, 3, 3> V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0.0)
    {
        V.col(2) *= -1.0;
    }

    Eigen::Matrix<T, 3, 3> R = V * U.transpose();

    std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > > rotatedPoints1(points1.size());
    for (size_t i = 0; i < points1.size(); ++i)
    {
        rotatedPoints1.at(i) = R * (points1.at(i) - c1);
    }

    double sum_ss = 0.0, sum_tt = 0.0;
    for (size_t i = 0; i < points1.size(); ++i)
    {
        sum_ss += (points1.at(i) - c1).squaredNorm();
        sum_tt += (points2.at(i) - c2).dot(rotatedPoints1.at(i));
    }

    double scale = sum_tt / sum_ss;

    Eigen::Matrix<T, 3, 3> sR = scale * R;
    Eigen::Matrix<T, 3, 1> t = c2 - sR * c1;

    return homogeneousTransform(sR, t);
}
//EigenUtils.h

//EigenQuaternionParameterization.h
class EigenQuaternionParameterization : public ceres::LocalParameterization
{
public:
    virtual ~EigenQuaternionParameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 4; }
    virtual int LocalSize() const { return 3; }

private:
    template<typename T>
    void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const;
};


template<typename T>
void
EigenQuaternionParameterization::EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const
{
    zw[0] = z[3] * w[0] + z[0] * w[3] + z[1] * w[2] - z[2] * w[1];
    zw[1] = z[3] * w[1] - z[0] * w[2] + z[1] * w[3] + z[2] * w[0];
    zw[2] = z[3] * w[2] + z[0] * w[1] - z[1] * w[0] + z[2] * w[3];
    zw[3] = z[3] * w[3] - z[0] * w[0] - z[1] * w[1] - z[2] * w[2];
}
//EigenQuaternionParameterization.h

//Camera.h
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE,
        SCARAMUZZA
    };

    class Parameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Parameters(ModelType modelType);

        Parameters(ModelType modelType, const std::string& cameraName,
                   int w, int h);

        ModelType& modelType(void);
        std::string& cameraName(void);
        int& imageWidth(void);
        int& imageHeight(void);

        ModelType modelType(void) const;
        const std::string& cameraName(void) const;
        int imageWidth(void) const;
        int imageHeight(void) const;

        int nIntrinsics(void) const;

        virtual bool readFromYamlFile(const std::string& filename) = 0;
        virtual void writeToYamlFile(const std::string& filename) const = 0;

    protected:
        ModelType m_modelType;
        int m_nIntrinsics;
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
    };

    virtual ModelType modelType(void) const = 0;
    virtual const std::string& cameraName(void) const = 0;
    virtual int imageWidth(void) const = 0;
    virtual int imageHeight(void) const = 0;

    virtual cv::Mat& mask(void);
    virtual const cv::Mat& mask(void) const;

    virtual void estimateIntrinsics(const cv::Size& boardSize,
                                    const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                    const std::vector< std::vector<cv::Point2f> >& imagePoints) = 0;
    virtual void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                                    const std::vector<cv::Point2f>& imagePoints,
                                    cv::Mat& rvec, cv::Mat& tvec) const;

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    // Lift points from the image plane to the projective space
    virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const = 0;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    //virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                          Eigen::Matrix<double,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;
    //%output p

    //virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const = 0;
    virtual cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                            float fx = -1.0f, float fy = -1.0f,
                                            cv::Size imageSize = cv::Size(0, 0),
                                            float cx = -1.0f, float cy = -1.0f,
                                            cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

    virtual int parameterCount(void) const = 0;

    virtual void readParameters(const std::vector<double>& parameters) = 0;
    virtual void writeParameters(std::vector<double>& parameters) const = 0;

    virtual void writeParametersToYamlFile(const std::string& filename) const = 0;

    virtual std::string parametersToString(void) const = 0;

    /**
     * \brief Calculates the reprojection distance between points
     *
     * \param P1 first 3D point coordinates
     * \param P2 second 3D point coordinates
     * \return euclidean distance in the plane
     */
    double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

    double reprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                             const std::vector< std::vector<cv::Point2f> >& imagePoints,
                             const std::vector<cv::Mat>& rvecs,
                             const std::vector<cv::Mat>& tvecs,
                             cv::OutputArray perViewErrors = cv::noArray()) const;

    double reprojectionError(const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& camera_q,
                             const Eigen::Vector3d& camera_t,
                             const Eigen::Vector2d& observed_p) const;

    void projectPoints(const std::vector<cv::Point3f>& objectPoints,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       std::vector<cv::Point2f>& imagePoints) const;
protected:
    cv::Mat m_mask;
};

typedef boost::shared_ptr<Camera> CameraPtr;
typedef boost::shared_ptr<const Camera> CameraConstPtr;

//Camera.h

//CostFunctionFactory.h
enum
{
    CAMERA_INTRINSICS =         1 << 0,
    CAMERA_POSE =               1 << 1,
    POINT_3D =                  1 << 2,
    ODOMETRY_INTRINSICS =       1 << 3,
    ODOMETRY_3D_POSE =          1 << 4,
    ODOMETRY_6D_POSE =          1 << 5,
    CAMERA_ODOMETRY_TRANSFORM = 1 << 6
};

class CostFunctionFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunctionFactory();

    static boost::shared_ptr<CostFunctionFactory> instance(void);

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p,
                                              int flags) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p,
                                              const Eigen::Matrix2d& sqrtPrecisionMat,
                                              int flags) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector2d& observed_p,
                                              int flags, bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector2d& observed_p,
                                              const Eigen::Matrix2d& sqrtPrecisionMat,
                                              int flags, bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& odo_pos,
                                              const Eigen::Vector3d& odo_att,
                                              const Eigen::Vector2d& observed_p,
                                              int flags, bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Quaterniond& cam_odo_q,
                                              const Eigen::Vector3d& cam_odo_t,
                                              const Eigen::Vector3d& odo_pos,
                                              const Eigen::Vector3d& odo_att,
                                              const Eigen::Vector2d& observed_p,
                                              int flags) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& cameraLeft,
                                              const CameraConstPtr& cameraRight,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p_left,
                                              const Eigen::Vector2d& observed_p_right) const;

private:
    static boost::shared_ptr<CostFunctionFactory> m_instance;
};
//CostFunctionFactory.h

//CameraFactory.h
class CameraFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};
//CameraFactory.h


//CataCamera.h
class CataCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double xi,
                   double k1, double k2, double p1, double p2,
                   double gamma1, double gamma2, double u0, double v0);

        double& xi(void);
        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& gamma1(void);
        double& gamma2(void);
        double& u0(void);
        double& v0(void);

        double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double gamma1(void) const;
        double gamma2(void) const;
        double u0(void) const;
        double v0(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_xi;
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_gamma1;
        double m_gamma2;
        double m_u0;
        double m_v0;
    };

    CataCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const std::string& cameraName,
               int imageWidth, int imageHeight,
               double xi, double k1, double k2, double p1, double p2,
               double gamma1, double gamma2, double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    CataCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                    Eigen::Matrix2d& J) const;

    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef boost::shared_ptr<CataCamera> CataCameraPtr;
typedef boost::shared_ptr<const CataCamera> CataCameraConstPtr;

template <typename T>
void
CataCamera::spaceToPlane(const T* const params,
                         const T* const q, const T* const t,
                         const Eigen::Matrix<T, 3, 1>& P,
                         Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0] = T(P(0));
    P_w[1] = T(P(1));
    P_w[2] = T(P(2));

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane
    T xi = params[0];
    T k1 = params[1];
    T k2 = params[2];
    T p1 = params[3];
    T p2 = params[4];
    T gamma1 = params[5];
    T gamma2 = params[6];
    T alpha = T(0); //cameraParams.alpha();
    T u0 = params[7];
    T v0 = params[8];

    // Transform to model plane
    T len = sqrt(P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2]);
    P_c[0] /= len;
    P_c[1] /= len;
    P_c[2] /= len;

    T u = P_c[0] / (P_c[2] + xi);
    T v = P_c[1] / (P_c[2] + xi);

    T rho_sqr = u * u + v * v;
    T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
    T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
    T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

    u = L * u + du;
    v = L * v + dv;
    p(0) = gamma1 * (u + alpha * v) + u0;
    p(1) = gamma2 * v + v0;
}
//CataCamera.h

//PinholeCamera.h
class PinholeCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double k1, double k2, double p1, double p2,
                   double fx, double fy, double cx, double cy);

        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& fx(void);
        double& fy(void);
        double& cx(void);
        double& cy(void);

        double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double fx(void) const;
        double fy(void) const;
        double cx(void) const;
        double cy(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
    };

    PinholeCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(const std::string& cameraName,
                  int imageWidth, int imageHeight,
                  double k1, double k2, double p1, double p2,
                  double fx, double fy, double cx, double cy);
    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                    Eigen::Matrix2d& J) const;

    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;

template <typename T>
void
PinholeCamera::spaceToPlane(const T* const params,
                            const T* const q, const T* const t,
                            const Eigen::Matrix<T, 3, 1>& P,
                            Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0] = T(P(0));
    P_w[1] = T(P(1));
    P_w[2] = T(P(2));

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane
    T k1 = params[0];
    T k2 = params[1];
    T p1 = params[2];
    T p2 = params[3];
    T fx = params[4];
    T fy = params[5];
    T alpha = T(0); //cameraParams.alpha();
    T cx = params[6];
    T cy = params[7];

    // Transform to model plane
    T u = P_c[0] / P_c[2];
    T v = P_c[1] / P_c[2];

    T rho_sqr = u * u + v * v;
    T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
    T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u);
    T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

    u = L * u + du;
    v = L * v + dv;
    p(0) = fx * (u + alpha * v) + cx;
    p(1) = fy * v + cy;
}
//PinholeCamera.h

//EquidistantCamera.h
class EquidistantCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();
        Parameters(const std::string& cameraName,
                   int w, int h,
                   double k2, double k3, double k4, double k5,
                   double mu, double mv,
                   double u0, double v0);

        double& k2(void);
        double& k3(void);
        double& k4(void);
        double& k5(void);
        double& mu(void);
        double& mv(void);
        double& u0(void);
        double& v0(void);

        double k2(void) const;
        double k3(void) const;
        double k4(void) const;
        double k5(void) const;
        double mu(void) const;
        double mv(void) const;
        double u0(void) const;
        double v0(void) const;

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        // projection
        double m_k2;
        double m_k3;
        double m_k4;
        double m_k5;

        double m_mu;
        double m_mv;
        double m_u0;
        double m_v0;
    };

    EquidistantCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    EquidistantCamera(const std::string& cameraName,
                      int imageWidth, int imageHeight,
                      double k2, double k3, double k4, double k5,
                      double mu, double mv,
                      double u0, double v0);
    /**
    * \brief Constructor from the projection model parameters
    */
    EquidistantCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
                      Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);

    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    template<typename T>
    static T r(T k2, T k3, T k4, T k5, T theta);


    void fitOddPoly(const std::vector<double>& x, const std::vector<double>& y,
                    int n, std::vector<double>& coeffs) const;

    void backprojectSymmetric(const Eigen::Vector2d& p_u,
                              double& theta, double& phi) const;

    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
};

typedef boost::shared_ptr<EquidistantCamera> EquidistantCameraPtr;
typedef boost::shared_ptr<const EquidistantCamera> EquidistantCameraConstPtr;

template<typename T>
T
EquidistantCamera::r(T k2, T k3, T k4, T k5, T theta)
{
    // k1 = 1
    return theta +
           k2 * theta * theta * theta +
           k3 * theta * theta * theta * theta * theta +
           k4 * theta * theta * theta * theta * theta * theta * theta +
           k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
}

template <typename T>
void
EquidistantCamera::spaceToPlane(const T* const params,
                                const T* const q, const T* const t,
                                const Eigen::Matrix<T, 3, 1>& P,
                                Eigen::Matrix<T, 2, 1>& p)
{
    T P_w[3];
    P_w[0] = T(P(0));
    P_w[1] = T(P(1));
    P_w[2] = T(P(2));

    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
    T q_ceres[4] = {q[3], q[0], q[1], q[2]};

    T P_c[3];
    ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    P_c[0] += t[0];
    P_c[1] += t[1];
    P_c[2] += t[2];

    // project 3D object point to the image plane;
    T k2 = params[0];
    T k3 = params[1];
    T k4 = params[2];
    T k5 = params[3];
    T mu = params[4];
    T mv = params[5];
    T u0 = params[6];
    T v0 = params[7];

    T len = sqrt(P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2]);
    T theta = acos(P_c[2] / len);
    T phi = atan2(P_c[1], P_c[0]);

    Eigen::Matrix<T,2,1> p_u = r(k2, k3, k4, k5, theta) * Eigen::Matrix<T,2,1>(cos(phi), sin(phi));

    p(0) = mu * p_u(0) + u0;
    p(1) = mv * p_u(1) + v0;
}
//EquidistantCamera.h

//ScaramuzzaCamera.h
#define SCARAMUZZA_POLY_SIZE 5
#define SCARAMUZZA_INV_POLY_SIZE 20

#define SCARAMUZZA_CAMERA_NUM_PARAMS (SCARAMUZZA_POLY_SIZE + SCARAMUZZA_INV_POLY_SIZE + 2 /*center*/ + 3 /*affine*/)

/**
 * Scaramuzza Camera (Omnidirectional)
 * https://sites.google.com/site/scarabotix/ocamcalib-toolbox
 */

class OCAMCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        Parameters();

        double& C(void) { return m_C; }
        double& D(void) { return m_D; }
        double& E(void) { return m_E; }

        double& center_x(void) { return m_center_x; }
        double& center_y(void) { return m_center_y; }

        double& poly(int idx) { return m_poly[idx]; }
        double& inv_poly(int idx) { return m_inv_poly[idx]; }

        double C(void) const { return m_C; }
        double D(void) const { return m_D; }
        double E(void) const { return m_E; }

        double center_x(void) const { return m_center_x; }
        double center_y(void) const { return m_center_y; }

        double poly(int idx) const { return m_poly[idx]; }
        double inv_poly(int idx) const { return m_inv_poly[idx]; }

        bool readFromYamlFile(const std::string& filename);
        void writeToYamlFile(const std::string& filename) const;

        Parameters& operator=(const Parameters& other);
        friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

    private:
        double m_poly[SCARAMUZZA_POLY_SIZE];
        double m_inv_poly[SCARAMUZZA_INV_POLY_SIZE];
        double m_C;
        double m_D;
        double m_E;
        double m_center_x;
        double m_center_y;
    };

    OCAMCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    OCAMCamera(const Parameters& params);

    Camera::ModelType modelType(void) const;
    const std::string& cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    void estimateIntrinsics(const cv::Size& boardSize,
                            const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints);

    // Lift points from the image plane to the sphere
    void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    //void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                  Eigen::Matrix<double,2,3>& J) const;
    //%output p
    //%output J

    void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const;
    //%output p

    template <typename T>
    static void spaceToPlane(const T* const params,
                             const T* const q, const T* const t,
                             const Eigen::Matrix<T, 3, 1>& P,
                             Eigen::Matrix<T, 2, 1>& p);
    template <typename T>
    static void spaceToSphere(const T* const params,
                              const T* const q, const T* const t,
                              const Eigen::Matrix<T, 3, 1>& P,
                              Eigen::Matrix<T, 3, 1>& P_s);
    template <typename T>
    static void LiftToSphere(const T* const params,
                              const Eigen::Matrix<T, 2, 1>& p,
                              Eigen::Matrix<T, 3, 1>& P);

    template <typename T>
    static void SphereToPlane(const T* const params, const Eigen::Matrix<T, 3, 1>& P,
                               Eigen::Matrix<T, 2, 1>& p);


    void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const;
    cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                    float fx = -1.0f, float fy = -1.0f,
                                    cv::Size imageSize = cv::Size(0, 0),
                                    float cx = -1.0f, float cy = -1.0f,
                                    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

    int parameterCount(void) const;

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    void readParameters(const std::vector<double>& parameterVec);
    void writeParameters(std::vector<double>& parameterVec) const;

    void writeParametersToYamlFile(const std::string& filename) const;

    std::string parametersToString(void) const;

private:
    Parameters mParameters;

    double m_inv_scale;
};

typedef boost::shared_ptr<OCAMCamera> OCAMCameraPtr;
typedef boost::shared_ptr<const OCAMCamera> OCAMCameraConstPtr;

template <typename T>
void
OCAMCamera::spaceToPlane(const T* const params,
                         const T* const q, const T* const t,
                         const Eigen::Matrix<T, 3, 1>& P,
                         Eigen::Matrix<T, 2, 1>& p)
{
    T P_c[3];
    {
        T P_w[3];
        P_w[0] = T(P(0));
        P_w[1] = T(P(1));
        P_w[2] = T(P(2));

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};

        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];
    }

    T c = params[0];
    T d = params[1];
    T e = params[2];
    T xc[2] = { params[3], params[4] };

    //T poly[SCARAMUZZA_POLY_SIZE];
    //for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
    //    poly[i] = params[5+i];

    T inv_poly[SCARAMUZZA_INV_POLY_SIZE];
    for (int i=0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        inv_poly[i] = params[5 + SCARAMUZZA_POLY_SIZE + i];

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1];
    T norm = T(0.0);
    if (norm_sqr > T(0.0))
        norm = sqrt(norm_sqr);

    T theta = atan2(-P_c[2], norm);
    T rho = T(0.0);
    T theta_i = T(1.0);

    for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
    {
        rho += theta_i * inv_poly[i];
        theta_i *= theta;
    }

    T invNorm = T(1.0) / norm;
    T xn[2] = {
        P_c[0] * invNorm * rho,
        P_c[1] * invNorm * rho
    };

    p(0) = xn[0] * c + xn[1] * d + xc[0];
    p(1) = xn[0] * e + xn[1]     + xc[1];
}

template <typename T>
void
OCAMCamera::spaceToSphere(const T* const params,
                          const T* const q, const T* const t,
                          const Eigen::Matrix<T, 3, 1>& P,
                          Eigen::Matrix<T, 3, 1>& P_s)
{
    T P_c[3];
    {
        T P_w[3];
        P_w[0] = T(P(0));
        P_w[1] = T(P(1));
        P_w[2] = T(P(2));

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};

        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];
    }

    //T poly[SCARAMUZZA_POLY_SIZE];
    //for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
    //    poly[i] = params[5+i];

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1] + P_c[2] * P_c[2];
    T norm = T(0.0);
    if (norm_sqr > T(0.0))
        norm = sqrt(norm_sqr);

    P_s(0) = P_c[0] / norm;
    P_s(1) = P_c[1] / norm;
    P_s(2) = P_c[2] / norm;
}

template <typename T>
void
OCAMCamera::LiftToSphere(const T* const params,
                          const Eigen::Matrix<T, 2, 1>& p,
                          Eigen::Matrix<T, 3, 1>& P)
{
    T c = params[0];
    T d = params[1];
    T e = params[2];
    T cc[2] = { params[3], params[4] };
    T poly[SCARAMUZZA_POLY_SIZE];
    for (int i=0; i < SCARAMUZZA_POLY_SIZE; i++)
       poly[i] = params[5+i];

    // Relative to Center
    T p_2d[2];
    p_2d[0] = T(p(0));
    p_2d[1] = T(p(1));

    T xc[2] = { p_2d[0] - cc[0], p_2d[1] - cc[1]};

    T inv_scale = T(1.0) / (c - d * e);

    // Affine Transformation
    T xc_a[2];

    xc_a[0] = inv_scale * (xc[0] - d * xc[1]);
    xc_a[1] = inv_scale * (-e * xc[0] + c * xc[1]);

    T norm_sqr = xc_a[0] * xc_a[0] + xc_a[1] * xc_a[1];
    T phi = sqrt(norm_sqr);
    T phi_i = T(1.0);
    T z = T(0.0);

    for (int i = 0; i < SCARAMUZZA_POLY_SIZE; i++)
    {
        if (i!=1) {
            z += phi_i * poly[i];
        }
        phi_i *= phi;
    }

    T p_3d[3];
    p_3d[0] = xc[0];
    p_3d[1] = xc[1];
    p_3d[2] = -z;

    T p_3d_norm_sqr = p_3d[0] * p_3d[0] + p_3d[1] * p_3d[1] + p_3d[2] * p_3d[2];
    T p_3d_norm = sqrt(p_3d_norm_sqr);

    P << p_3d[0] / p_3d_norm, p_3d[1] / p_3d_norm, p_3d[2] / p_3d_norm;
}

template <typename T>
void OCAMCamera::SphereToPlane(const T* const params, const Eigen::Matrix<T, 3, 1>& P,
                               Eigen::Matrix<T, 2, 1>& p) {
    T P_c[3];
    {
        P_c[0] = T(P(0));
        P_c[1] = T(P(1));
        P_c[2] = T(P(2));
    }

    T c = params[0];
    T d = params[1];
    T e = params[2];
    T xc[2] = {params[3], params[4]};

    T inv_poly[SCARAMUZZA_INV_POLY_SIZE];
    for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++)
        inv_poly[i] = params[5 + SCARAMUZZA_POLY_SIZE + i];

    T norm_sqr = P_c[0] * P_c[0] + P_c[1] * P_c[1];
    T norm = T(0.0);
    if (norm_sqr > T(0.0)) norm = sqrt(norm_sqr);

    T theta = atan2(-P_c[2], norm);
    T rho = T(0.0);
    T theta_i = T(1.0);

    for (int i = 0; i < SCARAMUZZA_INV_POLY_SIZE; i++) {
        rho += theta_i * inv_poly[i];
        theta_i *= theta;
    }

    T invNorm = T(1.0) / norm;
    T xn[2] = {P_c[0] * invNorm * rho, P_c[1] * invNorm * rho};

    p(0) = xn[0] * c + xn[1] * d + xc[0];
    p(1) = xn[0] * e + xn[1] + xc[1];
}
//ScaramuzzaCamera.h

//CameraCalibration.h
class CameraCalibration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraCalibration();

    CameraCalibration(Camera::ModelType modelType,
                      const std::string& cameraName,
                      const cv::Size& imageSize,
                      const cv::Size& boardSize,
                      float squareSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(void);

    int sampleCount(void) const;
    std::vector<std::vector<cv::Point2f> >& imagePoints(void);
    const std::vector<std::vector<cv::Point2f> >& imagePoints(void) const;
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;
    CameraPtr& camera(void);
    const CameraConstPtr camera(void) const;

    Eigen::Matrix2d& measurementCovariance(void);
    const Eigen::Matrix2d& measurementCovariance(void) const;

    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    void drawResults(std::vector<cv::Mat>& images) const;

    void writeParams(const std::string& filename) const;

    bool writeChessboardData(const std::string& filename) const;
    bool readChessboardData(const std::string& filename);

    void setVerbose(bool verbose);

private:
    bool calibrateHelper(CameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void optimize(CameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    cv::Size m_boardSize;
    float m_squareSize;

    CameraPtr m_camera;
    cv::Mat m_cameraPoses;

    std::vector<std::vector<cv::Point2f> > m_imagePoints;
    std::vector<std::vector<cv::Point3f> > m_scenePoints;

    Eigen::Matrix2d m_measurementCovariance;

    bool m_verbose;
};

}
//CameraCalibration.h





#endif
