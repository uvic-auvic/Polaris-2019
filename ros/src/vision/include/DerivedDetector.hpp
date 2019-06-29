#ifndef DERIVEDDERIVATIVE
#define DERIVEDDERIVATIVE

#include "Detector.hpp"
#include "Filter.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class DerivedDetector : public Detector , Filter
{
public:
    DerivedDetector(CameraInput input, std::string cascade_name);
    bool update();
    // add functions as needed

private:
    CameraInput &camera_input;
    // add functions/variables as needed
};

#endif