#include <pf_localization/pf_localization.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pf_localization");
    pf_localization::PFLocalization pf_localization;
    ros::spin();
    return 0;

}
