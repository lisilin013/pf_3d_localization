#include <pf_localization/pf_localization.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pf_localization");
    pf_localization::PFLocalization pf_localization;
    ros::Rate r(10);
    while (ros::ok()) {
        pf_localization.run();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
