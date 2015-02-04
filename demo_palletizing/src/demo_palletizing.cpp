#include <demo_palletizing/demo_palletizing.h>

namespace demo_palletizing
{
//-----------------------------------------------------------------
DemoPalletizing::DemoPalletizing()
{


}
//-----------------------------------------------------------------
}//end namespace demo_palletizing


/////////////////////////////////
//           MAIN              //
/////////////////////////////////


//---------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_palletizing");

    demo_palletizing::DemoPalletizing demo;

    ROS_INFO("Demo palletizing node ready");
    while(ros::ok())
    {
        //do smthing
        ros::spinOnce();
    }
    return 0;
}
//---------------------------------------------------------------------
