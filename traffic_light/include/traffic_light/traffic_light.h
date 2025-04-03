#include <iostream>
#include <string>
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include <std_msgs/Bool.h>

#include <yolov5_cone/BoundingBox.h>
#include <yolov5_cone/BoundingBox_vector.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <waypoint_maker/State.h> 


using namespace std;

//일단 애는 구조체 생성한거.
struct box_size  
{
    int width;
    int height;
    int id; 
};

//신호등 크기 비교하기 위해서 선언한거.
//======================================가로길이=width만 비교때린거.
bool compare(box_size a,  box_size b)
{
    return a.width > b.width;
}
//======================================넓이=width*height 넓이로 비교때린거 실험 해봐야할듯.
//bool compare(box_size a, box_size b)
//{
//    return a.width*a.height>b.width*b.height;
//}

class traffic_light
{
public:
    traffic_light()
    {   //토픽명: vision_traffic std_msg이용해서 한다.
        pub_traffic=nh.advertise<std_msgs::Bool>("vision_traffic", 100); //신호등 판탄하는거 보내는거

        
		darknet_sub_c = nh.subscribe("boundingbox_f", 6, &traffic_light::DarknetCallback_c, this); // 신호등 판단할거
        
        //미션 구간 나눠줄거
        location_sub_ = nh.subscribe("gps_state", 1, &traffic_light::stateCallback, this);
        
        vision_traffic.data = true; //원래 평상시에는 출발해야하니까 평소 값 true로 초기화
        
        //오버플로우 방지
        box_width.resize(10);
        box_height.resize(10);
		box_number.resize(10);
        width.resize(4);
    }


    vector <int> width;

    int decision_count = 0;


//======================노드 핸들     

    ros::NodeHandle nh; 

//======================퍼블리셔
    ros::Publisher pub_traffic; //신호등 퍼블리셔

//======================서브스크라이브
	ros::Subscriber darknet_sub_c;
     
    ros::Subscriber location_sub_;  //=============주행

//==============================state 사용할 변수
    int location=-1; 

//=============================사용할 vector 선언
    vector <int> box_width;
    vector <int> box_height;
	vector <int> box_number;  

    vector <box_size> final_box; //max값 할당해줄 벡터 구조체 이용한거

    //msg변수 선언
    std_msgs::Bool vision_traffic;  //신호등
    
    int max=0;
    //int final_max=max+1000;
    int result=0;

    //콜백 선언
    void stateCallback(const waypoint_maker::State::ConstPtr &current_state_); //=============주행

    void DarknetCallback_c(const yolov5_cone::BoundingBox_vector::ConstPtr &darknet_msg); 

    //함수 선언
    void traffic_control();
    void run();
};
/*
토픽명은  vision_traffic
*/