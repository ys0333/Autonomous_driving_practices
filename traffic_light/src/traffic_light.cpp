#include <traffic_light/traffic_light.h>


//=============주행
void traffic_light::stateCallback(const waypoint_maker::State::ConstPtr &current_state_) 
{ 
    location = current_state_->current_state;
}

//신호등 콜백을 받아올때 class id 가 6<=class_id<=9일때 즉 go, stop, sl, gl 
//일때 들어오면 그 정보에 대해서 크기가 큰 순선대로 max값에 집어넣고 그 max보다 큰 값이 들어왔을때 비교하는거다.
void traffic_light::DarknetCallback_c(const yolov5_cone::BoundingBox_vector::ConstPtr &darknet_msg)
{
    int size = darknet_msg->traffic.size();
    string class_name;


    // Clear previous results
    final_box.clear();
    width.clear();
    // Iterate through all detected bounding boxes
    for (int i = 0; i < darknet_msg->traffic.size(); ++i)
    {
        yolov5_cone::BoundingBox bounding_box = darknet_msg->traffic[i];


        int class_id = bounding_box.id;

        cout << "class_id ===== " << class_id << endl;

        // Check if the detected object is a traffic light (class_id between 4 and 7)
        if (class_id == 6 || class_id == 7 || class_id == 8 || class_id == 9)
        {
            // Store bounding box information
            box_size box;
            box.width = bounding_box.w;
            box.height = bounding_box.h;
            box.id = bounding_box.id;
            // Add to final_box vector (only store the latest detected traffic light)
            final_box.push_back(box);
        }
    }


    // Update result based on the most recent detected traffic light
    if (!final_box.empty())
    {
        result = final_box[0].id;
        cout << "final_box" << final_box[0].id << endl;

        // cout << "!final_box.empty() == " << !final_box.empty() << endl;
        // Get the index of the most recent detected traffic light (which is the last element in final_box)
        // int latest_index = final_box.size();
        // cout << "latest_index ======== " << latest_index << endl;


        // for (int i = 0; i< latest_index; i++)
        // {
        //     bool traffic_sig = compare(final_box[i], final_box[i+1]);

        //     if(traffic_sig == 1)
        //     {
        //         cout << "final_box" << final_box[i].id << endl;
        //         result = final_box[i].id;
        //     }
        //     else
        //     {
        //         result = final_box[i+1].id;
        //     }
        // }
        // Update result with the ID of the most recent detected traffic light
        //ROS_INFO("result: %d",result);
    }
    else if(final_box.empty())
    {
        result = 0;

        // cout << "들어오면 안되는 result == " << result << "===" << endl;
        decision_count = decision_count + 1;

        // cout << "decision_count ==== == === " <<  decision_count << " ======= " << endl;

        if(decision_count > 20)
        {
            // cout << "그냥 가라 이자식아" << endl;
            vision_traffic.data=true;
        }
    }
}

void traffic_light::traffic_control() 
{
    // cout << 'result ==== ' << result << endl;
    //cout<<"traffic==========="<<vision_traffic.data<<endl
    if(location==3 || location == 7 || location == 12)  //좌회전 신호등
    {   //위에 콜백에서 result안에는 더 가까운 쪽에 있는 신호등 인지해서 할당한 값이 들어가 있다
        //result 8,9 이기 때문에 sl은 빨간불+좌회전, gl은 직진+좌회전 이라서 좌회전해야 하는 state
        
        // cout << 'result ==== ' << result << endl;
        if(result==6 || result == 9) // Go
        {

            vision_traffic.data=true;
            decision_count = 0;
        
        }
        else if(result == 7 || result == 8) // Stop
        {

            vision_traffic.data=false;
            decision_count = 0;

        }


        // cout << "vision_traffic.data" << vision_traffic.data << endl;


        max=0; //각 스테이트 //각 스테이트 넘어갈때 마다 max값 초기화해야 다음 스테이트 넘어갈때 이전 맥스값 안 남음
    }
    else
    {
        vision_traffic.data=true;
    }

    // if(vision_traffic.data == true)
    // {
    //     cout << "vision_traffic.data == True" << endl;
    // }
    // else if(vision_traffic.data == false)
    // {
    //     cout << "vision_traffic.data == False" << endl;
    // }
    // else
    // {
    //     cout << "vision_traffic.data == 똥값 " << endl;
    // }
}



void traffic_light::run()
{
    traffic_light::traffic_control();
    // cout<<"sign======"<<vision_traffic.data <<endl;
    if(vision_traffic.data == true)
    {
        cout <<"True True True" << endl;
    }
    else if(vision_traffic.data == false)
    {
        cout << "false false false " << endl;
    }
    
    pub_traffic.publish(vision_traffic);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "traffic_light");
    traffic_light tl;
	ros::Rate loop_rate(30);

	while(ros::ok())
    {   ros::spinOnce();
        tl.run();
        loop_rate.sleep();
	}

	return 0;
}