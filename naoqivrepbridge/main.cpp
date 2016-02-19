#include <iostream>
#include <thread>
#include <chrono>

#include <v_repConst.h>
#include <vrep_driver.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>

#include <boost/shared_ptr.hpp>
#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

#include <signal.h>
#include <stdlib.h>

using namespace std;

int _nao_joint_handles[40];
int _nao_top_camera_handle;
simxInt _camera_resolution[2] = {640, 480};

bool stop_execution = false;

void sigint_handler(int s) {
	stop_execution = true;
}

bool get_Nao_Handles(int clientID) {
	int i=0;
	bool all_ok = true;

	//Head
	all_ok &= (simxGetObjectHandle(clientID, "HeadYaw#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "HeadPitch#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Arm
	all_ok &= (simxGetObjectHandle(clientID, "LShoulderPitch3#",	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LShoulderRoll3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LElbowYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LElbowRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LWristYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Fingers
	all_ok &= (simxGetObjectHandle(clientID, "NAO_LThumbBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint8#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_LLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint12#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint14#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint11#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint13#",	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Left Leg
	all_ok &= (simxGetObjectHandle(clientID, "LHipYawPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LHipRoll3#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LHipPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LKneePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LAnklePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "LAnkleRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Leg
	all_ok &= (simxGetObjectHandle(clientID, "RHipYawPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RHipRoll3#", 			&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RHipPitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RKneePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RAnklePitch3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RAnkleRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Arm
	all_ok &= (simxGetObjectHandle(clientID, "RShoulderPitch3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RShoulderRoll3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RElbowYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RElbowRoll3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "RWristYaw3#", 		&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	//Right Fingers
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RThumbBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint0#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RLFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint5#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint6#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "NAO_RRFingerBase#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint2#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);
	all_ok &= (simxGetObjectHandle(clientID, "Revolute_joint3#", 	&_nao_joint_handles[i++], simx_opmode_oneshot_wait) == simx_return_ok);

	//Camera
	all_ok &= (simxGetObjectHandle(clientID, "NAO_vision1", 		&_nao_top_camera_handle, simx_opmode_oneshot_wait) == simx_return_ok);

	if(all_ok) {
		simxUChar** image;
		simxGetVisionSensorImage(clientID, _nao_top_camera_handle, _camera_resolution, image, 0, simx_opmode_streaming); // Start streaming operation
	}
	
	return all_ok;
}

void update_Nao_State_From_VREP(int clientID, AL::ALMotionProxy& motionProxy) {
	std::vector<float> joint_positions, times;
	joint_positions.resize(26);
	times.resize(26, 1.0);

	for (int joint = 0; joint < 26; ++joint)
	{
		if(joint > 7)
			simxGetJointPosition(clientID, _nao_joint_handles[joint+7], &joint_positions[joint], simx_opmode_oneshot_wait);
		else
			simxGetJointPosition(clientID, _nao_joint_handles[joint], &joint_positions[joint], simx_opmode_oneshot_wait);	

		if((joint == 7) or (joint == 25))
			joint_positions[joint] = 1.0 - joint_positions[joint];
	}

	motionProxy.angleInterpolation("Body", joint_positions, times, true);
}

void set_Nao_Joint_Commands(int clientID, AL::ALMotionProxy& motionProxy) {
	std::vector<float> commandAngles = motionProxy.getAngles("Body", false);

	for (int joint = 0, i=0; joint < 25; ++joint)
	{
		if((joint == 7) or (joint == 25))
			for (int j = 0; j < 8; ++j)
				simxSetJointTargetPosition(clientID, _nao_joint_handles[i++], 1.0 - commandAngles[joint], simx_opmode_streaming);
		else
			simxSetJointTargetPosition(clientID, _nao_joint_handles[i++], commandAngles[joint], simx_opmode_streaming);			
	}
}

const int _image_size = 640 * 480 * 3;

int main(int argc, char *argv[])
{
	string vrep_ip = "127.0.0.1", nao_ip = "127.0.0.1";
	int vrep_port = 19997, nao_port = 9559;
	if(argc == 1)
		cout << "Usage : naoqi_vrep_bridge vrep_ip vrep_port naoqi_ip naoqi_port" << endl;
	if(argc > 1)
		vrep_ip = argv[1];
	if(argc > 2)
		vrep_port = atoi(argv[2]);
	if(argc > 3)
		nao_ip = argv[3];
	if(argc > 4)
		nao_port = atoi(argv[4]);

	boost::shared_ptr<AL::ALBroker> broker;
	try {
		broker = AL::ALBroker::createBroker(
		    "mybroker",
		    "0.0.0.0",
		    54000,
		    nao_ip,
		    nao_port,
		    0    // you can pass various options for the broker creation,
		         // but default is fine
		  );
	}
	catch(const AL::ALError& /* e */) {
		std::cerr << "Faild to connect broker to: "
		          << nao_ip
		          << ":"
		          << nao_port
		          << std::endl;
		AL::ALBrokerManager::getInstance()->killAllBroker();
		AL::ALBrokerManager::kill();
		return -1;
	}

	cout << "Connecting to V-REP on " << vrep_ip << ":" << vrep_port << endl;
	int clientID = simxStart((simxChar*)vrep_ip.c_str(), vrep_port, 1, 1, 2000, 5);

	cout << "clientID = " << clientID << endl;
	if(clientID == -1) {
		cerr << "Can't connect to V-REP on " << vrep_ip << ":" << vrep_port << endl;
		return -1;
	}

	cout << "Successfuly connected to V-REP" << endl;

	cout << "Getting Nao joint handles" << endl;
	if(not get_Nao_Handles(clientID)) {
		cerr << "Can't get Nao joint handles" << endl;
		return -1;
	}

	cout << "Creating motion and vision proxies" << endl;
	AL::ALMotionProxy motionProxy(broker);
	AL::ALVideoDeviceProxy videoProxy(broker);

	cout << "Updating Naoqi state with V-REP state" << endl;
	update_Nao_State_From_VREP(clientID, motionProxy);

	cout << "Subscribing to Nao top camera" << endl;
	string subscriberID = "vrep_bridge";
	int fps = 10;
	int camera_period = 1000/fps;
	// The subscriberID can be altered if other instances are already running
	string camID = videoProxy.subscribeCamera(subscriberID, AL::kTopCamera, AL::kVGA, AL::kRGBColorSpace, fps);

	cout << "Starting simulation" << endl;	
	simxStartSimulation(clientID, simx_opmode_oneshot_wait);

	typedef chrono::duration<int, chrono::milliseconds::period> cycle;	//define the type 'cycle'
	cout << "Starting main loop" << endl;

	signal (SIGINT, sigint_handler);

	int refresh_camera = 20;
	auto next_camera_refresh = chrono::steady_clock::now() + cycle(camera_period);
	thread camera_update;
	string image_tab;
	image_tab.resize(_image_size);
	while(not stop_execution) {
		auto start_time = chrono::steady_clock::now();					// start_time = current time
		auto end_time = start_time + cycle(10);							// end_time = current_timme + 10ms

		set_Nao_Joint_Commands(clientID, motionProxy);
		if(start_time > next_camera_refresh) {
			if(not camera_update.joinable()) {
				camera_update = thread([clientID, &videoProxy, &image_tab](){
					simxUChar* image = 0;

					simxGetVisionSensorImage(clientID, _nao_top_camera_handle, _camera_resolution, &image, 0, simx_opmode_buffer);

					int line_size = 640 * 3;
					for (int y = 0, idx = 0; y < 480; ++y)
						for (int x = 0; x < line_size; ++x, ++idx)
							image_tab[_image_size - (y+1)*line_size + x] = image[idx];

					videoProxy.putImage(AL::kTopCamera, _camera_resolution[0], _camera_resolution[1], image_tab);
				});
				camera_update.detach();
			}
			else
				cout << "Updating image took too long" << endl;
			next_camera_refresh = start_time + cycle(camera_period);
		}

		if(chrono::steady_clock::now() > end_time)
			cout << "Can't keep the petiod" << endl;
		else
			this_thread::sleep_until(end_time);								// Stop thread execution until 'end_time' (next cycle)
	}

	cout << "Exiting" << endl;

	videoProxy.unsubscribe(camID);

	simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
	
	return 0;
}