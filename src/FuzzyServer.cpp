#include <ros/ros.h>
#include <tutorial2_controller/FuzzyControlAction.h>
#include <actionlib/server/simple_action_server.h>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"

//Fuzzylite library
#include "fl/Headers.h"
using namespace fl;

#define PI 3.14159265

//Class for containing the server
class FuzzyServer{
public:
	
	FuzzyServer(std::string name):

	as(n, "fuzzy_control", boost::bind(&FuzzyServer::executeCB, this, _1), false),
	action_name(name)
	{
		as.registerPreemptCallback(boost::bind(&FuzzyServer::preemptCB, this));

		//Start the server
		as.start();	  
		
		//Subscriber current positon of servo
		positionservosub = n2.subscribe("/sensor/encoder/servo", 1, &FuzzyServer::SensorCallBack, this);
		
		//Publisher setpoint, current position and error of control
		error_controlpub = n2.advertise<geometry_msgs::Vector3>("/control/error", 1);		
		
		//Publisher PID output in servo
		positionservopub = n2.advertise<std_msgs::Float64>("/motor/servo", 1);
		
		//Max e Min Output Servo Controller
		float max = PI;
		float min = -PI;
		
		//Initializing Brushless Controller
		Initialize(min,max);
}


//Callback for handling preemption. Reset your helpers here.
//Note that you still have to check for preemption in your work method to break it off
void preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}

//Callback for processing a goal
void executeCB(const tutorial2_controller::FuzzyControlGoalConstPtr& goal)
{
	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(100);

	//Setup some local variables
	bool success = true;	
	
	//Loop control
	while(1)
	{
		std_msgs::Float64 msg_pos;
		
		//Fuzzy Controller
		msg_pos.data = FuzzyController(goal->position, position_encoder);
		
		//Publishing Fuzzy output in servo
		positionservopub.publish(msg_pos);
		
		//Auxiliary Message
		geometry_msgs::Vector3 msg_error;
		
		msg_error.x = goal->position;
		msg_error.y = position_encoder;
		msg_error.z = goal->position - position_encoder;
		
		//Publishing setpoint, feedback and error control
		error_controlpub.publish(msg_error);
		
		feedback.position = position_encoder;
    
    //Publish feedback to action client
    as.publishFeedback(feedback);
		
		//Check for ROS kill
		if(!ros::ok())
		{
			success = false;
			ROS_INFO("%s Shutting Down", action_name.c_str());
			break;
		}

		//If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;
		
		//Sleep for rate time
		rate.sleep();
	}
	
	//Publish the result if the goal wasn't preempted
	if(success)
	{
		result.ok = 1;
		as.setSucceeded(result);
	}
	else
	{
		result.ok = 0;
		as.setAborted(result,"I Failed!");
	}
}

void Initialize( float min, float max)
{
	setOutputLimits(min, max);
  lastError = 0;
}

void setOutputLimits(float min, float max)
{
	if (min > max) return;
    
	minLimit = min;
	maxLimit = max;
}

float FuzzyController(float setpoint, float PV)
{
	float error = setpoint - PV;
	float dErr = error - lastError;	

  //Create fuzzy
  fl::Engine* engine = new fl::Engine;
  engine->setName("");
  
  //Membership functions of input error
  fl::InputVariable* inputVariable1 = new fl::InputVariable;
  inputVariable1->setEnabled(true);
  inputVariable1->setName("e");
  inputVariable1->setRange(-7.000, 7.000);
  inputVariable1->addTerm(new fl::Trapezoid("N", -7.000, -7.000, -1.750, 0.000));
  inputVariable1->addTerm(new fl::Triangle("Z", -1.750, 0.000, 1.750));
  inputVariable1->addTerm(new fl::Trapezoid("P", 0.000, 1.750, 7.000, 7.000));
  engine->addInputVariable(inputVariable1);
  
  //Membership functions of input change of error
  fl::InputVariable* inputVariable2 = new fl::InputVariable;
  inputVariable2->setEnabled(true);
  inputVariable2->setName("ce");
  inputVariable2->setRange(-7.000, 7.000);
  inputVariable2->addTerm(new fl::Trapezoid("N", -7.000, -7.000, -1.750, 0.000));
  inputVariable2->addTerm(new fl::Triangle("Z", -1.750, 0.000, 1.750));
  inputVariable2->addTerm(new fl::Trapezoid("P", 0.000, 1.750, 7.000, 7.000));
  engine->addInputVariable(inputVariable2);
  
  //Membership functions of output 
  fl::OutputVariable* outputVariable = new fl::OutputVariable;
  outputVariable->setEnabled(true);
  outputVariable->setName("out");
  outputVariable->setRange(-3.500, 3.500);
  outputVariable->fuzzyOutput()->setAccumulation(new fl::Maximum);
  outputVariable->setDefuzzifier(new fl::Centroid(200));
  outputVariable->setDefaultValue(0.000);
  outputVariable->setLockValidOutput(true);
  outputVariable->setLockOutputRange(true);
  outputVariable->addTerm(new fl::Triangle("NB", -3.500, -3.500, -1.750));
  outputVariable->addTerm(new fl::Triangle("NS", -3.500, -1.750, 0.000));
  outputVariable->addTerm(new fl::Triangle("Z", -1.750, 0.000, 1.750));
  outputVariable->addTerm(new fl::Triangle("PS", 0.000, 1.750, 3.500));
  outputVariable->addTerm(new fl::Triangle("PB", 1.750, 3.500, 3.500));
  engine->addOutputVariable(outputVariable);
  
  //Rules
  fl::RuleBlock* ruleBlock = new fl::RuleBlock;
  ruleBlock->setEnabled(true);
  ruleBlock->setName("");
  ruleBlock->setConjunction(new fl::Minimum);
  ruleBlock->setDisjunction(NULL);
  ruleBlock->setActivation(new fl::Minimum);
  ruleBlock->addRule(fl::Rule::parse("if e is N and ce is N then out is NB", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is N and ce is Z then out is NS", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is N and ce is P then out is Z", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is Z and ce is N then out is NS", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is Z and ce is Z then out is Z", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is Z and ce is P then out is PS", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is P and ce is N then out is Z", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is P and ce is Z then out is PS", engine));
  ruleBlock->addRule(fl::Rule::parse("if e is P and ce is P then out is PB", engine));
  engine->addRuleBlock(ruleBlock);
  
  //Set inputs
	inputVariable1->setInputValue(error);
	inputVariable2->setInputValue(dErr);
	
	//Start fuzzy
	engine->process();
	
	//Defuzzification
	fl::scalar out1 = outputVariable->defuzzify();
	
	//Increment output
	output += out1/90;
   
	// Clamp output to bounds
	output = std::min(output, maxLimit);
	output = std::max(output, minLimit);  

	//Required values for next round */
	lastError = error;
	
	return output;
}

void SensorCallBack(const sensor_msgs::JointState& msg)
{
	position_encoder = msg.position[0];
}

protected:
	ros::NodeHandle n;
	ros::NodeHandle n2;
	
	//Variables Subscriber
	ros::Subscriber positionservosub;
	
	//Variables Publisher
	ros::Publisher positionservopub;
	ros::Publisher error_controlpub;
	
	//Variables Actionlib
	actionlib::SimpleActionServer<tutorial2_controller::FuzzyControlAction> as;
	tutorial2_controller::FuzzyControlFeedback feedback;
	tutorial2_controller::FuzzyControlResult result;
	std::string action_name;
	
	//Variables control	
	float position_encoder;
	float lastError;
	float minLimit, maxLimit;
  float output;
};

//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "fuzzy_server");

 	//Just a check to make sure the usage was correct
	if(argc != 1)
	{
		ROS_INFO("Usage: fuzzy_server");
		return 1;
	}
	
	//Spawn the server
	FuzzyServer server(ros::this_node::getName());
  
	ros::spin();

	return 0;
}
