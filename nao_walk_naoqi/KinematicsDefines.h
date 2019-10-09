/* 	FIXED TO MATCH V4 SPECIFICATIONS 
	@author Mumra
*/
#ifndef KINEMATICDEFINES_H
#define KINEMATICDEFINES_H

//Define for the lengths of the arms foots etc
#define ShoulderOffsetY 	98.0
#define ShoulderOffsetZ		100.0
#define ElbowOffsetY		15.0
#define UpperArmLength		105.0
#define LowerArmLength		55.95
#define HandOffsetX		57.75
#define HandOffsetZ		12.31
#define HipOffsetZ		85.0
#define HipOffsetY		50.0
#define ThighLength		100.0
#define TibiaLength		102.9
#define FootHeight		45.19
#define NeckOffsetZ		126.5
#define CameraBotomX		48.8
#define CameraBotomZ		23.81
#define CameraTopX		53.9
#define CameraTopZ		67.9

//Head Limits
#define HeadYawHigh		2.0857
#define HeadYawLow		-2.0857
#define HeadPitchHigh		0.5149
#define HeadPitchLow		-0.6720
//Left Hand limits
#define LShoulderPitchHigh	2.0857
#define LShoulderPitchLow 	-2.0857
#define LShoulderRollHigh	1.3265
#define LShoulderRollLow 	-0.3142
#define LElbowYawHigh		2.0875
#define LElbowYawLow 		-2.0875
#define LElbowRollHigh  	0.00001f//Aldebaran gives this value (-0.0349f) but the hand can go further
#define LElbowRollLow 		-1.5446
#define LWristYawHigh		1.8238
#define LWristYawLow		-1.8238
//Right Hand limits
#define RShoulderPitchHigh 	2.0857
#define RShoulderPitchLow 	-2.0857
#define RShoulderRollHigh	0.3142
#define RShoulderRollLow	-1.3265
#define RElbowYawHigh 		2.0875
#define RElbowYawLow 		-2.0875
#define RElbowRollHigh		1.5446
#define RElbowRollLow		-0.00001f//Aldebaran gives this value (0.0349f) but the hand can go further
#define RWristYawHigh		1.8238
#define RWristYawLow		-1.8238
//Left Leg limits
#define LHipYawPitchHigh	0.740810
#define LHipYawPitchLow		-1.145303 
#define LHipRollHigh		0.790477
#define LHipRollLow		-0.379472
#define LHipPitchHigh		0.484090
#define LHipPitchLow		-1.535889
#define LKneePitchHigh		2.112528
#define LKneePitchLow		-0.092346
#define LAnklePitchHigh		0.922747
#define LAnklePitchLow		-1.189516
#define LAnkleRollHigh		0.769001
#define LAnkleRollLow		-0.397880
//Left Right limits
#define RHipYawPitchHigh	0.740810
#define RHipYawPitchLow		-1.145303 
#define RHipRollHigh		0.379472
#define RHipRollLow		-0.790477
#define RHipPitchHigh		0.484090
#define RHipPitchLow		-1.535889
#define RKneePitchHigh		2.120198
#define RKneePitchLow		-0.103083
#define RAnklePitchHigh		0.932056
#define RAnklePitchLow		-1.186448
#define RAnkleRollHigh		0.397935
#define RAnkleRollLow		-0.768992

//Masses defines
//Total mass + battery
#define TotalMassH25			5.182530
//Torso
#define TorsoMass			1.0496
#define TorsoX				-4.13
#define TorsoY				0.00
#define TorsoZ				43.42

//Not provided by aldebaran
#define BatteryMass			0.345
#define BatteryX			-30.00
#define BatteryY			0.00
#define BatteryZ			39.00

//Head
#define HeadYawMass			0.06442
#define HeadYawX			-0.01
#define HeadYawY			0.000
#define HeadYawZ			-27.42

#define HeadPitchMass			0.60533
#define HeadPitchX			-1.1200
#define HeadPitchY			0.000
#define HeadPitchZ			52.5800

//Right Hand
#define RShoulderPitchMass		0.07504
#define RShoulderPitchX			-1.6500
#define RShoulderPitchY			26.6300
#define RShoulderPitchZ			0.14

#define RShoulderRollMass		0.15777
#define RShoulderRollX			24.5500
#define RShoulderRollY			-5.6300
#define RShoulderRollZ			3.3

#define RElbowYawMass			0.06483
#define RElbowYawX			-27.44
#define RElbowYawY			0.00
#define RElbowYawZ			-0.14

#define RElbowRollMass			0.07761
#define RElbowRollX			25.5600
#define RElbowRollY			-2.8100
#define RElbowRollZ			0.7600


#define RWristYawMass			0.18533
#define RWristYawX			LowerArmLength+34.34
#define RWristYawY			-0.88
#define RWristYawZ			3.08

//Right Leg
#define RHipYawPitchMass		0.06981
#define RHipYawPitchX			-7.8100
#define RHipYawPitchY			11.14
#define RHipYawPitchZ			26.61

#define RHipRollMass			0.13053
#define RHipRollX			-15.49
#define RHipRollY			-0.29
#define RHipRollZ			-5.15

#define RHipPitchMass			0.38968
#define RHipPitchX			1.38
#define RHipPitchY			-2.21
#define RHipPitchZ			-53.73

#define RKneePitchMass			0.29142
#define RKneePitchX			4.5300
#define RKneePitchY			-2.2500
#define RKneePitchZ			-49.3600

#define RAnklePitchMass			0.13416
#define RAnklePitchX			0.4500
#define RAnklePitchY			-0.29
#define RAnklePitchZ			6.85

#define RAnkleRollMass			0.16184
#define RAnkleRollX			25.42
#define RAnkleRollY			-3.30
#define RAnkleRollZ			-32.39

#endif
