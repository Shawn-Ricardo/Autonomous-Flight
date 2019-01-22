#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>


#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
   integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}


VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  
  // simulation assumes that
  // rotor_1: top left
  // rotor_2: top right
  // rotor_3: bottom left
  // rotor_4: bottom_right

 
  // tau_x = (f1 - f2 + f3 - f4) * l
  // regarding roll, f1 and f3 produce positive thrusts, while f2 and f4 produce negative thrusts.


  // tau_y = (f1 + f2 - f3 - f4) * l
  // regarding pitch, f1 and f2 produce positive thrusts, f3 and f4 produce negative thrusts.

  // We are given the equation,
  // torque = kappa * thrust.
  // this equation relates collective thrust to torque via a kappa parameter (which is obtained through empirical measurement).

  // the collective thrust is given as -
  // rotor_1 spins clockwise and produces a counter-clockwise moment, so its contribution will be negative
  // rotor_4 spins clockwise and produces a counter-clockwise moment, so its contribution will be negative
  // rotor_2/3 spin counter-clockwise and produce a clockwise, positive moment.
  // tau_z = (-f1 + f2 + f3 - f4) * kappa

  // the last equation is collective thrust.
  // fc = f1 + f2 + f3 + f4

  // in this body frame, all thrusts are positive.

  // at this point, there are 4 equations to 4 unknowns.
  // solving for f1-f4 shows entities that are apparant in each euqations.
  // they are listed below, q1 - q4

  float l = L / 1.41;
  float q1 =   momentCmd.x / l;
  float q2 =   momentCmd.y / l;
  float q3 = - momentCmd.z / kappa;
  float q4 =   collThrustCmd;

  // pluggin them into the equations to obtain desired thrusts.

  float rotor_1_thrust = ( q1 + q2 + q3 + q4) / 4.f;
  float rotor_2_thrust = (-q1 + q2 - q3 + q4) / 4.f;
  float rotor_3_thrust = ( q1 - q2 - q3 + q4) / 4.f;
  float rotor_4_thrust = (-q1 - q2 + q3 + q4) / 4.f;

  cmd.desiredThrustsN[0] = CONSTRAIN(rotor_1_thrust, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(rotor_2_thrust, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(rotor_3_thrust, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(rotor_4_thrust, minMotorThrust, maxMotorThrust);

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

 
  V3F momentCmd;

  
  // calculate errors. These are proportional erors between actual
  // and commanded body rate values.
  float u_bar_p = kpPQR.x * (pqrCmd.x - pqr.x);
  float u_bar_q = kpPQR.y * (pqrCmd.y - pqr.y);
  float u_bar_r = kpPQR.z * (pqrCmd.z - pqr.z);

  // torques about axes.
  float tau_x = u_bar_p * Ixx;
  float tau_y = u_bar_q * Iyy;
  float tau_z = u_bar_r * Izz;

  // moments about axes to command
  momentCmd.x = tau_x;
  momentCmd.y = tau_y;
  momentCmd.z = tau_z;
  
  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in GLOBAL XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]. BODY FRAME
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. 

  
  V3F pqrCmd;

  // rotation matrix that maps the body frame to the world frame
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  
  // i can control the movement of my drone in the x and y axis via,
  // x_dot_dot (in world frame) = collective_thrust_in_body_frame * b_x
  // y_dot_dot (in world frame) = collective_thrust_in_body_frame * b_y

  // where b_x and b_y map body frame to world frame and are given by the Rotation Matrix ( which transforms body frame accelerations
  // into world frame accelerations, angles given in radians).

  // in order to control b_x and b_y, I have to control the rotation rates around the x and y axis in the body frame (which are the terms p and q).

  // to obtain a p and q
  // (p_c, q_c) = (1/R(3,3) * [[R(2,1) , -R(1,1)] , [R(2,2) , -R(2,2)]] * [[b_dot_x_c], [b_dot_y_c]]

  // the R(n,n) is known, that is the current rotation matrix.
  // but, [[b_dot_x_c], [b_dot_y_c]] is new. it is

  // b_dot_x_c = (1/tau_rp) * (b_x - b_x_commanded)
  // b_dot_y_c = (1/tau_rp) * (b_y - b_y_commanded)

  // here, we resemble a P controller, where b_x is the current value of that particular Rotation Matrix entry and b_x_commanded is the desired value.
  // To find b_x_commanded, take the commanded acceleration in the body frame (given by the thrust) and divide that into the desired x_dot_dot and y_dot_dot.


  // collective thrust is always positive ( in body frame)
  // find collective acceleration and turn negative
  float collAcc = -1 * (collThrustCmd / mass);

  // get commanded R values.
  // b_x_c = x_dot_dot_c / thrust.
  // we are given the x_dot_dot and the collective thrust, so find this b_x_c and ensure
  // it does not violate tilt constraint.
  float b_x_commanded = CONSTRAIN(accelCmd.x / collAcc, -maxTiltAngle, maxTiltAngle);
  float b_y_commanded = CONSTRAIN(accelCmd.y / collAcc, -maxTiltAngle, maxTiltAngle);

  // get proportional errors
  float b_dot_x_c = kpBank * (b_x_commanded - R(0,2));
  float b_dot_y_c = kpBank * (b_y_commanded - R(1,2));

  // now, translate these commanded rotation matrix elements into p and q for deployment on the drone.
  // note that the control knobs dictating x_dot_dot and y_dot_dot are the rotation matrix
  // elements at R[0,2] and R[1,2], respectively. p and q are just inputs that 
  // the Body-Rate Controller needs to function.

  // put the matrix into linear equations and solve for p and q.
  pqrCmd.x = (R(1,0) * b_dot_x_c - R(0,0) * b_dot_y_c) / R(2,2);
  pqrCmd.y = (R(1,1) * b_dot_x_c - R(0,1) * b_dot_y_c) / R(2,2);

  // note that body rate about the z axis is zero. This is controlled via the yaw controller.
  pqrCmd.z = 0.0;

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // NOTE: 
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - return a force, not an acceleration
  //  - for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  // get position error
  float position_error = posZCmd - posZ;
  // update integratedAltidudeError
  integratedAltitudeError += position_error;
  // constrain velocity
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  // get velocity error
  float velocity_error = velZCmd - velZ;
  // get desired acceleration
  float u_bar = kpPosZ * position_error + kpVelZ * velocity_error + KiPosZ * integratedAltitudeError * dt + accelZCmd;
  // convert desired acceleration to thrust
  thrust = -mass * ((u_bar - 9.81) / R(2,2));

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // NOTE: 
  //   - limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // constrain the XY velocity.
  // compare the magnitude of the XY vector. If this magnitude is less than
  // the threshold magnitude, adjust desired velocity.
  if (velCmd.magXY() > maxSpeedXY)
  {
    // when the mag of XY is greater than maxSpeedXY, the following ratio will
    // result in a value < 1. Multiplied into the velCmd will reduce the magnitude
    // to be equation to maxSpeedXY. It normalizes.
    velCmd *= maxSpeedXY / velCmd.magXY();
  }

  // obtain acceleration
  V3F accelComponent = kpVelXY * (velCmd - vel) + kpPosXY * (posCmd - pos) + accelCmdFF;
  accelComponent[2] = 0.0;

  // constrain acceleration the same way that velocity is constrained.
  if (accelComponent.magXY() > maxAccelXY)
  {
    accelComponent *= maxAccelXY / accelComponent.magXY();
  }

  return accelComponent;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  
  float yawRateCmd = 0;

  // this is a simple P controller.

  float yaw_cmd = kpYaw * (yawCmd - yaw);
  yawRateCmd = fmodf(yaw_cmd, 3.14);  // the range is from [-pi, pi]. fmodf will keep sign.

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
