
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

float i_term = 0.;
float velZ_1 = 0.;
float velZ_2 = 0.;

float t = 0.;

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
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS:
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float l = L / ( 2 * sqrt(2.f));

  float tau_x = momentCmd.x;
  float tau_y = momentCmd.y;
  float tau_z = momentCmd.z;

  float F_x = tau_x / L;
  float F_y = tau_y / L;
  float F_z = - tau_z / kappa;
  float F_total = collThrustCmd;

  float F_1 = (F_x + F_y + F_z + F_total)/4.f;
  float F_2 = (-F_x + F_y - F_z + F_total)/4.f;
  float F_3 = (-F_x - F_y + F_z + F_total)/4.f;
  float F_4 = (F_x - F_y - F_z + F_total)/4.f;

  F_1 = CONSTRAIN(F_1, minMotorThrust, maxMotorThrust);
  F_2 = CONSTRAIN(F_2, minMotorThrust, maxMotorThrust);
  F_3 = CONSTRAIN(F_3, minMotorThrust, maxMotorThrust);
  F_4 = CONSTRAIN(F_4, minMotorThrust, maxMotorThrust);

  cmd.desiredThrustsN[0] = F_1; // front left
  cmd.desiredThrustsN[1] = F_2; // front right
  cmd.desiredThrustsN[2] = F_4; // rear left
  cmd.desiredThrustsN[3] = F_3; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:https:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F I;
  I.x = Ixx;
  I.y = Iyy;
  I.z = Izz;
  momentCmd = I * kpPQR * ( pqrCmd - pqr );

  //printf("momentCmd= %f %f %f\n",momentCmd.x, momentCmd.y, momentCmd.z);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F acc_limit;
  acc_limit = collThrustCmd / mass;

  float kpBank_x = kpBank;

  float b_x = R(0,2); //R13
  float b_x_err = accelCmd.x/acc_limit.x - b_x;

  float b_x_p_term = kpBank * b_x_err;

  float b_y = R(1,2); //R23
  float b_y_err = accelCmd.y/acc_limit.y - b_y;
  float b_y_p_term = 15 * b_y_err;

  Mat3x3F rot_mat1;
  rot_mat1(0,0) = R(1,0);
  rot_mat1(0,1) = -R(0,0);
  rot_mat1(1,0) = R(1,1);
  rot_mat1(1,1) = -R(0,1);

  rot_mat1 = rot_mat1 / R(2,2);

  V3F cmd_dot;
  cmd_dot.x = b_x_p_term;
  cmd_dot.y = b_y_p_term;

  V3F rot_rate;
  rot_rate = rot_mat1 * cmd_dot;

  pqrCmd.x = rot_rate.x;
  pqrCmd.y = rot_rate.y;
  pqrCmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

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

  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  float i_term = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float z_err = posZCmd - posZ;
  float z_dot_err = velZCmd - velZ;

  float kpPosZ_up_limit = 80.;
  float kpPosZ_down_limit = 18.;

  if (abs(z_err)>=0.05)
      kpPosZ = kpPosZ + 0.085;

  else if(abs(z_err)<0.05 && abs(z_err)>0.01)
      kpPosZ = kpPosZ - 0.8;

  if (kpPosZ > kpPosZ_up_limit)
      kpPosZ = kpPosZ_up_limit;

  else if (kpPosZ<kpPosZ_down_limit)
      kpPosZ = kpPosZ_down_limit;

  //printf("kpPosZ %f\n", kpPosZ);

  float p_term = kpPosZ * z_err;
  float d_term = kpVelZ * z_dot_err;
  integratedAltitudeError =+ z_err * dt;

  i_term = integratedAltitudeError * KiPosZ;

  float b_z = R(2,2);

  float u_1_bar = p_term + d_term + i_term;

  float u = ( u_1_bar - CONST_GRAVITY ) / b_z;

  float acc = CONSTRAIN(u, -maxAscentRate / dt, maxDescentRate / dt);

  thrust = - acc * mass;

  //t = t + dt;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmd: desired acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you cap the horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmd.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F kpPos;
  kpPos.x = kpPosXY;
  kpPos.y = kpPosXY;
  kpPos.z = 0.f;

  V3F kpVel;
  kpVel.x = kpVelXY;
  kpVel.y = kpVelXY;
  kpVel.z = 0.f;

  V3F capVelCmd;
  capVelCmd.x = fmodf(velCmd.x, maxSpeedXY);
  capVelCmd.y = fmodf(velCmd.y, maxSpeedXY);
  capVelCmd.z = 0.f;

  V3F accel;
  accelCmd = kpPos * ( posCmd - pos ) + kpVel * ( capVelCmd - vel ) + accelCmd;

  //printf("accelCmd %f\n", accelCmd);

  V3F capaccel;
  capaccel.x = - CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  capaccel.y = - CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  capaccel.z = 0;

  capaccel = capaccel;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return capaccel;
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
  // HINTS:
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float yaw_error = yawCmd - yaw;

  yawRateCmd = kpYaw * yaw_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  //printf("curTrajPoint.position.z %f\n", curTrajPoint.accel.z);

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
