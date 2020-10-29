/*
***********************************************************************
* threadloop.h: thread-based DP controller and network
* function to run the whole loop on server (including TCP/IP server,
* senser, estimator, controller, planner, database, etc).
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _THREADLOOP_H_
#define _THREADLOOP_H_

#include "config.h"

namespace ASV {

class threadloop : public StateMonitor {
 public:
  threadloop() : StateMonitor(), _jsonparse(parameter_json_path) {
    // // write prettified JSON to another file
    // std::ofstream o("pretty.json");
    // o << std::setw(4) << j << std::endl;
  }
  ~threadloop() = default;

  void mainloop() {
    std::thread targettracking_thread(&threadloop::target_tracking_loop, this);
    std::thread route_planner_thread(&threadloop::route_planner_loop, this);
    std::thread path_planner_thread(&threadloop::path_planner_loop, this);
    std::thread estimator_thread(&threadloop::estimatorloop, this);
    std::thread controller_thread(&threadloop::controllerloop, this);
    std::thread sql_thread(&threadloop::sqlloop, this);
    std::thread gps_thread(&threadloop::gps_loop, this);
    std::thread marine_radar_thread(&threadloop::marine_radar_loop, this);
    std::thread timer_thread(&threadloop::utc_timer_loop, this);
    std::thread gui_thread(&threadloop::gui_loop, this);
    std::thread stm32_thread(&threadloop::stm32loop, this);
    std::thread socket_thread(&threadloop::socket_loop, this);
    std::thread statemonitor_thread(&threadloop::state_monitor_loop, this);

    targettracking_thread.join();
    route_planner_thread.join();
    path_planner_thread.join();
    estimator_thread.join();
    controller_thread.join();
    sql_thread.join();
    gps_thread.join();
    marine_radar_thread.join();
    timer_thread.join();
    gui_thread.join();
    stm32_thread.join();
    socket_thread.join();
    statemonitor_thread.join();
  }

 private:
  /********************* Real time Data  *********************/
  planning::RoutePlannerRTdata RoutePlanner_RTdata{
      common::STATETOGGLE::IDLE,  // state_toggle
      0,                          // setpoints_X
      0,                          // setpoints_Y;
      0,                          // setpoints_heading;
      0,                          // setpoints_longitude;
      0,                          // setpoints_latitude;
      "OFF",                      // UTM zone
      0,                          // speed
      0,                          // los_capture_radius
      Eigen::VectorXd::Zero(2),   // Waypoint_X
      Eigen::VectorXd::Zero(2),   // Waypoint_Y
      Eigen::VectorXd::Zero(2),   // Waypoint_longitude
      Eigen::VectorXd::Zero(2)    // Waypoint_latitude
  };

  // real time data of tracker
  control::trackerRTdata tracker_RTdata{
      control::TRACKERMODE::STARTED,  // trackermode
      Eigen::Vector3d::Zero(),        // setpoint
      Eigen::Vector3d::Zero()         // v_setpoint
  };

  // real time data of controller
  control::controllerRTdata<num_thruster, dim_controlspace> controller_RTdata{
      common::STATETOGGLE::IDLE,                           // state_toggle
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // tau
      Eigen::Matrix<double, dim_controlspace, 1>::Zero(),  // BalphaU
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // command_u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // command_rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // command_alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // command_alpha_deg
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // feedback_u
      Eigen::Matrix<int, num_thruster, 1>::Zero(),         // feedback_rotation
      Eigen::Matrix<double, num_thruster, 1>::Zero(),      // feedback_alpha
      Eigen::Matrix<int, num_thruster, 1>::Zero()          // feedback_alpha_deg
  };

  // realtime parameters of the estimators
  localization::estimatorRTdata estimator_RTdata{
      common::STATETOGGLE::IDLE,            // state_toggle
      Eigen::Matrix3d::Identity(),          // CTB2G
      Eigen::Matrix3d::Identity(),          // CTG2B
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement
      Eigen::Matrix<double, 6, 1>::Zero(),  // Measurement_6dof
      Eigen::Matrix<double, 6, 1>::Zero(),  // Marine_state
      Eigen::Matrix<double, 5, 1>::Zero(),  // radar_state
      Eigen::Matrix<double, 6, 1>::Zero(),  // State
      Eigen::Vector3d::Zero(),              // p_error
      Eigen::Vector3d::Zero(),              // v_error
      Eigen::Vector3d::Zero()               // BalphaU
  };

  // real time data
  planning::CartesianState Planning_Marine_state{
      0,           // x
      0,           // y
      M_PI / 3.0,  // theta
      0,           // kappa
      2,           // speed
      0,           // dspeed
      0,           // yaw_rate
      0            // yaw_accel
  };

  // real time GPS/IMU data
  messages::gpsRTdata gps_data{
      0,  // UTC
      0,  // latitude
      0,  // longitude
      0,  // heading
      0,  // pitch
      0,  // roll
      0,  // altitude
      0,  // Ve
      0,  // Vn
      0,  // roti
      0,  // status
      0,  // UTM_x
      0,  // UTM_y
      ""  // UTM_zone
  };

  // real time stm32 data
  messages::stm32data stm32_data{
      "",                              // UTC_time
      0,                               // command_u1
      -10,                             // command_u2
      0,                               // feedback_u1
      0,                               // feedback_u2
      0,                               // feedback_pwm1
      0,                               // feedback_pwm2
      0,                               // RC_X
      0,                               // RC_Y
      0,                               // RC_Mz
      0,                               // voltage_b1
      0,                               // voltage_b2
      0,                               // voltage_b3
      messages::STM32STATUS::STANDBY,  // feedback_stm32status
      messages::STM32STATUS::STANDBY,  // command_stm32status
      common::LINKSTATUS::CONNECTED    // linkstatus;
  };

  // real time gui-link data
  messages::guilinkRTdata<num_thruster, 3> guilink_RTdata{
      "",                                           // UTC_time
      messages::GUISTATUS::STANDBY,                 // guistutus_PC2gui
      messages::GUISTATUS::STANDBY,                 // guistutus_gui2PC
      common::LINKSTATUS::DISCONNECTED,             // linkstatus
      0,                                            // indicator_autocontrolmode
      0,                                            // indicator_windstatus
      0.0,                                          // latitude
      0.0,                                          // longitude
      Eigen::Matrix<double, 6, 1>::Zero(),          // State
      0.0,                                          // roll
      0.0,                                          // pitch
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_rotation
      Eigen::Matrix<int, num_thruster, 1>::Zero(),  // feedback_alpha
      Eigen::Matrix<double, 3, 1>::Zero(),          // battery_voltage
      Eigen::Vector3d::Zero(),                      // setpoints
      0.0,                                          // desired_speed
      Eigen::Vector2d::Zero(),                      // startingpoint
      Eigen::Vector2d::Zero(),                      // endingpoint
      Eigen::Matrix<double, 2, 8>::Zero(),          // waypoints
      Eigen::VectorXd::Zero(2),                     // WX
      Eigen::VectorXd::Zero(2)                      // WY
  };

  // real time data of target tracker
  perception::TargetTrackerRTdata<max_num_targets> TargetTracker_RTdata{
      perception::SPOKESTATE::OUTSIDE_ALARM_ZONE,         // spoke_state
      Eigen::Matrix<int, max_num_targets, 1>::Zero(),     // targets_state
      Eigen::Matrix<int, max_num_targets, 1>::Zero(),     // targets_intention
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_x
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_y
      Eigen::Matrix<double, max_num_targets,
                    1>::Zero(),  // targets_square_radius
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_vx
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_vy
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_CPA_x
      Eigen::Matrix<double, max_num_targets, 1>::Zero(),  // targets_CPA_y
      Eigen::Matrix<double, max_num_targets, 1>::Zero()   // targets_TCPA
  };

  // real time SpokeProcess data
  perception::SpokeProcessRTdata SpokeProcess_RTdata;

  // real time radar detected data
  perception::TargetDetectionRTdata TargetDetection_RTdata;

  // real time data from marine radar
  messages::MarineRadarRTdata MarineRadar_RTdata{
      common::STATETOGGLE::IDLE,  // state_toggle
      0,                          // spoke_azimuth_deg
      0,                          // spoke_samplerange_m
      {0x00, 0x00, 0x00}          // spokedata
  };

  // real time utc
  std::string pt_utc;

  /********************* Modules  *********************/
  // json
  common::jsonparse<num_thruster, dim_controlspace> _jsonparse;

  //##################### target tracking ########################//
  void target_tracking_loop() {
    perception::TargetTracking<> Target_Tracking(
        _jsonparse.getalarmzonedata(), _jsonparse.getSpokeProcessdata(),
        _jsonparse.getTargetTrackingdata(), _jsonparse.getClusteringdata());

    common::timecounter timer_targettracking;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * Target_Tracking.getsampletime());

    StateMonitor::check_target_tracking();

    std::size_t size_spokedata = sizeof(MarineRadar_RTdata.spokedata) /
                                 sizeof(MarineRadar_RTdata.spokedata[0]);
    while (1) {
      outerloop_elapsed_time = timer_targettracking.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::SIMULATION_FRENET:
        case common::TESTMODE::SIMULATION_AVOIDANCE:
        case common::TESTMODE::EXPERIMENT_DP:
        case common::TESTMODE::EXPERIMENT_LOS:
        case common::TESTMODE::EXPERIMENT_FRENET: {
          // no planner
          break;
        }
        case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
          TargetTracker_RTdata =
              Target_Tracking
                  .AutoTracking(MarineRadar_RTdata.spokedata, size_spokedata,
                                MarineRadar_RTdata.spoke_azimuth_deg,
                                MarineRadar_RTdata.spoke_samplerange_m,
                                estimator_RTdata.radar_state(0),
                                estimator_RTdata.radar_state(1),
                                estimator_RTdata.radar_state(3),
                                estimator_RTdata.radar_state(4),
                                estimator_RTdata.radar_state(5))
                  .getTargetTrackerRTdata();

          SpokeProcess_RTdata = Target_Tracking.getSpokeProcessRTdata();
          TargetDetection_RTdata = Target_Tracking.getTargetDetectionRTdata();
          break;
        }
        default:
          break;
      }  // end switch

      innerloop_elapsed_time = timer_targettracking.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time_ms - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time_ms)
        CLOG(INFO, "TargetTracking") << "Too much time!";
    }
  }  // target_tracking_loop

  //##################### route planning ########################//
  void route_planner_loop() {
    planning::RoutePlanning Route_Planner(RoutePlanner_RTdata,
                                          _jsonparse.getvessel());

    while (1) {
      if (RoutePlanner_RTdata.state_toggle == common::STATETOGGLE::IDLE) {
        // double initial_long = 121.4378246;
        // double initial_lat = 31.0285510;
        // double final_long = 121.4388565;
        // double final_lat = 31.0282296;
        double initial_long = 121.4377186;
        double initial_lat = 31.0286309;
        double final_long = 121.4389307;
        double final_lat = 31.0281764;

        Eigen::VectorXd W_long(2);
        Eigen::VectorXd W_lat(2);
        W_long << initial_long, final_long;
        W_lat << initial_lat, final_lat;

        RoutePlanner_RTdata = Route_Planner.setCruiseSpeed(1)
                                  .setWaypoints(W_long, W_lat)
                                  .getRoutePlannerRTdata();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }  // route_planner_loop

  //##################### local path planner ########################//
  void path_planner_loop() {
    planning::LatticePlanner _trajectorygenerator(
        _jsonparse.getlatticedata(), _jsonparse.getcollisiondata());

    common::timecounter timer_planner;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time_ms =
        static_cast<long int>(1000 * _trajectorygenerator.getsampletime());

    StateMonitor::check_pathplanner();

    _trajectorygenerator.regenerate_target_course(
        RoutePlanner_RTdata.Waypoint_X, RoutePlanner_RTdata.Waypoint_Y);

    while (1) {
      outerloop_elapsed_time = timer_planner.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::EXPERIMENT_DP:
        case common::TESTMODE::EXPERIMENT_LOS: {
          // no planner
          break;
        }
        case common::TESTMODE::SIMULATION_FRENET: {
          std::vector<double> ob_x{3433794};
          std::vector<double> ob_y{350955};

          _trajectorygenerator.setup_obstacle(ob_x, ob_y);

          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5),
                                     RoutePlanner_RTdata.speed)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        case common::TESTMODE::SIMULATION_AVOIDANCE: {
          if (TargetTracker_RTdata.spoke_state ==
              perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
            _trajectorygenerator.setup_obstacle(
                TargetTracker_RTdata.targets_state,
                TargetTracker_RTdata.targets_x, TargetTracker_RTdata.targets_y);
          }

          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5),
                                     RoutePlanner_RTdata.speed)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        case common::TESTMODE::EXPERIMENT_FRENET: {
          std::vector<double> ob_x{3433797};
          std::vector<double> ob_y{350948.5};

          _trajectorygenerator.setup_obstacle(ob_x, ob_y);

          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5),
                                     RoutePlanner_RTdata.speed)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
          if (TargetTracker_RTdata.spoke_state ==
              perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
            _trajectorygenerator.setup_obstacle(
                TargetTracker_RTdata.targets_state,
                TargetTracker_RTdata.targets_x, TargetTracker_RTdata.targets_y);
          }

          auto Plan_cartesianstate =
              _trajectorygenerator
                  .trajectoryonestep(estimator_RTdata.Marine_state(0),
                                     estimator_RTdata.Marine_state(1),
                                     estimator_RTdata.Marine_state(2),
                                     estimator_RTdata.Marine_state(3),
                                     estimator_RTdata.Marine_state(4),
                                     estimator_RTdata.Marine_state(5),
                                     RoutePlanner_RTdata.speed)
                  .getnextcartesianstate();

          std::tie(Planning_Marine_state.x, Planning_Marine_state.y,
                   Planning_Marine_state.theta, Planning_Marine_state.kappa,
                   Planning_Marine_state.speed, Planning_Marine_state.dspeed) =
              common::math::Cart2Marine(
                  Plan_cartesianstate.x, Plan_cartesianstate.y,
                  Plan_cartesianstate.theta, Plan_cartesianstate.kappa,
                  Plan_cartesianstate.speed, Plan_cartesianstate.dspeed);
          break;
        }
        default:
          break;
      }  // end switch

      innerloop_elapsed_time = timer_planner.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time_ms - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time_ms)
        CLOG(INFO, "planner") << "Too much time!";
    }

  }  // path_planner_loop

  //################### path following, controller, TA ####################//
  void controllerloop() {
    control::controller<10, num_thruster, indicator_actuation, dim_controlspace>
        _controller(controller_RTdata, _jsonparse.getcontrollerdata(),
                    _jsonparse.getvessel(), _jsonparse.getpiddata(),
                    _jsonparse.getthrustallocationdata(),
                    _jsonparse.gettunneldata(), _jsonparse.getazimuthdata(),
                    _jsonparse.getmainrudderdata(),
                    _jsonparse.gettwinfixeddata());

    control::trajectorytracking _trajectorytracking(
        _jsonparse.getcontrollerdata(), tracker_RTdata);

    common::timecounter timer_controler;
    long int outerloop_elapsed_time = 0;
    long int innerloop_elapsed_time = 0;
    long int sample_time =
        static_cast<long int>(1000 * _controller.getsampletime());

    StateMonitor::check_controller();

    controller_RTdata =
        _controller.initializecontroller().getcontrollerRTdata();

    _trajectorytracking.set_grid_points(
        RoutePlanner_RTdata.Waypoint_X, RoutePlanner_RTdata.Waypoint_Y,
        RoutePlanner_RTdata.speed, RoutePlanner_RTdata.los_capture_radius);

    while (1) {
      outerloop_elapsed_time = timer_controler.timeelapsed();

      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          tracker_RTdata.setpoint = Eigen::Vector3d::Zero();
          tracker_RTdata.v_setpoint = Eigen::Vector3d::Zero();

          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);

          break;
        }
        case common::TESTMODE::SIMULATION_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          _trajectorytracking.Grid_LOS(estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::SIMULATION_FRENET: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();
          break;
        }
        case common::TESTMODE::SIMULATION_AVOIDANCE: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();

          break;
        }
        case common::TESTMODE::EXPERIMENT_DP: {
          _controller.setcontrolmode(control::CONTROLMODE::DYNAMICPOSITION);
          tracker_RTdata.setpoint = Eigen::Vector3d::Zero();
          tracker_RTdata.v_setpoint = Eigen::Vector3d::Zero();

          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);

          break;
        }
        case common::TESTMODE::EXPERIMENT_LOS: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);

          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);

          // trajectory tracking
          _trajectorytracking.Grid_LOS(estimator_RTdata.State.head(2));
          tracker_RTdata = _trajectorytracking.gettrackerRTdata();

          break;
        }
        case common::TESTMODE::EXPERIMENT_FRENET: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();

          break;
        }
        case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
          _controller.setcontrolmode(control::CONTROLMODE::MANEUVERING);
          _controller.set_thruster_feedback(
              controller_RTdata.command_rotation,
              controller_RTdata.command_alpha_deg);
          // trajectory tracking
          tracker_RTdata = _trajectorytracking
                               .FollowCircularArc(Planning_Marine_state.kappa,
                                                  Planning_Marine_state.speed,
                                                  Planning_Marine_state.theta)
                               .gettrackerRTdata();

          break;
        }
        default:
          break;
      }  // end switch

      // controller
      controller_RTdata = _controller
                              .controlleronestep(Eigen::Vector3d::Zero(),
                                                 estimator_RTdata.p_error,
                                                 estimator_RTdata.v_error,
                                                 Eigen::Vector3d::Zero(),
                                                 tracker_RTdata.v_setpoint)
                              .getcontrollerRTdata();
      // std::cout << elapsed_time << std::endl;
      innerloop_elapsed_time = timer_controler.timeelapsed();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

      if (outerloop_elapsed_time > 1.1 * sample_time)
        CLOG(INFO, "controller") << "Too much time!";
    }
  }  // controllerloop

  //##################### state estimation and simulator ####################//
  void estimatorloop() {
    // initialization of estimator
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        // simulation
        localization::estimator<indicator_kalman,  // indicator_kalman
                                1,                 // nlp_x
                                1,                 // nlp_y
                                1,                 // nlp_z
                                1,                 // nlp_heading
                                1,                 // nlp_roll
                                1,                 // nlp_pitch
                                1,                 // nlp_u
                                1,                 // nlp_v
                                1                  // nlp_roti
                                >
            _estimator(estimator_RTdata, _jsonparse.getvessel(),
                       _jsonparse.getestimatordata());

        simulation::simulator _simulator(_jsonparse.getsimulatordata(),
                                         _jsonparse.getvessel());

        common::timecounter timer_estimator;
        long int outerloop_elapsed_time = 0;
        long int innerloop_elapsed_time = 0;
        long int sample_time =
            static_cast<long int>(1000 * _estimator.getsampletime());

        // State monitor toggle
        StateMonitor::check_estimator();

        estimator_RTdata =
            _estimator.setvalue(350938.7, 3433823.54, 0, 0, 0, 90, 0, 0, 0)
                .getEstimatorRTData();
        _simulator.setX(estimator_RTdata.State);

        // real time calculation in estimator
        while (1) {
          outerloop_elapsed_time = timer_estimator.timeelapsed();

          auto x = _simulator
                       .simulator_onestep(tracker_RTdata.setpoint(2),
                                          controller_RTdata.BalphaU)
                       .getX();
          _estimator
              .updateestimatedforce(controller_RTdata.BalphaU,
                                    Eigen::Vector3d::Zero())
              .estimatestate(x, tracker_RTdata.setpoint(2));

          estimator_RTdata = _estimator
                                 .estimateerror(tracker_RTdata.setpoint,
                                                tracker_RTdata.v_setpoint)
                                 .getEstimatorRTData();

          innerloop_elapsed_time = timer_estimator.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "estimator") << "Too much time!";
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET:
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        //                  experiment                        //

        // initializtion
        localization::estimator<indicator_kalman,  // indicator_kalman
                                1,                 // nlp_x
                                1,                 // nlp_y
                                1,                 // nlp_z
                                1,                 // nlp_heading
                                1,                 // nlp_roll
                                1,                 // nlp_pitch
                                5,                 // nlp_u
                                5,                 // nlp_v
                                1                  // nlp_roti
                                >
            _estimator(estimator_RTdata, _jsonparse.getvessel(),
                       _jsonparse.getestimatordata());

        common::timecounter timer_estimator;
        long int outerloop_elapsed_time = 0;
        long int innerloop_elapsed_time = 0;
        long int sample_time =
            static_cast<long int>(1000 * _estimator.getsampletime());

        // State monitor toggle
        StateMonitor::check_estimator();
        estimator_RTdata = _estimator
                               .setvalue(gps_data.UTM_x,     // gps_x
                                         gps_data.UTM_y,     // gps_y
                                         gps_data.altitude,  // gps_z
                                         gps_data.roll,      // gps_roll
                                         gps_data.pitch,     // gps_pitch
                                         gps_data.heading,   // gps_heading
                                         gps_data.Ve,        // gps_Ve
                                         gps_data.Vn,        // gps_Vn
                                         gps_data.roti       // gps_roti
                                         )
                               .getEstimatorRTData();

        // real time calculation in estimator
        while (1) {
          outerloop_elapsed_time = timer_estimator.timeelapsed();

          _estimator
              .updateestimatedforce(controller_RTdata.BalphaU,
                                    Eigen::Vector3d::Zero())
              .estimatestate(gps_data.UTM_x,             // gps_x
                             gps_data.UTM_y,             // gps_y
                             gps_data.altitude,          // gps_z
                             gps_data.roll,              // gps_roll
                             gps_data.pitch,             // gps_pitch
                             gps_data.heading,           // gps_heading
                             gps_data.Ve,                // gps_Ve
                             gps_data.Vn,                // gps_Vn
                             gps_data.roti,              // gps_roti
                             tracker_RTdata.setpoint(2)  //_dheading
              );

          estimator_RTdata = _estimator
                                 .estimateerror(tracker_RTdata.setpoint,
                                                tracker_RTdata.v_setpoint)
                                 .getEstimatorRTData();

          innerloop_elapsed_time = timer_estimator.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "estimator") << "Too much time!";
        }

        break;
      }
      default:
        break;
    }

  }  // estimatorloop()

  // loop to save real time data using sqlite3 and modern_sqlite3_cpp_wrapper
  void sqlloop() {
    std::string sqlpath = _jsonparse.getsqlitepath();
    std::string db_config_path = _jsonparse.getdbconfigpath();

    common::gps_db _gps_db(sqlpath, db_config_path);
    common::stm32_db _stm32_db(sqlpath, db_config_path);
    common::marineradar_db _marineradar_db(sqlpath, db_config_path);
    common::estimator_db _estimator_db(sqlpath, db_config_path);
    common::planner_db _planner_db(sqlpath, db_config_path);
    common::controller_db _controller_db(sqlpath, db_config_path);
    common::perception_db _perception_db(sqlpath, db_config_path);

    _gps_db.create_table();
    _stm32_db.create_table();
    _marineradar_db.create_table();
    _estimator_db.create_table();
    _planner_db.create_table();
    _controller_db.create_table();
    _perception_db.create_table();

    while (1) {
      switch (testmode) {
        case common::TESTMODE::SIMULATION_DP:
        case common::TESTMODE::SIMULATION_LOS:
        case common::TESTMODE::SIMULATION_FRENET: {
          // simulation
          _estimator_db.update_measurement_table(
              common::est_measurement_db_data{
                  -1,                               // local_time
                  estimator_RTdata.Measurement(0),  // meas_x
                  estimator_RTdata.Measurement(1),  // meas_y
                  estimator_RTdata.Measurement(2),  // meas_theta
                  estimator_RTdata.Measurement(3),  // meas_u
                  estimator_RTdata.Measurement(4),  // meas_v
                  estimator_RTdata.Measurement(5)   // meas_r
              });
          _estimator_db.update_state_table(common::est_state_db_data{
              -1,                                // local_time
              estimator_RTdata.State(0),         // state_x
              estimator_RTdata.State(1),         // state_y
              estimator_RTdata.State(2),         // state_theta
              estimator_RTdata.State(3),         // state_u
              estimator_RTdata.State(4),         // state_v
              estimator_RTdata.State(5),         // state_r
              estimator_RTdata.Marine_state(3),  // curvature
              estimator_RTdata.Marine_state(4),  // speed
              estimator_RTdata.Marine_state(5)   // dspeed
          });
          _estimator_db.update_error_table(common::est_error_db_data{
              -1,                           // local_time
              estimator_RTdata.p_error(0),  // perror_x
              estimator_RTdata.p_error(1),  // perror_y
              estimator_RTdata.p_error(2),  // perror_mz
              estimator_RTdata.v_error(0),  // verror_x
              estimator_RTdata.v_error(1),  // verror_y
              estimator_RTdata.v_error(2)   // verror_mz
          });

          _controller_db.update_setpoint_table(common::control_setpoint_db_data{
              -1,                            // local_time
              tracker_RTdata.setpoint(0),    // set_x
              tracker_RTdata.setpoint(1),    // set_y
              tracker_RTdata.setpoint(2),    // set_theta
              tracker_RTdata.v_setpoint(0),  // set_u
              tracker_RTdata.v_setpoint(1),  // set_v
              tracker_RTdata.v_setpoint(2)   // set_r
          });
          _controller_db.update_TA_table(common::control_TA_db_data{
              -1,                            // local_time
              controller_RTdata.tau(0),      // desired_Fx
              controller_RTdata.tau(1),      // desired_Fy
              controller_RTdata.tau(2),      // desired_Mz
              controller_RTdata.BalphaU(0),  // est_Fx
              controller_RTdata.BalphaU(1),  // est_Fy
              controller_RTdata.BalphaU(2),  // est_Mz
              std::vector<int>(controller_RTdata.command_alpha_deg.data(),
                               controller_RTdata.command_alpha_deg.data() +
                                   num_thruster),  // alpha
              std::vector<int>(controller_RTdata.command_rotation.data(),
                               controller_RTdata.command_rotation.data() +
                                   num_thruster)  // rpm
          });
          break;
        }
        case common::TESTMODE::SIMULATION_AVOIDANCE: {
          _estimator_db.update_measurement_table(
              common::est_measurement_db_data{
                  -1,                               // local_time
                  estimator_RTdata.Measurement(0),  // meas_x
                  estimator_RTdata.Measurement(1),  // meas_y
                  estimator_RTdata.Measurement(2),  // meas_theta
                  estimator_RTdata.Measurement(3),  // meas_u
                  estimator_RTdata.Measurement(4),  // meas_v
                  estimator_RTdata.Measurement(5)   // meas_r
              });
          _estimator_db.update_state_table(common::est_state_db_data{
              -1,                                // local_time
              estimator_RTdata.State(0),         // state_x
              estimator_RTdata.State(1),         // state_y
              estimator_RTdata.State(2),         // state_theta
              estimator_RTdata.State(3),         // state_u
              estimator_RTdata.State(4),         // state_v
              estimator_RTdata.State(5),         // state_r
              estimator_RTdata.Marine_state(3),  // curvature
              estimator_RTdata.Marine_state(4),  // speed
              estimator_RTdata.Marine_state(5)   // dspeed
          });
          _estimator_db.update_error_table(common::est_error_db_data{
              -1,                           // local_time
              estimator_RTdata.p_error(0),  // perror_x
              estimator_RTdata.p_error(1),  // perror_y
              estimator_RTdata.p_error(2),  // perror_mz
              estimator_RTdata.v_error(0),  // verror_x
              estimator_RTdata.v_error(1),  // verror_y
              estimator_RTdata.v_error(2)   // verror_mz
          });

          _controller_db.update_setpoint_table(common::control_setpoint_db_data{
              -1,                            // local_time
              tracker_RTdata.setpoint(0),    // set_x
              tracker_RTdata.setpoint(1),    // set_y
              tracker_RTdata.setpoint(2),    // set_theta
              tracker_RTdata.v_setpoint(0),  // set_u
              tracker_RTdata.v_setpoint(1),  // set_v
              tracker_RTdata.v_setpoint(2)   // set_r
          });
          _controller_db.update_TA_table(common::control_TA_db_data{
              -1,                            // local_time
              controller_RTdata.tau(0),      // desired_Fx
              controller_RTdata.tau(1),      // desired_Fy
              controller_RTdata.tau(2),      // desired_Mz
              controller_RTdata.BalphaU(0),  // est_Fx
              controller_RTdata.BalphaU(1),  // est_Fy
              controller_RTdata.BalphaU(2),  // est_Mz
              std::vector<int>(controller_RTdata.command_alpha_deg.data(),
                               controller_RTdata.command_alpha_deg.data() +
                                   num_thruster),  // alpha
              std::vector<int>(controller_RTdata.command_rotation.data(),
                               controller_RTdata.command_rotation.data() +
                                   num_thruster)  // rpm
          });
          // _sqlite.update_surroundings_table(SpokeProcess_RTdata);
          break;
        }
        case common::TESTMODE::EXPERIMENT_DP:
        case common::TESTMODE::EXPERIMENT_LOS:
        case common::TESTMODE::EXPERIMENT_FRENET: {
          // experiment

          _gps_db.update_gps_table(common::gps_db_data{
              0,                   // local_time
              gps_data.UTC,        // UTC
              gps_data.latitude,   // latitude
              gps_data.longitude,  // longitude
              gps_data.heading,    // heading
              gps_data.pitch,      // pitch
              gps_data.roll,       // roll
              gps_data.altitude,   // altitude
              gps_data.Ve,         // Ve
              gps_data.Vn,         // Vn
              gps_data.roti,       // roti
              gps_data.status,     // status
              gps_data.UTM_x,      // UTM_x
              gps_data.UTM_y,      // UTM_y
              gps_data.UTM_zone    // UTM_zone
          });
          _stm32_db.update_table(common::stm32_db_data{
              0,                                        // local_time
              static_cast<int>(stm32_data.linkstatus),  // stm32_link
              static_cast<int>(
                  stm32_data.feedback_stm32status),  // stm32_status
              stm32_data.command_u1,                 // command_u1
              stm32_data.command_u2,                 // command_u2
              stm32_data.feedback_u1,                // feedback_u1
              stm32_data.feedback_u2,                // feedback_u2
              stm32_data.feedback_pwm1,              // feedback_pwm1
              stm32_data.feedback_pwm2,              // feedback_pwm2
              stm32_data.RC_X,                       // RC_X
              stm32_data.RC_Y,                       // RC_Y
              stm32_data.RC_Mz,                      // RC_Mz
              stm32_data.voltage_b1,                 // voltage_b1
              stm32_data.voltage_b2,                 // voltage_b2
              stm32_data.voltage_b3                  // voltage_b3
          });

          _estimator_db.update_measurement_table(
              common::est_measurement_db_data{
                  -1,                               // local_time
                  estimator_RTdata.Measurement(0),  // meas_x
                  estimator_RTdata.Measurement(1),  // meas_y
                  estimator_RTdata.Measurement(2),  // meas_theta
                  estimator_RTdata.Measurement(3),  // meas_u
                  estimator_RTdata.Measurement(4),  // meas_v
                  estimator_RTdata.Measurement(5)   // meas_r
              });
          _estimator_db.update_state_table(common::est_state_db_data{
              -1,                                // local_time
              estimator_RTdata.State(0),         // state_x
              estimator_RTdata.State(1),         // state_y
              estimator_RTdata.State(2),         // state_theta
              estimator_RTdata.State(3),         // state_u
              estimator_RTdata.State(4),         // state_v
              estimator_RTdata.State(5),         // state_r
              estimator_RTdata.Marine_state(3),  // curvature
              estimator_RTdata.Marine_state(4),  // speed
              estimator_RTdata.Marine_state(5)   // dspeed
          });
          _estimator_db.update_error_table(common::est_error_db_data{
              -1,                           // local_time
              estimator_RTdata.p_error(0),  // perror_x
              estimator_RTdata.p_error(1),  // perror_y
              estimator_RTdata.p_error(2),  // perror_mz
              estimator_RTdata.v_error(0),  // verror_x
              estimator_RTdata.v_error(1),  // verror_y
              estimator_RTdata.v_error(2)   // verror_mz
          });

          _controller_db.update_setpoint_table(common::control_setpoint_db_data{
              -1,                            // local_time
              tracker_RTdata.setpoint(0),    // set_x
              tracker_RTdata.setpoint(1),    // set_y
              tracker_RTdata.setpoint(2),    // set_theta
              tracker_RTdata.v_setpoint(0),  // set_u
              tracker_RTdata.v_setpoint(1),  // set_v
              tracker_RTdata.v_setpoint(2)   // set_r
          });
          _controller_db.update_TA_table(common::control_TA_db_data{
              -1,                            // local_time
              controller_RTdata.tau(0),      // desired_Fx
              controller_RTdata.tau(1),      // desired_Fy
              controller_RTdata.tau(2),      // desired_Mz
              controller_RTdata.BalphaU(0),  // est_Fx
              controller_RTdata.BalphaU(1),  // est_Fy
              controller_RTdata.BalphaU(2),  // est_Mz
              std::vector<int>(controller_RTdata.command_alpha_deg.data(),
                               controller_RTdata.command_alpha_deg.data() +
                                   num_thruster),  // alpha
              std::vector<int>(controller_RTdata.command_rotation.data(),
                               controller_RTdata.command_rotation.data() +
                                   num_thruster)  // rpm
          });

          _planner_db.update_latticeplanner_table(common::plan_lattice_db_data{
              -1,                           // local_time
              Planning_Marine_state.x,      // lattice_x
              Planning_Marine_state.y,      // lattice_y
              Planning_Marine_state.theta,  // lattice_theta
              Planning_Marine_state.kappa,  // lattice_kappa
              Planning_Marine_state.speed,  // lattice_speed
              Planning_Marine_state.dspeed  // lattice_dspeed
          });

          if (RoutePlanner_RTdata.state_toggle == common::STATETOGGLE::READY) {
            auto num_wp = RoutePlanner_RTdata.Waypoint_X.size();
            _planner_db.update_routeplanner_table(common::plan_route_db_data{
                -1,                                       // local_time
                RoutePlanner_RTdata.setpoints_X,          // setpoints_X
                RoutePlanner_RTdata.setpoints_Y,          // setpoints_Y
                RoutePlanner_RTdata.setpoints_heading,    // setpoints_heading
                RoutePlanner_RTdata.setpoints_longitude,  // setpoints_longitude
                RoutePlanner_RTdata.setpoints_latitude,   // setpoints_latitude
                RoutePlanner_RTdata.speed,                // speed
                RoutePlanner_RTdata.los_capture_radius,   // captureradius
                RoutePlanner_RTdata.utm_zone,             // utm_zone
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_X.data(),
                    RoutePlanner_RTdata.Waypoint_X.data() + num_wp),  // WPX
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_Y.data(),
                    RoutePlanner_RTdata.Waypoint_Y.data() + num_wp),  // WPY
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_longitude.data(),
                    RoutePlanner_RTdata.Waypoint_longitude.data() +
                        num_wp),  // WPLONG
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_latitude.data(),
                    RoutePlanner_RTdata.Waypoint_latitude.data() +
                        num_wp)  // WPLAT
            });
          }
          break;
        }
        case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
          _gps_db.update_gps_table(common::gps_db_data{
              0,                   // local_time
              gps_data.UTC,        // UTC
              gps_data.latitude,   // latitude
              gps_data.longitude,  // longitude
              gps_data.heading,    // heading
              gps_data.pitch,      // pitch
              gps_data.roll,       // roll
              gps_data.altitude,   // altitude
              gps_data.Ve,         // Ve
              gps_data.Vn,         // Vn
              gps_data.roti,       // roti
              gps_data.status,     // status
              gps_data.UTM_x,      // UTM_x
              gps_data.UTM_y,      // UTM_y
              gps_data.UTM_zone    // UTM_zone
          });
          _stm32_db.update_table(common::stm32_db_data{
              0,                                        // local_time
              static_cast<int>(stm32_data.linkstatus),  // stm32_link
              static_cast<int>(
                  stm32_data.feedback_stm32status),  // stm32_status
              stm32_data.command_u1,                 // command_u1
              stm32_data.command_u2,                 // command_u2
              stm32_data.feedback_u1,                // feedback_u1
              stm32_data.feedback_u2,                // feedback_u2
              stm32_data.feedback_pwm1,              // feedback_pwm1
              stm32_data.feedback_pwm2,              // feedback_pwm2
              stm32_data.RC_X,                       // RC_X
              stm32_data.RC_Y,                       // RC_Y
              stm32_data.RC_Mz,                      // RC_Mz
              stm32_data.voltage_b1,                 // voltage_b1
              stm32_data.voltage_b2,                 // voltage_b2
              stm32_data.voltage_b3                  // voltage_b3
          });
          _estimator_db.update_measurement_table(
              common::est_measurement_db_data{
                  -1,                               // local_time
                  estimator_RTdata.Measurement(0),  // meas_x
                  estimator_RTdata.Measurement(1),  // meas_y
                  estimator_RTdata.Measurement(2),  // meas_theta
                  estimator_RTdata.Measurement(3),  // meas_u
                  estimator_RTdata.Measurement(4),  // meas_v
                  estimator_RTdata.Measurement(5)   // meas_r
              });
          _estimator_db.update_state_table(common::est_state_db_data{
              -1,                                // local_time
              estimator_RTdata.State(0),         // state_x
              estimator_RTdata.State(1),         // state_y
              estimator_RTdata.State(2),         // state_theta
              estimator_RTdata.State(3),         // state_u
              estimator_RTdata.State(4),         // state_v
              estimator_RTdata.State(5),         // state_r
              estimator_RTdata.Marine_state(3),  // curvature
              estimator_RTdata.Marine_state(4),  // speed
              estimator_RTdata.Marine_state(5)   // dspeed
          });
          _estimator_db.update_error_table(common::est_error_db_data{
              -1,                           // local_time
              estimator_RTdata.p_error(0),  // perror_x
              estimator_RTdata.p_error(1),  // perror_y
              estimator_RTdata.p_error(2),  // perror_mz
              estimator_RTdata.v_error(0),  // verror_x
              estimator_RTdata.v_error(1),  // verror_y
              estimator_RTdata.v_error(2)   // verror_mz
          });

          _controller_db.update_setpoint_table(common::control_setpoint_db_data{
              -1,                            // local_time
              tracker_RTdata.setpoint(0),    // set_x
              tracker_RTdata.setpoint(1),    // set_y
              tracker_RTdata.setpoint(2),    // set_theta
              tracker_RTdata.v_setpoint(0),  // set_u
              tracker_RTdata.v_setpoint(1),  // set_v
              tracker_RTdata.v_setpoint(2)   // set_r
          });
          _controller_db.update_TA_table(common::control_TA_db_data{
              -1,                            // local_time
              controller_RTdata.tau(0),      // desired_Fx
              controller_RTdata.tau(1),      // desired_Fy
              controller_RTdata.tau(2),      // desired_Mz
              controller_RTdata.BalphaU(0),  // est_Fx
              controller_RTdata.BalphaU(1),  // est_Fy
              controller_RTdata.BalphaU(2),  // est_Mz
              std::vector<int>(controller_RTdata.command_alpha_deg.data(),
                               controller_RTdata.command_alpha_deg.data() +
                                   num_thruster),  // alpha
              std::vector<int>(controller_RTdata.command_rotation.data(),
                               controller_RTdata.command_rotation.data() +
                                   num_thruster)  // rpm
          });

          _planner_db.update_latticeplanner_table(common::plan_lattice_db_data{
              -1,                           // local_time
              Planning_Marine_state.x,      // lattice_x
              Planning_Marine_state.y,      // lattice_y
              Planning_Marine_state.theta,  // lattice_theta
              Planning_Marine_state.kappa,  // lattice_kappa
              Planning_Marine_state.speed,  // lattice_speed
              Planning_Marine_state.dspeed  // lattice_dspeed
          });

          if (RoutePlanner_RTdata.state_toggle == common::STATETOGGLE::READY) {
            auto num_wp = RoutePlanner_RTdata.Waypoint_X.size();
            _planner_db.update_routeplanner_table(common::plan_route_db_data{
                -1,                                       // local_time
                RoutePlanner_RTdata.setpoints_X,          // setpoints_X
                RoutePlanner_RTdata.setpoints_Y,          // setpoints_Y
                RoutePlanner_RTdata.setpoints_heading,    // setpoints_heading
                RoutePlanner_RTdata.setpoints_longitude,  // setpoints_longitude
                RoutePlanner_RTdata.setpoints_latitude,   // setpoints_latitude
                RoutePlanner_RTdata.speed,                // speed
                RoutePlanner_RTdata.los_capture_radius,   // captureradius
                RoutePlanner_RTdata.utm_zone,             // utm_zone
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_X.data(),
                    RoutePlanner_RTdata.Waypoint_X.data() + num_wp),  // WPX
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_Y.data(),
                    RoutePlanner_RTdata.Waypoint_Y.data() + num_wp),  // WPY
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_longitude.data(),
                    RoutePlanner_RTdata.Waypoint_longitude.data() +
                        num_wp),  // WPLONG
                std::vector<double>(
                    RoutePlanner_RTdata.Waypoint_latitude.data(),
                    RoutePlanner_RTdata.Waypoint_latitude.data() +
                        num_wp)  // WPLAT
            });
          }

          std::size_t size_spokedata = sizeof(MarineRadar_RTdata.spokedata) /
                                       sizeof(MarineRadar_RTdata.spokedata[0]);
          _marineradar_db.update_table(common::marineradar_db_data{
              0,                                       // local_time
              MarineRadar_RTdata.spoke_azimuth_deg,    // azimuth_deg
              MarineRadar_RTdata.spoke_samplerange_m,  // sample_range
              std::vector<uint8_t>(
                  &MarineRadar_RTdata.spokedata[0],
                  &MarineRadar_RTdata.spokedata[size_spokedata])  // spokedata
          });

          if (TargetTracker_RTdata.spoke_state ==
              ASV::perception::SPOKESTATE::LEAVE_ALARM_ZONE) {
            _perception_db.update_spoke_table(common::perception_spoke_db_data{
                -1,  // local_time
                SpokeProcess_RTdata
                    .surroundings_bearing_rad,  // surroundings_bearing_rad
                SpokeProcess_RTdata
                    .surroundings_range_m,             // surroundings_range_m
                SpokeProcess_RTdata.surroundings_x_m,  // surroundings_x_m
                SpokeProcess_RTdata.surroundings_y_m   // surroundings_y_m
            });
            _perception_db.update_detection_table(
                common::perception_detection_db_data{
                    -1,                               // local_time
                    TargetDetection_RTdata.target_x,  // detected_target_x
                    TargetDetection_RTdata.target_y,  // detected_target_y
                    TargetDetection_RTdata
                        .target_square_radius  // detected_target_radius
                });
            _perception_db.update_trackingtarget_table(
                common::perception_trackingtarget_db_data{
                    -1,  // local_time
                    static_cast<int>(
                        TargetTracker_RTdata.spoke_state),  // spoke_state
                    std::vector<int>(TargetTracker_RTdata.targets_state.data(),
                                     TargetTracker_RTdata.targets_state.data() +
                                         max_num_targets),  // targets_state
                    std::vector<int>(
                        TargetTracker_RTdata.targets_intention.data(),
                        TargetTracker_RTdata.targets_intention.data() +
                            max_num_targets),  // targets_intention
                    std::vector<double>(TargetTracker_RTdata.targets_x.data(),
                                        TargetTracker_RTdata.targets_x.data() +
                                            max_num_targets),  // targets_x
                    std::vector<double>(TargetTracker_RTdata.targets_y.data(),
                                        TargetTracker_RTdata.targets_y.data() +
                                            max_num_targets),  // targets_y
                    std::vector<double>(
                        TargetTracker_RTdata.targets_square_radius.data(),
                        TargetTracker_RTdata.targets_square_radius.data() +
                            max_num_targets),  // targets_square_radius
                    std::vector<double>(TargetTracker_RTdata.targets_vx.data(),
                                        TargetTracker_RTdata.targets_vx.data() +
                                            max_num_targets),  // targets_vx
                    std::vector<double>(TargetTracker_RTdata.targets_vy.data(),
                                        TargetTracker_RTdata.targets_vy.data() +
                                            max_num_targets),  // targets_vy
                    std::vector<double>(
                        TargetTracker_RTdata.targets_CPA_x.data(),
                        TargetTracker_RTdata.targets_CPA_x.data() +
                            max_num_targets),  // targets_CPA_x
                    std::vector<double>(
                        TargetTracker_RTdata.targets_CPA_y.data(),
                        TargetTracker_RTdata.targets_CPA_y.data() +
                            max_num_targets),  // targets_CPA_y
                    std::vector<double>(
                        TargetTracker_RTdata.targets_TCPA.data(),
                        TargetTracker_RTdata.targets_TCPA.data() +
                            max_num_targets)  // targets_TCPA
                });
          }

          break;
        }
        default:
          break;
      }  // end switch
    }
  }  // sqlloop()

  //##################### STM32 ########################//
  void stm32loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET:
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        // experiment
        messages::stm32_link _stm32_link(stm32_data,
                                         _jsonparse.getstm32baudrate(),
                                         _jsonparse.getstm32port());
        while (1) {
          messages::STM32STATUS _command_stm32 =
              static_cast<messages::STM32STATUS>(
                  guilink_RTdata.guistutus_gui2PC);
          _stm32_link
              .setstm32data(_command_stm32, pt_utc, controller_RTdata.command_u,
                            controller_RTdata.command_alpha)
              .stm32onestep();
          stm32_data = _stm32_link.getstmdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // stm32loop()

  //################### GPS sensor and UTM conversion ######################//
  void gps_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET:
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        messages::GPS _gpsimu(_jsonparse.getgpsbaudrate(),
                              _jsonparse.getgpsport());

        // experiment
        while (1) {
          gps_data =
              _gpsimu.parseGPS(RoutePlanner_RTdata.utm_zone).getgpsRTdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // gps_loop()

  //##################### marine radar and spoke messages ####################//
  void marine_radar_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE:
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        messages::MarineRadar Marine_Radar;
        Marine_Radar.StartMarineRadar();
        // experiment
        while (1) {
          MarineRadar_RTdata = Marine_Radar.getMarineRadarRTdata();
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        break;
      }
      default:
        break;
    }  // end switch

  }  // marine_radar_loop

  //################### GUI data link ######################//
  void gui_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        // simulation: do nothing
        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET:
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        messages::guilink_serial<num_thruster, 3, dim_controlspace> _gui_link(
            guilink_RTdata, _jsonparse.getguibaudrate(),
            _jsonparse.getguiport());

        // experiment
        while (1) {
          Eigen::Vector3d batteries =
              (Eigen::Vector3d() << stm32_data.voltage_b1,
               stm32_data.voltage_b2, stm32_data.voltage_b3)
                  .finished();
          Eigen::Vector2i feedback_pwm =
              (Eigen::Vector2i() << stm32_data.feedback_pwm1,
               stm32_data.feedback_pwm2)
                  .finished();

          messages::GUISTATUS _guistutus_PC2gui =
              static_cast<messages::GUISTATUS>(stm32_data.feedback_stm32status);
          _gui_link
              .setguilinkRTdata(_guistutus_PC2gui, gps_data.latitude,
                                gps_data.longitude,
                                estimator_RTdata.Measurement_6dof(3),
                                estimator_RTdata.Measurement_6dof(4),
                                estimator_RTdata.State, feedback_pwm, batteries)
              .guicommunication();
          guilink_RTdata = _gui_link.getguilinkRTdata();
        }

        break;
      }
      default:
        break;
    }  // end switch

  }  // gui_loop

  //################### UTC clock ######################//
  void utc_timer_loop() {
    common::timecounter utc_timer;

    while (1) {
      pt_utc = utc_timer.getUTCtime();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }  // utc_timer_loop

  //################### state monitor ######################//
  void state_monitor_loop() {
    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        while (1) {
          if ((StateMonitor::indicator_routeplanner ==
               common::STATETOGGLE::IDLE) &&
              (RoutePlanner_RTdata.state_toggle ==
               common::STATETOGGLE::READY)) {
            StateMonitor::indicator_routeplanner = common::STATETOGGLE::READY;
            CLOG(INFO, "route-planner") << "initialation successful!";
          }
          if (StateMonitor::indicator_estimator == common::STATETOGGLE::IDLE) {
            StateMonitor::indicator_estimator = common::STATETOGGLE::READY;
            CLOG(INFO, "estimator") << "initialation successful!";
          }

          if ((StateMonitor::indicator_pathplanner ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_controller ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_estimator ==
               common::STATETOGGLE::READY)) {
            CLOG(INFO, "path-planner") << "initialation successful!";
            CLOG(INFO, "controller") << "initialation successful!";
            StateMonitor::indicator_pathplanner = common::STATETOGGLE::READY;
            StateMonitor::indicator_controller = common::STATETOGGLE::READY;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET: {
        // experiment
        while (1) {
          if ((StateMonitor::indicator_routeplanner ==
               common::STATETOGGLE::IDLE) &&
              (RoutePlanner_RTdata.state_toggle ==
               common::STATETOGGLE::READY)) {
            StateMonitor::indicator_routeplanner = common::STATETOGGLE::READY;
            CLOG(INFO, "route-planner") << "initialation successful!";
          }

          if ((gps_data.status >= 1) &&
              (StateMonitor::indicator_gps == common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_routeplanner ==
               common::STATETOGGLE::READY)) {
            StateMonitor::indicator_gps = common::STATETOGGLE::READY;
            CLOG(INFO, "GPS") << "initialation successful!";
          }

          if ((StateMonitor::indicator_gps == common::STATETOGGLE::READY) &&
              (StateMonitor::indicator_estimator ==
               common::STATETOGGLE::IDLE)) {
            StateMonitor::indicator_estimator = common::STATETOGGLE::READY;
            CLOG(INFO, "estimator") << "initialation successful!";
          }

          if ((StateMonitor::indicator_pathplanner ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_controller ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_estimator ==
               common::STATETOGGLE::READY)) {
            CLOG(INFO, "path-planner") << "initialation successful!";
            CLOG(INFO, "controller") << "initialation successful!";
            StateMonitor::indicator_pathplanner = common::STATETOGGLE::READY;
            StateMonitor::indicator_controller = common::STATETOGGLE::READY;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        break;
      }
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        while (1) {
          if ((StateMonitor::indicator_routeplanner ==
               common::STATETOGGLE::IDLE) &&
              (RoutePlanner_RTdata.state_toggle ==
               common::STATETOGGLE::READY)) {
            StateMonitor::indicator_routeplanner = common::STATETOGGLE::READY;
            CLOG(INFO, "route-planner") << "initialation successful!";
          }
          if ((gps_data.status >= 1) &&
              (StateMonitor::indicator_gps == common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_routeplanner ==
               common::STATETOGGLE::READY)) {
            StateMonitor::indicator_gps = common::STATETOGGLE::READY;
            CLOG(INFO, "GPS") << "initialation successful!";
          }

          if ((StateMonitor::indicator_gps == common::STATETOGGLE::READY) &&
              (StateMonitor::indicator_estimator ==
               common::STATETOGGLE::IDLE)) {
            StateMonitor::indicator_estimator = common::STATETOGGLE::READY;
            CLOG(INFO, "estimator") << "initialation successful!";
          }

          if ((StateMonitor::indicator_marine_radar ==
               common::STATETOGGLE::IDLE) &&
              (MarineRadar_RTdata.state_toggle == common::STATETOGGLE::READY)) {
            StateMonitor::indicator_marine_radar = common::STATETOGGLE::READY;
            CLOG(INFO, "marine-radar") << "initialation successful!";
          }

          if ((StateMonitor::indicator_estimator ==
               common::STATETOGGLE::READY) &&
              (StateMonitor::indicator_marine_radar ==
               common::STATETOGGLE::READY) &&
              (StateMonitor::indicator_target_tracking ==
               common::STATETOGGLE::IDLE)) {
            StateMonitor::indicator_target_tracking =
                common::STATETOGGLE::READY;
            CLOG(INFO, "target-tracking") << "initialation successful!";
          }

          if ((StateMonitor::indicator_pathplanner ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_controller ==
               common::STATETOGGLE::IDLE) &&
              (StateMonitor::indicator_target_tracking ==
               common::STATETOGGLE::READY)) {
            CLOG(INFO, "path-planner") << "initialation successful!";
            CLOG(INFO, "controller") << "initialation successful!";
            StateMonitor::indicator_pathplanner = common::STATETOGGLE::READY;
            StateMonitor::indicator_controller = common::STATETOGGLE::READY;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        break;
      }
      default:
        break;
    }

  }  // state_monitor_loop

  //################### socket TCP server ######################//
  void socket_loop() {
    tcpserver _tcpserver("9340");

    switch (testmode) {
      case common::TESTMODE::SIMULATION_DP:
      case common::TESTMODE::SIMULATION_LOS:
      case common::TESTMODE::SIMULATION_FRENET:
      case common::TESTMODE::SIMULATION_AVOIDANCE: {
        union socketmsg {
          double double_msg[20];
          char char_msg[160];
        };

        const int recv_size = 10;
        const int send_size = 160;
        char recv_buffer[recv_size];
        socketmsg _sendmsg = {0.0, 0.0, 0.0, 0.0, 0.0};

        common::timecounter timer_socket;
        long int outerloop_elapsed_time = 0;
        long int innerloop_elapsed_time = 0;
        long int sample_time = 100;

        while (1) {
          outerloop_elapsed_time = timer_socket.timeelapsed();

          for (int i = 0; i != 6; ++i)
            _sendmsg.double_msg[i] = estimator_RTdata.State(i);  // State

          _sendmsg.double_msg[6] =
              RoutePlanner_RTdata.los_capture_radius;          // curvature
          _sendmsg.double_msg[7] = RoutePlanner_RTdata.speed;  // speed
          _sendmsg.double_msg[8] =
              RoutePlanner_RTdata.setpoints_X;  // waypoint0
          _sendmsg.double_msg[9] =
              RoutePlanner_RTdata.setpoints_Y;  // waypoint0
          _sendmsg.double_msg[10] =
              RoutePlanner_RTdata.setpoints_heading;  // waypoint1
          _sendmsg.double_msg[11] =
              RoutePlanner_RTdata.setpoints_longitude;  // waypoint1

          for (int i = 0; i != dim_controlspace; ++i) {
            _sendmsg.double_msg[12 + i] = controller_RTdata.tau(i);  // tau
          }
          for (int i = 0; i != num_thruster; ++i) {
            _sendmsg.double_msg[12 + dim_controlspace + i] =
                controller_RTdata.command_rotation(i);  // rotation
          }
          _tcpserver.selectserver(recv_buffer, _sendmsg.char_msg, recv_size,
                                  send_size);

          innerloop_elapsed_time = timer_socket.timeelapsed();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(sample_time - innerloop_elapsed_time));

          if (outerloop_elapsed_time > 1.1 * sample_time)
            CLOG(INFO, "socket") << "Too much time!";
        }

        break;
      }
      case common::TESTMODE::EXPERIMENT_DP:
      case common::TESTMODE::EXPERIMENT_LOS:
      case common::TESTMODE::EXPERIMENT_FRENET:
      case common::TESTMODE::EXPERIMENT_AVOIDANCE: {
        // experiment: do nothing
        break;
      }
      default:
        break;
    }  // end switch

  }  // socketloop()

};  // namespace ASV

}  // end namespace ASV

#endif /* _THREADLOOP_H_ */