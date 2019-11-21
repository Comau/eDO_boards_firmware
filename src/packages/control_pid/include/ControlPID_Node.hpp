/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */



#pragma once
// --- INCLUDES -----------------------------------------------------------------
#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <core/utils/BasicSensor.hpp>
#include <core/utils/BasicActuator.hpp>

#include <ModuleConfiguration.hpp>

#include <core/control_pid/ControlPIDNodeConfiguration.hpp>
#include <core/control_msgs/Ctrlpnt_f32.hpp>
#include <core/control_msgs/PID_param.hpp>
#include <core/common_msgs/Float32.hpp>
#include <core/control_msgs/Encoder_State.hpp>
#include <core/control_pid/PID2.hpp>
#include <core/control_pid/Disengage.hpp>
#include <comau_edo/edo_msgs/EdoJointVersion.hpp>

// Nibbe basso: codifica su 4 bits
#define J_STATE_ACK_INIT      1
#define J_STATE_ACK_CALIB     2
#define J_STATE_ACK_CONFIG    3
#define J_STATE_ACK_RESET     4
#define J_STATE_ACK_VERSION   5
#define J_STATE_ACK_MASK      0x0F
// Nibble alto: viene invece gestito a singolo bit
#define J_STATE_COLLISION     4  // Bit XXXY.ZZZZ Not used
#define J_STATE_UNDERVOLTAGE  5  // Bit XXYX.ZZZZ
#define J_STATE_OVERCURRENT   6  // Bit XYXX.ZZZZ
#define J_STATE_UNCALIB       7  // Bit YXXX.ZZZZ

#define J_STATE_ACK_CYCLES    30
#define PI                    3.14159265358979f
#define PLANE_ANGLE           180.0f
#define DPOSEPS               352942.00f  // [Rad_Mot]  
#define TH_CUR                0.01f       // [A] 
#define TH_ERRCNT             50         
#define TH_POS                352942.00f  // [Rad_Mot]    // [Rad] = 5 [Deg_Link]


#define CONTROL_DEFAULT_PARAMETERS \
          7.5,        /* kp   */ \
          0.0,        /* ti   */ \
          0.0,        /* td   */ \
          1.0,        /* ts   */ \
         -4902.0,     /* min  */ \
          4902.0,     /* max  */ \
          10000000.0, /* sat  */ \
          0,          /* kff  */ \
          0.01,       /* kpv  */ \
          0.005,      /* tiv  */ \
          0.0,        /* tdv  */ \
          1.0,        /* tsv  */ \
         -12.0,       /* minv */ \
          12.0,       /* maxv */ \
          100000.0,   /* satv */ \
          0.0,        /* kfv  */ \
          0.0,        /* kpt  */ \
          0.0,        /* tit  */ \
          0.0,        /* tdt  */ \
          1.0,        /* tst  */ \
         -12.0,       /* mint */ \
          12.0,       /* maxt */ \
          100000000,  /* satt */ \
          3.0         /* kft  */

// --- DEFINITION -------------------------------------------------------------
namespace core {
	
namespace control_pid{
class ControlNode:
    public mw::CoreNode,
    public mw::CoreConfigurable<ControlPIDNodeConfiguration>,
    public PID2,
	public Disengage
{
public:
    ControlNode(
        const char*                        name,
        core::utils::BasicSensor<float>&   encoder,
        core::utils::BasicActuator<float>& motor,
        os::Thread::Priority               priority = os::Thread::PriorityEnum::NORMAL
    );

    bool communication_setup(const int ID, const float red_ratio);
    bool version_publish(const int &id);
    void ISR_updateCurrent( float new_sample );
    void ISR_updateVoltage( float new_sample );
    void setack(const uint8_t flag);
    void setStatusMask(const uint8_t bit, const bool set);
    void disengage_brake(const uint32_t & time, const float & offset);
    void pos_calibration(void);
	void current_calibration(void);
	void set_coll_threshold(float coll_limit);
	void set_doublecheck_threshold(float diff_limit);
    virtual ~ControlNode();


private:

    bool onInitialize();
    bool onConfigure();
    bool onPrepareHW();
    bool onPrepareMW();
    bool onStart();
    bool onLoop();
    bool onStop();
    static bool setpoint_callback( const core::control_msgs::Ctrlpnt_f32& msg, void* context );
    static bool parameters_callback( const core::control_msgs::PID_param& msg, void* context );
    float filterCurrent();
    float filterVoltage();
	unsigned int safetyCheck( float _misFiltered, bool _fllerr, bool _fllerr_power );
	void engageElectricBrakes(void);
	void disengageElectricBrakes(void);
    //Devices
    core::utils::BasicSensor<float>&   _encoder;
    core::utils::BasicActuator<float>& _motor;

    //Communication channels
    mw::Subscriber<core::control_msgs::Ctrlpnt_f32, 2> _subscriber_setpoint;
    mw::Publisher<core::control_msgs::Encoder_State>   _encoder_publisher;
    mw::Subscriber<core::control_msgs::PID_param, 2>   _subscriber_parameters;
    mw::Publisher<comau_edo::edo_msgs::EdoJointVersion>   _version_publisher;

    //Internal variables
    float   _target;								//Angular position target
    float   _accpos;								//Accumultade angular position
    float   _calibpos;							//Angular position calibration offset
    float   _calibtarget;							//Target position calibration offset
    float   _ff_vel;								//Velocity feedforward
    float   _current;                             //Dynamic Model Current
    float   _ff_torque;							//Torque feedforward
    float   _currvel;								//Motor speed
	int     _ErrorCheckSafe;                      // Safety variable
    int     _ackindex;							//
	int     _identity;
    uint8_t _ack;								//Acknowledge/Errors
    uint8_t _jnt_err;							//Joint warnings/errors

    char _topic_contr[16];						//Topic name to receive the setpoints
    char _topic_enc[16];						//Topic name to publish the joint state
    char _topic_param[16];						//Topic name to receive PID parameters
    char _topic_version[16];					//Topic name to publish firmware version

    core::os::Time _last_encoder_timestamp;
    core::os::Time _last_encoder_measure;
    core::os::Time _last_supply_drop_timestamp;
    core::os::Time _ms5_timestamp;
    core::os::Time _ms10_timestamp;
    core::os::Time _ms500_timestamp;
    core::os::Time _s10_timestamp;
    core::os::Time _infinite_timestamp;

    float    _reduction_ratio;
    float    _accumulator_A;
    uint32_t _counter_A;
    float    _accumulator_V;
    uint32_t _counter_V;
    uint32_t _curr_alert_cntr;
    float    _motor_current;
    float    _motor_supply;
    bool     _comm_ready;
    uint32_t _calibrat_cntr;
    float    _current_offset;
    bool     _pos_calibration;
    bool     _current_calibration;
    bool     _current_calibration_completed;
    float    _target_predisengage;
    float    _current_limit;
	float    _res_diff_limit;
 
      //---------SAFETY CHECK -------//
    bool     _fllerr;
    bool     _fllerr_power;
	bool     _velerr;
	bool 	 _set_Enable;
    float    _misFiltered;
	bool 	 _savepos_flag;
};
}
}
