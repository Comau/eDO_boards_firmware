/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/master_package/MasterNode.hpp>

#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>


#include<math.h>

enum INIT_MODE {
	DISCOVERY_MODE = 0,  // 0
	SET_MODE,            // 1
	CANCEL_MODE,         // 2
	THRESHOLD_MODE,      // 3 mod threshold for joint collision
	MAXVEL_MODE,         // 4 mod factor for max velocity limit
	FLLERR_MODE          // 5 mod factor for following error bound
};

// --- MODULE -----------------------------------------------------------------

extern Module module;

// --- NODE -----------------------------------------------------------------

namespace core {
namespace master_package {
MasterNode::MasterNode(
    const char*                        			name,
    core::control_pid::ControlNode&    			controller,
    core::interpolation_package::InterpNode& 	interpolator,
    os::Thread::Priority               			priority
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable::CoreConfigurable(name),
	_identity(-1),
	_reduction(0.0f),
	_controller(controller),
	_interpolator(interpolator)
{
    _workingAreaSize = 1024;
}

MasterNode::~MasterNode()
{
    teardown();
}

bool
MasterNode::onInitialize()
{    
	bool success = true;
    return success;
}

bool
MasterNode::onConfigure()
{    
	bool success=true;

	_identity=configuration().Configured;
	_reduction=configuration().Reduction;

    return success;
}

bool
MasterNode::onPrepareHW()
{
    return true;
}

bool
MasterNode::onPrepareMW()
{
	_subscriber_init.set_callback(MasterNode::init_callback);
	this->subscribe(_subscriber_init , "joint_init");

	_subscriber_calibration.set_callback(MasterNode::calibration_callback);
	this->subscribe(_subscriber_calibration , "joint_calib");

	_subscriber_reset.set_callback(MasterNode::reset_callback);
	this->subscribe(_subscriber_reset , "joint_reset");

	_subscriber_version.set_callback(MasterNode::version_callback);
	this->subscribe(_subscriber_version , "joint_version");

	if (_identity > 0 ) {
		
		//PID configuration
		_controller.config_unsafe(	7.5,		//kp
									0.0,		//ti
									0.0, 	//td
									1.0, 	//ts
									-4902.0,	//min
									4902.0, 	//max
									10000000.0, 	//sat
									0, 			//kff
									0.01, 		//kpv
									0.005, 		//tiv
									0.0, 	//tdv
									1.0, 	//tsv
									-12.0, 	//minv
									12.0, 	//maxv
									100000.0, 	//satv
									0.0, 		//kfv
									0.0,		//kpt
									0.0,	//tit
									0.0,	//tdt
									1.0,	//tst
									-12.0,		//mint
									12.0,		//maxt
									100000000,	//satt
									0.0);		//kft
		
		_controller.communication_setup(_identity, _reduction);

		_interpolator.communication_setup(_identity);

	}
    return true;
}

bool
MasterNode::onStart()
{
    bool success = true;
    return success;
}

bool
MasterNode::onLoop()
{
	if (!this->spin(core::os::Time::ms(100))) {
		// No recovery action
	}
	
    return true;
} // MasterNode::onLoop

bool
MasterNode::onStop()
{
    bool success = true;
    return success;
}

bool
MasterNode::init_callback(
		const comau_edo::edo_msgs::EdoJointInit& msg,
		void*                                  context
)
{
	MasterNode* _this = static_cast<MasterNode*>(context);
	uint32_t    jmask = (uint32_t)msg.joints_mask; // From U Long Long to U Long

	switch (msg.mode)
	{
		case (DISCOVERY_MODE):
		{  // Discovery ID
			if ((_this->_identity > 0) &&
					(jmask & (1uL << (_this->_identity - 1))))
			{
				_this->_controller.setack(J_STATE_ACK_INIT);
			}
			break;
		}

		case (SET_MODE):
		{  // Set ID
			if (_this->_identity <= 0 )
			{
				//Search the joint ID
				for(uint8_t i = 1; jmask != 0; i++, jmask >>= 1)
				{
					if (jmask & 1uL)
					{
						// Found!
						_this->_identity = i;
						_this->_reduction = msg.reduction;

						//Configue the PID
						_this->_controller.config_unsafe(	7.5,		//kp
															0.0,		//ti
															0.0, 	//td
															1.0, 	//ts
															-4902.0,	//min
															4902.0,		//max
															10000000.0,	//sat
															0, 			//kff
															0.01, 		//kpv
															0.005, 		//tiv
															0.0, 	//tdv
															1.0, 	//tsv
															-12.0,		//minv
															12.0,		//maxv
															100000.0, 	//satv
															0.0, 		//kfv
															0.0,		//kpt
															0.0,	//tit
															0.0,	//tdt
															1.0,	//tst
															-12.0,		//mint
															12.0,		//maxt
															100000000,	//satt
															3.0);		//kft

						_this->_controller.communication_setup(_this->_identity, _this->_reduction);
						_this->_interpolator.communication_setup(_this->_identity);

						_this->overrideConfiguration();
						_this->overridingConfiguration().Configured = _this->_identity;
						_this->overridingConfiguration().Reduction  = _this->_reduction;

						module.configurations().saveTo(module.configurationStorage());   // This will take a while, about 30 ms

						_this->_controller.setack(J_STATE_ACK_INIT);
						break;
					}
				}
			}
			break;
		}

		case (CANCEL_MODE):
		{  // Cancel ID
			if ((_this->_identity > 0) &&
				(jmask & (1uL << (_this->_identity - 1))))
			{
				//Delete just the ID into the flash.
				//On next startup the joint will be not configured.
				_this->overrideConfiguration();
				_this->overridingConfiguration().Configured = -1;
				_this->overridingConfiguration().Reduction = 0.0f;

				module.configurations().saveTo(module.configurationStorage());  // This will take a while, about 30 ms

				_this->_controller.setack(J_STATE_ACK_INIT);
			}
			break;
		}
		case (THRESHOLD_MODE):
		{  // Set Threshold
		
			if ((_this->_identity > 0) &&
				(jmask & (1uL << 6))) 
			{
				_this->_controller.set_doublecheck_threshold(msg.reduction);
			}
			
			if ((_this->_identity > 0) &&
				(jmask & (1uL << (_this->_identity - 1)))) 
			{
				_this->_controller.set_coll_threshold(msg.reduction);
				_this->_controller.setack(J_STATE_ACK_INIT);
			}

			break;
		}
		case (MAXVEL_MODE):
		{  // Set multiplying factor for Max Velocity
			if ((_this->_identity > 0) &&
				(jmask & (1uL << (_this->_identity - 1)))) 
			{
				_this->_controller.set_maxvel_threshold(msg.reduction);
				
				_this->_controller.setack(J_STATE_ACK_INIT);
			}

			break;
		}
		case (FLLERR_MODE):
		{  // Set multiplying factor for Following Error
			if ((_this->_identity > 0) &&
				(jmask & (1uL << (_this->_identity - 1)))) 
			{
				_this->_controller.set_fllerr_threshold(msg.reduction);
				
				_this->_controller.setack(J_STATE_ACK_INIT);
			}

			break;
		}
	}

	return true;
}



bool
MasterNode::reset_callback(
		const comau_edo::edo_msgs::EdoJointReset& msg,
		void*                                  context
)
{
	MasterNode* _this = static_cast<MasterNode*>(context);

	if ((_this->_identity > 0) &&
		(msg.joints_mask & (1uL << (_this->_identity - 1)))) 
	{
		_this->_controller.disengage_brake(msg.disengage_steps, msg.disengage_offset);

		_this->_controller.setack(J_STATE_ACK_RESET);
	}

	return true;

}

bool
MasterNode::calibration_callback(
		const comau_edo::edo_msgs::EdoJointCalibration& msg,
		void*                                  context
)
{
	 MasterNode* _this = static_cast<MasterNode*>(context);

	if (_this->_identity > 0)
	{	
		if(msg.joints_mask & (1uL << (_this->_identity - 1))) 
		{

		_this->_controller.pos_calibration();
		_this->_interpolator.pos_calibration();

		_this->_controller.setack(J_STATE_ACK_CALIB);
		}
		
		if(msg.joints_mask & 0x20) // Only if is the 6 axes node is calibrated (0x20=32=00001)
		{
		_this->_controller.current_calibration();
		}
	}

	return true;

}

bool
MasterNode::version_callback(
		const comau_edo::edo_msgs::EdoJointVersion& msg,
		void*                                       context
)
{
	MasterNode* _this = static_cast<MasterNode*>(context);
	if (msg.id == _this->_identity)
	{
		_this->_controller.setack(J_STATE_ACK_VERSION);
		return _this->_controller.version_publish(_this->_identity);
	}

	return false;

}

}
}


