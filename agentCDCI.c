#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>


#define UNCOMMITTED 0
#define OPT_BLUE 1
#define OPT_RED 2

#define	BEACON 77
#define	AGENT 21

/* Enum for different motion types */
typedef enum {
	STOP = 0,
	FORWARD,
	TURN_LEFT,
	TURN_RIGHT,
} motion_t;

/* Enum for boolean flags */
typedef enum {
	false = 0,
	true = 1,
} bool;

/* Flag for successful message sent */
bool message_sent = false;

/* Flag for decision to broadcast a message */
bool broadcast_msg = false;

/* current motion type */
motion_t current_motion_type = STOP;

/* current commitment */
uint8_t my_commitment;

/* option quality in range [0,100] (unit digit used as a decimal, i.e., v=v*0.1) */
uint8_t option_quality;

/* counters for motion, turning, broadcasting and status-update */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t max_straight_ticks = 300;
const uint32_t broadcast_ticks = 32;
uint32_t last_motion_ticks = 0;
uint32_t last_broadcast_ticks = 0;
const uint32_t update_ticks = 400; /* setting how often performing the commitment update. a tick here is every ~31ms */
uint32_t last_update_ticks = 0;

double sigmaConstant = 0.3;
const double scaling = 0.008;
double timeScaling;

/* Variables for outgoing messages */
message_t message;

/* Variables for incoming messages */
uint8_t received_option;
uint8_t received_quality;
bool received_message;
uint8_t discovered_option;
uint8_t discovered_quality;
bool discovered;


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
	if( current_motion_type != new_motion_type ){
		int calibrated = true;
		switch( new_motion_type ) {
		case FORWARD:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_straight_left,kilo_straight_right);
			else
				set_motors(67,67);
			break;
		case TURN_LEFT:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_turn_left,0);
			else
				set_motors(70,0);
			break;
		case TURN_RIGHT:
			spinup_motors();
			if (calibrated)
				set_motors(0,kilo_turn_right);
			else
				set_motors(0,70);
			break;
		case STOP:
		default:
			set_motors(0,0);
		}
		current_motion_type = new_motion_type;
	}
}

/*-------------------------------------------------------------------*/
/* Function for setting the the new commitment state                 */
/* (including LED colour and message initialisation)                 */
/*-------------------------------------------------------------------*/
void set_commitment( uint8_t new_commitment_state, uint8_t new_quality ) {
	/* update the commitment state varieable */
	my_commitment = new_commitment_state;
	option_quality = new_quality;
	switch (new_commitment_state){
	case UNCOMMITTED:
		/* set the LED colour to Green */
		set_color(RGB(0,2,0));
		break;
	case OPT_BLUE:
		/* set the LED colour to Blue */
		set_color(RGB(0,0,2));
		break;
	case OPT_RED:
		/* set the LED colour to Red */
		set_color(RGB(2,0,0));
		break;
	}
	/* Initialise the message variable */
	message.data[0] = AGENT;
	message.data[1] = my_commitment;
	message.data[2] = option_quality;
	message.type    = NORMAL;
	message.crc     = message_crc(&message);
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	/* Initialise commitment and LED */
	set_commitment(UNCOMMITTED, 0);

	/* Initialise motors */
	set_motors(0,0);

	/* Initialise random seed */
	uint8_t seed = rand_hard();
	rand_seed(seed);
	seed = rand_hard();
	srand(seed);

	/* Initialise motion variables */
	set_motion( FORWARD );
	last_motion_ticks = rand_soft() % max_straight_ticks + 1;

	/* Initialise broadcast variables */
	last_broadcast_ticks = rand_soft() % broadcast_ticks + 1;

	/* Initialise the scaling factor */
	timeScaling = scaling * 0.031 * update_ticks;

	/* Initialise received message variables */
	received_message = false;
	received_option = UNCOMMITTED;
	received_quality = 0;
	discovered = false;
	discovered_option = UNCOMMITTED;
	discovered_quality = 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
	uint8_t received_type = msg->data[0];
	if (received_type == AGENT) {
		received_option = msg->data[1];
		received_quality = msg->data[2];
		received_message = true;
	} else {
		discovered_option = msg->data[1];
		discovered_quality = msg->data[2];
		discovered = true;
	}
}

/*--------------------------------------------------------------------------*/
/* Function to normalise the quality from range [0,100] to range [0,255]    */
/*--------------------------------------------------------------------------*/
uint8_t normaliseQuality(uint8_t quality){
	double norm_quality = quality * 2.56;
	uint8_t norm_quality_ui = (uint8_t)(norm_quality);
	if (norm_quality > 255){ norm_quality_ui = 255; } //special case for quality = 100.
	return norm_quality_ui;
}

/*--------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message) */
/*--------------------------------------------------------------------------*/
void update_commitment( ) {

	/* Updating the commitment only each update_ticks */
	if( kilo_ticks > last_update_ticks + update_ticks ) {
		last_update_ticks = kilo_ticks;
		/* drawing a random number */
		int randomInt = RAND_MAX;
		while (randomInt > 30000){
			randomInt = rand();
		}
		unsigned int RANGE_RND = 10000;
		unsigned int random = randomInt % RANGE_RND + 1;

		/* if the agent is uncommitted, it can do discovery or recruitment */
		if (my_commitment == UNCOMMITTED){

			double P_discovery;
			double P_recruitment;

			/* compute the transition probabilities as a fucntion of the estimated qualities */
			/* discovery is only possible if the message is received from a BEACON robot */
			if (discovered){
				P_discovery = timeScaling * discovered_quality / 10.0;
			} else {
				P_discovery = 0;
			}
			/* recruitment is only possible if the message is received from (i) an AGENT robot (ii) committed to an option */
			if (received_message && received_option != UNCOMMITTED) {
				P_recruitment = timeScaling * received_quality / 10.0;
			} else {
				P_recruitment = 0;
			}

			unsigned int P_discoveryInt = (unsigned int)(P_discovery*RANGE_RND)+1;
			unsigned int P_recruitmentInt = (unsigned int)(P_recruitment*RANGE_RND)+1;


			/* DISCOVERY */
			if (P_discoveryInt > 0 && random <= P_discoveryInt){
				/* the agent discovers a new option */
				set_commitment(discovered_option, discovered_quality);
			}

			/* RECRUITMENT*/
			else if (P_recruitmentInt > 0 && random <= (P_discoveryInt + P_recruitmentInt) ){
				/* the agent discovers a new option */
				set_commitment(received_option, received_quality);
			}
		}
		/* if the agent is committed */
		else {
			double P_abandonment;
			double P_inhibition;

			/* compute the transition probabilities as a fucntion of the estimated qualities */
			P_abandonment = timeScaling / (option_quality / 10);
			/* I get inhibited only if I receive a message from another agent (no beacon) */
			/* the other agent must be: (i) committed and (ii) with option different than mine */
			if (received_message && (received_option != UNCOMMITTED && my_commitment != received_option)){
				P_inhibition = timeScaling * sigmaConstant;
			} else {
				P_inhibition = 0;
			}

			unsigned int P_abandonmentInt = (unsigned int)(P_abandonment*RANGE_RND)+1;
			unsigned int P_inhibitionInt = (unsigned int)(P_inhibition*RANGE_RND)+1;

			/* ABANDONMENT */
			if (P_abandonmentInt > 0 && random <= P_abandonmentInt){
				set_commitment(UNCOMMITTED, 0);
			}

			/* CROSS-INHIBITION */
			else if (P_inhibitionInt > 0 && random <= (P_abandonmentInt + P_inhibitionInt)){
				set_commitment(UNCOMMITTED, 0);
			}
		}

		received_message = false;
		discovered = false;
	}
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
   switch( current_motion_type ) {
   case TURN_LEFT:
   case TURN_RIGHT:
      if( kilo_ticks > last_motion_ticks + turning_ticks ) {
         /* start moving forward */
         last_motion_ticks = kilo_ticks;
         set_motion(FORWARD);
      }
      break;
   case FORWARD:
      if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
         /* perform a radnom turn */
         last_motion_ticks = kilo_ticks;
         if( rand_soft()%2 ) {
            set_motion(TURN_LEFT);
         }
         else {
            set_motion(TURN_RIGHT);
         }
         turning_ticks = rand_soft()%max_turning_ticks + 1;
      }
      break;
   case STOP:
   default:
      set_motion(STOP);
   }
}

/*-------------------------------------------------------------------*/
/* Function to broadcast the commitment message                     */
/*-------------------------------------------------------------------*/
void broadcast() {
   if( kilo_ticks > last_broadcast_ticks + broadcast_ticks ) {
      last_broadcast_ticks = kilo_ticks;

      /* set broadcast flag for transmission */
      broadcast_msg = true;
   }
}

/*-------------------------------------------------------------------*/
/* Callback function for message transmission                        */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
   if( broadcast_msg ) {
      return &message;
   }
   return 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void tx_message_success() {
   broadcast_msg = false;
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
	random_walk();
	broadcast();
	update_commitment();
}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
	kilo_init();
	kilo_message_tx = message_tx;
	kilo_message_tx_success = tx_message_success;
	kilo_message_rx=message_rx;
	kilo_start(setup, loop);

	return 0;
}
