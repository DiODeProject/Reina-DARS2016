#include "kilolib.h"

#define UNCOMMITTED 0
#define OPT_BLUE 1
#define OPT_RED 2

#define	BEACON 77
#define	AGENT 21

/* Enum for boolean flags */
typedef enum {
	false = 0,
	true = 1,
} bool;

/* Robot type is BEACON */
uint8_t my_robot_type = BEACON;

/* Robot option and quality */
uint8_t my_option = OPT_BLUE;
uint8_t quality = 50;

/* Time between two messages (in clock-ticks of unknown length)
 * My current approximations says that (given the current loop-code)
 * a value of 120'000 ticks correspond roughly at 1s delay between two messages. */
uint32_t Tx = 120000;
uint32_t timesteps;

/* Flag for decision to broadcast a message */
bool broadcast_msg = false;

/* Message variable */
message_t message;

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
	if (my_option == OPT_BLUE) {
		set_color(RGB(0,0,1));
	} else {
		set_color(RGB(1,0,0));
	}

	timesteps = 0;

	message.type = NORMAL;
	message.data[0] = my_robot_type;
	message.data[1] = my_option;
	message.data[2] = quality;
	message.crc = message_crc(&message);
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
	if (my_option == OPT_BLUE) {
		set_color(RGB(0,0,1));
	} else {
		set_color(RGB(1,0,0));
	}
	delay(50);
	set_color(RGB(0, 0, 0));
}

/*-------------------------------------------------------------------*/
/* Function to broadcast the message every Tx seconds                */
/*-------------------------------------------------------------------*/
void broadcast() {
	timesteps = timesteps+1;
	if (timesteps >= Tx){
		timesteps = 0;
		broadcast_msg = true;
	}
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
	broadcast();
}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
	kilo_init();
	kilo_message_tx = message_tx;
	kilo_message_tx_success = tx_message_success;
	kilo_start(setup, loop);

	return 0;
}
