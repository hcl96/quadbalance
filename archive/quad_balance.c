/**
* @file quad_balance.c
*
* This development code will aim to balance the full 6 DOF quadcopter dynamics using linear control
* Linear quadratic gaussian regulator will be used for the linear optimal controller
* Start Date: 2018/11/18
* Author: Steven Liang
* Organization: IARC Quadcopter Project, IEEE UCSD
*/

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <math.h>
#include <termios.h>
#include <time.h>

#define I2C_BUS 2
#define TIMESTEP 0.01
// function declarations

struct rawangle_t {
	// all angles with right hand rule. Phi: X, Theta: Y, Psi: Z. XYZ coordinate with NED axis.
	float phi_a[2];
	float phi_g[2];
	float theta_a[2];
	float theta_g[2];
	float psi_a[2];
	float psi_g[2];
} rawangle;

struct xhat_t {
	// estimated angles from Kalman filter
	float phi[2];
	float phi_dot[2];
	float theta[2];
	float theta_dot[2];
	float psi[2];
	float psi_dot[2];
} xhat;

void initTermios(int echo);
void resetTermios(void);
char getch(void);
void resetTermios(void);

void on_pause_press();
void on_pause_release();
void filter_initialize(); //initialize raw gyro array
void motorcontrol(float u1, float u2, float u3, float u4);
void angle_calc(struct rc_mpu_data_t data);
float filter_complementary(struct rawangle_t rawangle, struct xhat_t xhat);

void* thread1(void* ptr);
void* thread2(void* ptr);

static struct termios old, new;
clock_t begin;
pthread_t thread;


int verbose = 0;
rc_mpu_data_t data;

float phi_filtered;
float theta_filtered;

float u_control[4];
float power = 0.05;

float theta_a_filtered[2];
float theta_g_filtered[2];
float phi_a_filtered[2];
float phi_g_filtered[2];

int input[4];
int counter;

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// Following is initialization protocol with Robot Control Library
	if(rc_kill_existing_process(2.0) < -2) return -1;
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_make_pid_file();

	// Following is quadcopter balance initialization
	printf("\nQuadcopter Balance\n");
	printf("Initailizing ...\n");
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	if(rc_mpu_initialize(&data, conf)){
        fprintf(stderr,"rc_mpu_initialize_failed\n");
        return -1;
    }

    // Take initial reading of IMU
    rc_mpu_read_accel(&data);
    rc_mpu_read_gyro(&data);
    filter_initialize(); //initialize raw gyro array

    rc_servo_init();
	motorcontrol(0,0,0,0); //initialize ESC for ground zero
	pthread_create(&thread,NULL,thread1,NULL);
	pthread_create(&thread,NULL,thread2,NULL);
	printf("Initailization Complete. Begin control sequence\n");

	rc_set_state(RUNNING); // done initializing so set state to RUNNING

	printf("Quadcopter Controller\n");
	printf("|   u1   |   u2   |   u3   |   u4   | Power %% |   Phi   |   Theta   |\n");

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){

			// read IMU
			rc_mpu_read_accel(&data);
			rc_mpu_read_gyro(&data);

			// obtain raw angles from IMU data
			angle_calc(data);

			// obtain filtered angles from raw angles
			phi_filtered = filter_complementary(rawangle, xhat);
			//theta_filtered = filter_complementary(theta_a_raw, theta_g_raw, theta_a_filtered, theta_g_filtered);

			//using LQR
			float k = 0.25;
			// u_control[4] = 0.344*k*data.gyro[0]*(3.14159/180.0) + 0.528*k*phi_filtered + 0.342*k*data.gyro[1]*(3.14159/180.0) + 0.527*k*theta_filtered;
			// u_control[1] = 0.344*k*data.gyro[0]*(3.14159/180.0) + 0.528*k*phi_filtered - 0.342*k*data.gyro[1]*(3.14159/180.0) - 0.527*k*theta_filtered;
			// u_control[3] = -0.344*k*data.gyro[0]*(3.14159/180.0) - 0.528*k*phi_filtered + 0.342*k*data.gyro[1]*(3.14159/180.0) + 0.527*k*theta_filtered;
			// u_control[2] = -0.344*k*data.gyro[0]*(3.14159/180.0) - 0.528*k*phi_filtered - 0.342*k*data.gyro[1]*(3.14159/180.0) - 0.527*k*theta_filtered;

			// u_control[1] = -k*(0.7554*phi_filtered+0.0018*data.gyro[1]+0.7553*theta_filtered+0.0017*data.gyro[0]);
			// u_control[2] = -k*(-0.7554*phi_filtered-0.0018*data.gyro[1]+0.7553*theta_filtered+0.0017*data.gyro[0]);
			// u_control[3] = -k*(0.7554*phi_filtered+0.0018*data.gyro[1]-0.7553*theta_filtered-0.0017*data.gyro[0]);
			// u_control[4] = -k*(-0.7554*phi_filtered-0.0018*data.gyro[1]-0.7553*theta_filtered-0.0017*data.gyro[0]);

			//wait for comp filter to stabilize
			if (counter <= 500){
				motorcontrol(0,0,0,0);
			}
			else {
				motorcontrol(power+u_control[1],power+u_control[2],power+u_control[3],power+u_control[4]);
			}
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point
		rc_usleep(10000);
		counter+=1;
	}

	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	printf("\nProgram Exited Cleanly\n");
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

/**
 * @brief calculates angles phi(x), theta(y) from raw IMU acceleration 
 * @param rc_imu_data_t struct imu_data
 * @return none
 */
void angle_calc (struct rc_mpu_data_t data){
	rawangle.theta_a[1]= atan2f(data.accel[1],data.accel[2]);	//rotation around x axis, positive with right hand rule
	rawangle.theta_g[1] = rawangle.phi_g[0]+ (float) (TIMESTEP*data.gyro[0]*(3.14159/180.0));
	rawangle.phi_a[1]  = -atan2f(data.accel[0],sqrt(pow(data.accel[1],2)+pow(data.accel[2],2))); //rotation around y axis, positive with right hand rule
	rawangle.phi_g[1] = rawangle.theta_g[0]+ (float) (TIMESTEP*data.gyro[1]*(3.14159/180.0));

	//update raw gyro array
	rawangle.phi_g[0] = rawangle.phi_g[1];
	rawangle.theta_g[0] = rawangle.theta_g[1];
	rawangle.phi_a[0] = rawangle.phi_a[1];
	rawangle.theta_a[0] = rawangle.theta_a[1];
}

/**
 * @brief applies complementary filter to angle estimates from accelerometer and gyroscope
 * @param float angle_a_raw, angle_g_raw
 * @return none
 */
float filter_complementary(struct rawangle_t rawangle, struct xhat_t xhat){
	//CREATING DIGITAL FILTER
	float angle_filtered;
	float wc=15; // specify crossover freq
	// Low Pass Filter
	// angle_a_filtered[1] = (1/(200+wc))*((200-wc)*angle_a_filtered[0]+wc*angle_a_raw[1]+wc*angle_a_raw[0]);
	// angle_a_filtered[0] = angle_a_filtered[1];
	// angle_g_filtered[1] = ((200-wc)/(200+wc))*angle_g_filtered[0]+(200/(200+wc))*(angle_g_raw[1]-angle_g_raw[0]);
	// angle_g_filtered[0] = angle_g_filtered[1];
	// angle_filtered=angle_a_filtered[1]+angle_g_filtered[1];
	return angle_filtered;
}

/**
 * @brief executes the kalman filter
 * @param
 * @return estimated xhat angles
 */
float kalman (struct xhat_t xhat) {
	return 1;
}


/**
 * @brief applies u (from 0 to 1) to ESC
 * @param int u1, int u2, int u3, int u4
 * @return none
 */
void motorcontrol(float u1, float u2, float u3, float u4){
	input[0] = 1050+ (int) (700*u1);
	input[1] = 1050+ (int) (700*u2);
	input[2] = 1050+ (int) (700*u3);
	input[3] = 1050+ (int) (700*u4);
	rc_servo_send_pulse_us(1, input[0]);
	rc_servo_send_pulse_us(2, input[1]);
	rc_servo_send_pulse_us(3, input[2]);
	rc_servo_send_pulse_us(4, input[3]);
}

/**
 * @brief initialize filter array with zeros
 * @param none
 * @return none
 */
void filter_initialize(){
	rawangle.theta_a[0] = 0;
	rawangle.theta_g[0] = 0;
	rawangle.phi_a[0] 	= 0;
	rawangle.phi_g[0]	= 0;
	xhat.phi[0]			= 0;
	xhat.phi_dot[0]		= 0;
	xhat.theta[0]		= 0;
	xhat.theta_dot[0]	= 0;
	xhat.psi[0]			= 0;
	xhat.psi_dot[0]		= 0;
}



void* thread2(void* ptr){
	while(rc_get_state()!=EXITING){
		printf("\r");
		// printf("Reading IMU: \n");
		// printf("Phi-Accel: %.3f | Phi-Gyro: %.3f\n", phi_a_raw[1]*(180/3.14159), phi_g_raw[1]*(180/3.14159));
		// printf("T-Accel: %.3f | T-Gyro: %.3f\n", theta_a_raw[1]*(180/3.14159), theta_g_raw[1]*(180/3.14159));

		printf("|  %d  |  %d  |  %d  |  %d  |  %4.2f  |", input[0], input[1], input[2], input[3], (double)power*125);
		printf("  %4.3f  |   %4.3f   |", ((double)phi_filtered*(180/3.14159)), ((double)theta_filtered*(180/3.14159)));
		//printf("  %4.3f  |   %4.3f   |", xhat1, xhat2);

		fflush(stdout);
		rc_usleep(200000);
	}
	return NULL;
}

void* thread1(void* ptr)
{
	char move_c;
	while(rc_get_state()!=EXITING){
		move_c = getch();
			if (move_c=='w' && power < (float)0.8){
				power = power + (float)0.025;
			}
			else if (move_c=='s' && power > 0){
				power = power - (float)0.025;
			}
			else if (move_c=='`'){
				rc_set_state(EXITING);
			}
			rc_usleep(5000);
	}
	return NULL;
}




void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      new.c_lflag |= ECHO; /* set echo mode */
  } else {
      new.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}