#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <lcm/lcm.h>
#include <lcmtypes/microstrain_ins_t.h>

#include <getopt.h>
#define ACC_ANG_MAG 0xCB
#define LENGTH_ACC_ANG_MAG 43

#define DANG_DVEL_MAG 0xD3
#define LENGTH_DANG_DVEL_MAG 43

#define ACC_ANG_MAG_ROT 0xCC
#define LENGTH_ACC_ANG_MAG_ROT 79

#define CONTINUOUS_MODE_COMMAND 0xC4
#define LENGTH_CONTINUOUS_MODE_ECHO 8

#define DELTA_ANG_VEL_DT 0.01

typedef unsigned char Byte;
using namespace std;

#define INPUT_BUFFER_SIZE 2000

//self structure
typedef struct {
  //communication variables
  int comm, status;

  //input buffer variables
  Byte input_buffer[INPUT_BUFFER_SIZE];
  Byte message_mode;
  BotRingBuf * read_buffer;
  char current_segment;
  int expected_segment_length;

  //packet variables
  int message_size;
  int message_start_byte;

  //boolean setting flags
  bool quiet, verbose;

  bool little_endian;

  int64_t utime_prev;

  BotParam * param;

  lcm_t * lcm;

} app_t;

bool systemLittleEndianCheck()
{
  short int word = 0x0001;
  char *bytes = (char *) &word;
  if (bytes[0] == 0)
    return false;
  else
    return true;
}

unsigned int make32UnsignedInt(Byte* pBytes, bool little_endian)
{

  unsigned int i;

  if (little_endian) {
    ((Byte*) (&i))[0] = pBytes[3];
    ((Byte*) (&i))[1] = pBytes[2];
    ((Byte*) (&i))[2] = pBytes[1];
    ((Byte*) (&i))[3] = pBytes[0];
  }
  else {
    ((Byte*) (&i))[0] = pBytes[0];
    ((Byte*) (&i))[1] = pBytes[1];
    ((Byte*) (&i))[2] = pBytes[2];
    ((Byte*) (&i))[3] = pBytes[3];
  }

  return i;
}

unsigned int make16UnsignedInt(const Byte* pBytes, bool little_endian)
{

  unsigned int i;

  if (little_endian) {
    ((Byte*) (&i))[0] = pBytes[1];
    ((Byte*) (&i))[1] = pBytes[0];

  }
  else {
    ((Byte*) (&i))[0] = pBytes[0];
    ((Byte*) (&i))[1] = pBytes[1];
  }

  return i;
}

float make32bitFloat(const Byte* pBytes, bool little_endian)
{
  float f = 0;

  if (little_endian) {
    ((Byte*) (&f))[0] = pBytes[3];
    ((Byte*) (&f))[1] = pBytes[2];
    ((Byte*) (&f))[2] = pBytes[1];
    ((Byte*) (&f))[3] = pBytes[0];
  }
  else {
    ((Byte*) (&f))[0] = pBytes[0];
    ((Byte*) (&f))[1] = pBytes[1];
    ((Byte*) (&f))[2] = pBytes[2];
    ((Byte*) (&f))[3] = pBytes[3];
  }

  return f;
}

void unpack32BitFloats(float * dest, const Byte * source_bytes, int length, bool little_endian)
{
  int ii;
  int byte_ind = 0;
  for (ii = 0; ii < length; ii++) {
    dest[ii] = make32bitFloat(&source_bytes[byte_ind], little_endian);
    byte_ind += 4;
  }
}

void convertFloatToDouble(double * dest, const float * source, int length)
{
  for (int ii = 0; ii < length; ii++) {
    dest[ii] = (double) source[ii];
  }
}

// ------------- Functions taken from microstrain driver for opening commport with proper settings and finding an attached microstrain device -----------

//scandev
//finds attached microstrain devices and prompts user for choice then returns selected portname
bool scandev(char * comm_port_name)
{

  FILE *instream;
  char devnames[255][255];//allows for up to 256 devices with path links up to 255 characters long each
  int devct = 0; //counter for number of devices
  int i = 0;
  int j = 0;
  int userchoice = 0;

  char command[] = "find /dev/serial -print | grep -i microstrain";//search /dev/serial for microstrain devices

  printf("Searching for devices...\n");

  instream = popen(command, "r");//execute piped command in read mode

  if (!instream) {//SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
    printf("ERROR BROKEN PIPELINE %s\n", command);
    return false;
  }

  for (i = 0; i < 255 && (fgets(devnames[i], sizeof(devnames[i]), instream)); i++) {//load char array of device addresses
    ++devct;
  }

  for (i = 0; i < devct; i++) {
    for (j = 0; j < sizeof(devnames[i]); j++) {
      if (devnames[i][j] == '\n') {
        devnames[i][j] = '\0';//replaces newline inserted by pipe reader with char array terminator character
        break;//breaks loop after replacement
      }
    }
    printf("Device Found:\n%d: %s\n", i, devnames[i]);
  }

  //CHOOSE DEVICE TO CONNECT TO AND CONNECT TO IT (IF THERE ARE CONNECTED DEVICES)

  if (devct > 0) {
    printf("Number of devices = %d\n", devct);
    if (devct > 1) {
      printf("Please choose the number of the device to connect to (0 to %i):\n", devct - 1);
      while (scanf("%i", &userchoice) == 0 || userchoice < 0 || userchoice > devct - 1) {//check that there's input and in the correct range
        printf("Invalid choice...Please choose again between 0 and %d:\n", devct - 1);
        getchar();//clear carriage return from keyboard buffer after invalid choice
      }
    }
    strcpy(comm_port_name, devnames[userchoice]);
    return true;
  }
  else {
    printf("No MicroStrain devices found.\n");
    return false;
  }

}

// OpenComPort
// Opens a com port with the correct settings for communicating with a MicroStrain 3DM-GX3-25 sensor

int OpenComPort(const char* comPortPath)
{

  int comPort = open(comPortPath, O_RDWR | O_NOCTTY);

  if (comPort == -1) { //Opening of port failed

    printf("Unable to open com Port %s\n Errno = %i\n", comPortPath, errno);
    return -1;

  }

  //Get the current options for the port...
  struct termios options;
  tcgetattr(comPort, &options);

  //set the baud rate to 115200
  int baudRate = B115200;
  cfsetospeed(&options, baudRate);
  cfsetispeed(&options, baudRate);

  //set the number of data bits.
  options.c_cflag &= ~CSIZE; // Mask the character size bits
  options.c_cflag |= CS8;

  //set the number of stop bits to 1
  options.c_cflag &= ~CSTOPB;

  //Set parity to None
  options.c_cflag &= ~PARENB;

  //set for non-canonical (raw processing, no echo, etc.)
  options.c_iflag = IGNPAR; // ignore parity check close_port(int
  options.c_oflag = 0; // raw output
  options.c_lflag = 0; // raw input

  //Time-Outs -- won't work with NDELAY option in the call to open
  options.c_cc[VMIN] = 0; // block reading until RX x characers. If x = 0, it is non-blocking.
  options.c_cc[VTIME] = 100; // Inter-Character Timer -- i.e. timeout= x*.1 s

  //Set local mode and enable the receiver
  options.c_cflag |= (CLOCAL | CREAD);

  tcflush(comPort, TCIOFLUSH);

  //Set the new options for the port...
  int status = tcsetattr(comPort, TCSANOW, &options);

  if (status != 0) { //For error message

    printf("Configuring comport failed\n");
    return status;

  }

  //Purge serial port buffers
  tcflush(comPort, TCIOFLUSH);

  return comPort;

}

/*
 * prints byte arrays in hex
 */
void print_array_char_hex(const unsigned char * array, int length)
{
  int ii;
  for (ii = 0; ii < length; ii++) {
    fprintf(stderr, "%02X ", (unsigned char) array[ii]);
  }
  fprintf(stderr, "\n");
}

/*
 * checksum function
 */
unsigned short cksum(const Byte * packet_bytes, int packet_length)
{
  unsigned short check_sum_val = 0;
  for (int ii = 0; ii < packet_length - 2; ii++) {
    check_sum_val += packet_bytes[ii];
  }
  return check_sum_val;
}

bool handle_message(app_t* app)
{

  microstrain_ins_t ins_message;
  memset(&ins_message, 0, sizeof(ins_message));
  int ins_timer;
  float vals[9];

  if (app->verbose) {
    fprintf(stderr, "Received data packet:\n");
    print_array_char_hex((unsigned char *) app->input_buffer, app->message_size);
  }

  bool got_quat = false;
  int success = 0;
  switch (app->message_start_byte) {
  case ACC_ANG_MAG_ROT:
    {
      if (app->message_mode != ACC_ANG_MAG_ROT && !app->quiet)
        printf("error, received ACC_ANG_MAG_ROT instead of ACC_ANG_MAG\n");

      double rot[9];
      unpack32BitFloats(vals, &app->input_buffer[37], 9, app->little_endian);
      convertFloatToDouble(rot, vals, 9);
      bot_matrix_to_quat(rot, ins_message.quat);
      got_quat = true;
      //fall into standard ins message handling
    }
  case ACC_ANG_MAG:
    {
      if (app->message_mode == ACC_ANG_MAG_ROT && !got_quat && !app->quiet)
        printf("error, received ACC_ANG_MAG instead of ACC_ANG_MAG_ROT (no quat received)\n");

      //get the data we care about
      unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
      convertFloatToDouble(ins_message.accel, vals, 3);

      unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
      convertFloatToDouble(ins_message.gyro, vals, 3);

      unpack32BitFloats(vals, &app->input_buffer[25], 3, app->little_endian);
      convertFloatToDouble(ins_message.mag, vals, 3);

      //ins internal timer, currently not used
      ins_timer = make32UnsignedInt(&app->input_buffer[37], app->little_endian);

      ins_message.device_time = ((double) ins_timer) / 62500.0;
      ins_message.utime = bot_timestamp_now();
      microstrain_ins_t_publish(app->lcm, "MICROSTRAIN_INS", &ins_message);
      break;
    }

  case DANG_DVEL_MAG:
    {
      if (app->message_mode != DANG_DVEL_MAG && !app->quiet)
        printf("error: received unexpecte DANG_DVEL_MAG message\n");

      //get the data we care about
      unpack32BitFloats(vals, &app->input_buffer[1], 3, app->little_endian);
      convertFloatToDouble(ins_message.gyro, vals, 3);
      bot_vector_scale_3d(ins_message.gyro, 1 / DELTA_ANG_VEL_DT);

      unpack32BitFloats(vals, &app->input_buffer[13], 3, app->little_endian);
      convertFloatToDouble(ins_message.accel, vals, 3);
      bot_vector_scale_3d(ins_message.accel, 1 / DELTA_ANG_VEL_DT);

      unpack32BitFloats(vals, &app->input_buffer[25], 3, app->little_endian);
      convertFloatToDouble(ins_message.mag, vals, 3);

      //ins internal timer, currently not used
      ins_timer = make32UnsignedInt(&app->input_buffer[37], app->little_endian);

      ins_message.device_time = ((double) ins_timer) / 62500.0;
      ins_message.utime = bot_timestamp_now();
      microstrain_ins_t_publish(app->lcm, "MICROSTRAIN_INS", &ins_message);
      break;
    }
  case CONTINUOUS_MODE_COMMAND:
    {
      fprintf(stderr, "received continuous mode command echo\n");
      break;
    }
  default:
    {
      if (!app->quiet)
        fprintf(stderr, "Unknown message start byte: %d\n", app->message_start_byte);
    }
  }

  return true;
}

/*
 * gets data packets out of circular buffer.
 *
 * has 2 states, either looking for the header, or looking for the data + checksum bytes.  it waits until it
 * has all expected bytes before taking appropriate action.
 */
void unpack_packets(app_t * app)
{

  while (bot_ringbuf_available(app->read_buffer) >= app->expected_segment_length) {
    switch (app->current_segment) {
    case 's':
      bot_ringbuf_peek(app->read_buffer, 1, (char *) &app->message_start_byte);

      if (app->verbose)
        fprintf(stderr, "received message start byte: id=%d\n", app->message_start_byte);

      app->current_segment = 'p';

      switch (app->message_start_byte) {
      case ACC_ANG_MAG:
        app->expected_segment_length = LENGTH_ACC_ANG_MAG;
        break;
      case ACC_ANG_MAG_ROT:
        app->expected_segment_length = LENGTH_ACC_ANG_MAG_ROT;
        break;
      case DANG_DVEL_MAG:
        app->expected_segment_length = LENGTH_DANG_DVEL_MAG;
        break;
      case CONTINUOUS_MODE_COMMAND:
        app->expected_segment_length = LENGTH_CONTINUOUS_MODE_ECHO;
        break;
      default:
        if (!app->quiet) {
          fprintf(stderr, "no match for message start byte %d\n", app->message_start_byte);
        }
        //read a byte and continue if we don't have a match
        bot_ringbuf_read(app->read_buffer, 1, (char *) &app->message_start_byte);
        app->current_segment = 's';
      }
      break;
    case 'p':
      bot_ringbuf_read(app->read_buffer, app->expected_segment_length, (char *) app->input_buffer);
      unsigned short transmitted_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2],
          app->little_endian);
      unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);
      if (computed_cksum != transmitted_cksum) {
        if (!app->quiet)
          fprintf(stderr, "Failed check sum! got: %d, expected: %d\n", transmitted_cksum, computed_cksum);
        break;
      }

      if (app->verbose)
        fprintf(stderr, "Passed checksum,handling message\n");

      bool message_success = handle_message(app);
      if (!message_success && !app->quiet)
        fprintf(stderr, "Message handling failed\n");

      app->current_segment = 's';
      app->expected_segment_length = 1;
      break;
    }

  }
}

/*
 * reads serial bytes from ardu as they become available from g_io_watch and then writes them into the circular buffer
 * and calls unpack_packets
 *
 */
static gboolean serial_read_handler(GIOChannel * source, GIOCondition condition, void * user)
{
  app_t * app = (app_t *) user;

  static char middle_buffer[INPUT_BUFFER_SIZE];

  //get number of bytes available
  int available = 0;

  if (ioctl(app->comm, FIONREAD, &available) != 0) {
    if (!app->quiet)
      fprintf(stderr, "ioctl check for bytes available didn't return 0, breaking read\n");
    return TRUE;
  }

  if (available > INPUT_BUFFER_SIZE) {
    if (!app->quiet)
      fprintf(stderr, "too many bytes available: %d, flushing input buffer\n", available);
    tcflush(app->comm, TCIFLUSH);
    return TRUE;
  }

  int num_read = read(app->comm, middle_buffer, available);

  if (num_read != available) {
    if (!app->quiet)
      fprintf(stderr, "warning, read %d of %d available bytes\n", num_read, available);
  }

  if (num_read > 0) {
    bot_ringbuf_write(app->read_buffer, num_read, middle_buffer);
  }

  unpack_packets(app);

  return TRUE;
}

static void usage(const char *progname)
{
  char *basename = g_path_get_basename(progname);
  printf("Usage: %s [options]\n"
    "\n"
    "Options:\n"
    "\n"
    "    -h, --help                Shows this help text and exits\n"
    "    -v, --verbose\n"
    "    -q, --quiet\n"
    "    -c, --comm                specify comm port manuall (default will try to find attached microstrain)\n"
    "    -r, --quat                publish quaternion as well (will use more comm bandwidth)\n"
    "    -d, --delta               publish delta angle and delta velocity vectors normalized by dt: %f\n"
    "\n", basename, DELTA_ANG_VEL_DT);
  free(basename);
  exit(1);
}

int main(int argc, char **argv)
{

  app_t * app = (app_t *) calloc(1, sizeof(app_t));
  app->little_endian = systemLittleEndianCheck();

  //default settings
  app->verbose = 0;
  app->quiet = 0;
  app->message_mode = ACC_ANG_MAG;

  bool auto_comm = true;

  char user_comm_port_name[60];

  const char *optstring = "hvqc:rd";

  struct option long_opts[] = { { "help", no_argument, 0, 'h' },
      { "verbose", no_argument, 0, 'v' },
      { "quiet", no_argument, 0, 'q' },
      { "comm", required_argument, 0, 'c' },
      { "quat", no_argument, 0, 'r' },
      { "delta", no_argument, 0, 'd' },
      { 0, 0, 0, 0 } };

  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      break;
    case 'v':
      app->verbose = 1;
      break;
    case 'q':
      app->quiet = 1;
      break;
    case 'c':
      auto_comm = false;
      strcpy(user_comm_port_name, optarg);
      break;
    case 'r':
      app->message_mode = ACC_ANG_MAG_ROT;
      break;
    case 'd':
      app->message_mode = DANG_DVEL_MAG;
      break;
    default:
      usage(argv[0]);
      break;
    }
  }

  if (optind < argc - 1) {
    usage(argv[0]);
  }

  GMainLoop * mainloop = g_main_loop_new(NULL, FALSE);
  app->lcm = bot_lcm_get_global(NULL);
  app->utime_prev = bot_timestamp_now();
  //  app->param = bot_param_new_from_server(app->lcm, 1);

  app->read_buffer = bot_ringbuf_create(INPUT_BUFFER_SIZE);

  char comm_port_name[255];
  if (auto_comm)
    scandev(comm_port_name);
  else
    strcpy(comm_port_name, user_comm_port_name);

  // comm initialization
  app->comm = OpenComPort(comm_port_name);
  if (app->comm < 0) {
    exit(1);
  }

  char set_mode_string[] = { 0xC4, 0xC1, 0x29, app->message_mode };
  cout << "setting continuous mode\n";
  int written = write(app->comm, set_mode_string, 4);
  if (written!=4){
    printf("Error writing command to set continuous mode");
    exit(1);
  }

  app->current_segment = 's';
  app->expected_segment_length = 1;
  GIOChannel * ioc = g_io_channel_unix_new(app->comm);
  g_io_add_watch_full(ioc, G_PRIORITY_HIGH, G_IO_IN, (GIOFunc) serial_read_handler, (void *) app, NULL);

  g_main_loop_run(mainloop);
  return 0;
}

