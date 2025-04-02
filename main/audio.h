
// audio.c


void initSpiffs();

void locoAudioInit(void);
void startAudioThread();
void setVolume (int volume);
void codecRestart (int volume);

void toggleTestTone();
void toggleStereoTestTone();
void testToneOff();

// main.c - TODO cleanup

void initialiseWifi();
int startWifiFromCredentials();

void lockLVGL ();
void unlockLVGL ();
char *cleanName (char *s);

void keepAwake();
uint64_t millis ();

int getSettingsVolume();
void setSettingsVolume(int volume);
void memDebugB ();
void savePresets ();

void locoCallback(int e);
extern int newIpFlag;
extern int wifiDisconnectedFlag;

void cdump(unsigned char *buf, int len);
void startWifiSta2();


int isMounted ();
int mountSd(int format);
void initSDDetect ();
int getSDDetect ();
void sdPoll();

typedef struct {
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	int flag;
} waitlock_t;

